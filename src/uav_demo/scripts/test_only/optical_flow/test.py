#!/usr/bin/env python3
"""
optical_flow_node.py
Drone ego-motion velocity + position estimation using rpicam-vid (RPi 5 / Ubuntu 24.04).
Replaces the ROS2 Image subscription with a background subprocess reader
so no camera ROS driver is needed — rpicam-vid pipes MJPEG directly.

Subscribes : /uav/mavros/rangefinder_sub  (sensor_msgs/Range      — TFMini Plus)
Publishes  : /drone/optical_flow_vel  (geometry_msgs/TwistStamped)
             /drone/optical_flow_odom (nav_msgs/Odometry)
               └─ pose.pose.position  : integrated x/y + TFMini z
               └─ twist.twist.linear  : current vx/vy (same as vel topic)
Service    : /drone/reset_pose        (std_srvs/Empty) — zero the x/y integrator

Position is dead-reckoning (integrated velocity) and will drift over time.
For long missions fuse /drone/optical_flow_odom with GPS or ArUco fixes in an EKF.
dt is measured from the ROS clock each callback so timer jitter doesn't corrupt the integral.
"""

import subprocess
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


# ── Camera settings (match your hardware) ────────────────────────────────────
CAMERA_INDEX = 1          # right camera (use 0 for the one facing down / forward)
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
FPS          = 30
# ─────────────────────────────────────────────────────────────────────────────


def make_rpicam_proc(camera_index: int) -> subprocess.Popen:
    """Spawn rpicam-vid MJPEG streamer — same pattern as the stereo ArUco script."""
    cmd = [
        "rpicam-vid",
        "--camera",    str(camera_index),
        "--codec",     "mjpeg",
        "-t",          "0",               # run indefinitely
        "--width",     str(FRAME_WIDTH),
        "--height",    str(FRAME_HEIGHT),
        "--framerate", str(FPS),
        "--nopreview",
        "--gain",      "2.0",
        "--awb",       "indoor",
        "--inline",
        "-o",          "-",               # stdout
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)


class CameraReader(threading.Thread):
    """
    Background thread: continuously drains rpicam-vid stdout and keeps
    the most recent decoded frame ready.  Uses the same JPEG-boundary
    search as the stereo ArUco script.
    """

    def __init__(self, proc: subprocess.Popen):
        super().__init__(daemon=True)
        self._proc   = proc
        self._buffer = b""
        self._frame  = None
        self._lock   = threading.Lock()

    def run(self):
        while True:
            chunk = self._proc.stdout.read(4096)
            if not chunk:
                break
            self._buffer += chunk
            a = self._buffer.find(b'\xff\xd8')
            b = self._buffer.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg          = self._buffer[a:b + 2]
                self._buffer = self._buffer[b + 2:]
                frame = cv2.imdecode(
                    np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    # Rotate 180° — same correction as the stereo script
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                    with self._lock:
                        self._frame = frame

    def get_frame(self):
        """Return latest frame (or None if not yet available)."""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None


class OpticalFlowNode(Node):

    def __init__(self):
        super().__init__('optical_flow_node')

        # ── Camera intrinsics (replace with your calibration YAML values) ──
        self.fx = 982.0
        self.fy = 982.0

        # ── State ────────────────────────────────────────────────────────────
        self.altitude  = 1.0    # metres — updated by TFMini
        self.prev_gray = None
        self.prev_pts  = None
        self._last_stamp = None             # rclpy.time.Time of previous callback

        # Integrated x/y position (metres, drone-local horizontal plane).
        # z is taken directly from TFMini — no integration needed there.
        self._pos_x = 0.0
        self._pos_y = 0.0

        # Diagonal position covariance (m²) — grows with distance travelled.
        self._BASE_POS_COV   = 0.01         # 10 cm std-dev at origin
        self._COV_GROW_RATE  = 0.0002       # added per metre of travel
        self._pos_cov        = self._BASE_POS_COV
        self._dist_travelled = 0.0

        # ── rpicam-vid subprocess + reader thread ────────────────────────────
        self._proc  = make_rpicam_proc(CAMERA_INDEX)
        self._cam   = CameraReader(self._proc)
        self._cam.start()
        self.get_logger().info(
            f"rpicam-vid started (camera {CAMERA_INDEX}, "
            f"{FRAME_WIDTH}×{FRAME_HEIGHT} @ {FPS} fps)"
        )

        # ── ROS interfaces ───────────────────────────────────────────────────
        self.sub_alt = self.create_subscription(
            Range, '/uav/mavros/rangefinder_sub',
            lambda msg: setattr(self, 'altitude', msg.range),
            10,
        )

        self.pub_flow = self.create_publisher(
            TwistStamped, '/drone/optical_flow_vel', 10)

        # self.pub_odom = self.create_publisher(
        #     Odometry, '/drone/optical_flow_odom', 10)
        self.pub_pose = self.create_publisher(
            PoseStamped, '/uav/mavros/vision_pose/pose', 10)

        self.srv_reset = self.create_service(
            Empty, '/drone/reset_pose', self._reset_pose_cb)

        # Timer drives the optical-flow loop at camera FPS
        self.timer = self.create_timer(1.0 / FPS, self._flow_callback)

    # ── Main processing loop ─────────────────────────────────────────────────

    def _flow_callback(self):
        frame = self._cam.get_frame()
        if frame is None:
            return                          # camera not ready yet

        now  = self.get_clock().now()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # First frame — seed feature points and clock
        if self.prev_gray is None:
            self.prev_gray   = gray
            self.prev_pts    = self._detect_features(gray)
            self._last_stamp = now
            return

        # Actual elapsed time since last callback (handles jitter correctly)
        dt = (now - self._last_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1.0 / FPS              # fallback if clock hiccup

        # Need enough features to track
        if self.prev_pts is None or len(self.prev_pts) < 10:
            self.prev_pts    = self._detect_features(gray)
            self.prev_gray   = gray
            self._last_stamp = now
            return

        # ── Lucas-Kanade sparse optical flow ─────────────────────────────────
        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None)

        good_prev = self.prev_pts[status == 1]
        good_curr = curr_pts[status == 1]

        if len(good_prev) < 10:
            self.prev_pts    = self._detect_features(gray)
            self.prev_gray   = gray
            self._last_stamp = now
            return

        # Mean pixel displacement this frame
        flow      = good_curr - good_prev
        mean_flow = np.mean(flow, axis=0)   # (dx_px, dy_px)

        # Scale to m/s:  v = (dp_px / focal_px) * altitude_m / dt
        # Using actual dt instead of hard-coded FPS so jitter doesn't
        # produce incorrect velocity spikes.
        #
        # Axis remap — camera is mounted 90° rotated relative to drone body:
        #   image-x displacement → drone LEFT  (body Y+)
        #   image-y displacement → drone FORWARD (body X+)
        # Swap here so published axes match ROS base_link convention:
        #   linear.x = forward/back,  linear.y = left/right
        vx = float((mean_flow[1] / self.fy) * self.altitude / dt)   # forward  (was image-y)
        vy = float((mean_flow[0] / self.fx) * self.altitude / dt)   # left     (was image-x)
        vx = -vx
        vy = -vy

        # ── Integrate position ────────────────────────────────────────────────
        dx = vx * dt
        dy = vy * dt
        self._pos_x += dx
        self._pos_y += dy

        step = float(np.hypot(dx, dy))
        self._dist_travelled += step
        self._pos_cov += step * self._COV_GROW_RATE

        stamp = now.to_msg()

        # ── Velocity topic (unchanged) ────────────────────────────────────────
        twist = TwistStamped()
        twist.header.stamp    = stamp
        twist.header.frame_id = 'drone_base_link'
        twist.twist.linear.x  = vx
        twist.twist.linear.y  = vy
        self.pub_flow.publish(twist)

        # ── Odometry topic ────────────────────────────────────────────────────
        # Pose  : integrated x/y + TFMini altitude as z (absolute, no drift)
        # Twist : current vx/vy (same values as above)
        odom = Odometry()
        odom.header.stamp     = stamp
        odom.header.frame_id  = 'odom'          # world-fixed integration frame
        odom.child_frame_id   = 'drone_base_link'

        odom.pose.pose.position.x = self._pos_x
        odom.pose.pose.position.y = self._pos_y
        odom.pose.pose.position.z = float(self.altitude)
        # Identity quaternion — yaw not estimated from optical flow alone
        odom.pose.pose.orientation.w = 1.0

        # 6×6 row-major covariance.  Off-diagonals zero; diagonal entries:
        # [cov_xx, cov_yy, cov_zz, cov_roll, cov_pitch, cov_yaw]
        cov_z   = 0.005   # TFMini is accurate; 7 cm std-dev
        cov_rot = 1e6     # rotation not estimated — mark as unknown
        pc = [0.0] * 36
        pc[0]  = self._pos_cov   # x
        pc[7]  = self._pos_cov   # y
        pc[14] = cov_z           # z
        pc[21] = cov_rot         # roll
        pc[28] = cov_rot         # pitch
        pc[35] = cov_rot         # yaw
        odom.pose.covariance = pc

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        # Velocity covariance — simple fixed estimate; tune after real tests
        vel_cov = 0.02   # ~14 cm/s std-dev
        tc = [0.0] * 36
        tc[0]  = vel_cov
        tc[7]  = vel_cov
        tc[14] = 1e6     # vz unknown
        tc[21] = 1e6
        tc[28] = 1e6
        tc[35] = 1e6
        odom.twist.covariance = tc
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose   = odom.pose.pose

        # self.pub_odom.publish(odom)
        self.pub_pose.publish(pose)

        # ── Roll forward ─────────────────────────────────────────────────────
        self.prev_gray   = gray
        self.prev_pts    = good_curr.reshape(-1, 1, 2)
        self._last_stamp = now

        # Refresh feature pool when it gets thin
        if len(self.prev_pts) < 50:
            self.prev_pts = self._detect_features(gray)

    # ── Reset service ─────────────────────────────────────────────────────────

    def _reset_pose_cb(self, _request, response):
        """ros2 service call /drone/reset_pose std_srvs/srv/Empty"""
        self._pos_x          = 0.0
        self._pos_y          = 0.0
        self._pos_cov        = self._BASE_POS_COV
        self._dist_travelled = 0.0
        self.get_logger().info('Position integrator reset to origin.')
        return response

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _detect_features(gray):
        return cv2.goodFeaturesToTrack(
            gray, maxCorners=200, qualityLevel=0.01, minDistance=10)

    def destroy_node(self):
        self._proc.terminate()
        self.get_logger().info("rpicam-vid subprocess terminated.")
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()