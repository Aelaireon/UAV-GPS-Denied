#!/usr/bin/env python3
"""
optical_flow_node.py
Drone ego-motion velocity + position estimation using rpicam-vid (RPi 5 / Ubuntu 24.04).
Replaces the ROS2 Image subscription with a background subprocess reader
so no camera ROS driver is needed — rpicam-vid pipes MJPEG directly.

Subscribes : /uav/mavros/rangefinder_sub  (sensor_msgs/Range   — TFMini Plus)
             /uav/mavros/imu/data         (sensor_msgs/Imu     — attitude quaternion + angular rates)
Publishes  : /drone/optical_flow_vel      (geometry_msgs/TwistStamped)
             /uav/mavros/vision_pose/pose (geometry_msgs/PoseStamped — integrated x/y + TFMini z)
Service    : /drone/reset_pose            (std_srvs/Empty) — zero the x/y integrator

IMU is used for two corrections:
  1. Tilt-corrected altitude: TFMini measures slant range when pitched/rolled;
     true_alt = range x cos(roll) x cos(pitch) using the IMU quaternion.
  2. Rotational flow compensation: pitching/rolling in place causes apparent
     pixel motion. Apparent velocity error = omega x true_alt (focal length
     cancels). Subtracted from vx/vy before integration.

Position is dead-reckoning (integrated velocity) and will drift over time.
dt is measured from the ROS clock each callback so timer jitter does not corrupt the integral.
"""

import subprocess
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range, Imu
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


# ── Camera settings ───────────────────────────────────────────────────────────
CAMERA_INDEX = 1
FRAME_WIDTH  = 1280
FRAME_HEIGHT = 720
FPS          = 30
# ─────────────────────────────────────────────────────────────────────────────


def make_rpicam_proc(camera_index: int) -> subprocess.Popen:
    """Spawn rpicam-vid MJPEG streamer."""
    cmd = [
        "rpicam-vid",
        "--camera",    str(camera_index),
        "--codec",     "mjpeg",
        "-t",          "0",
        "--width",     str(FRAME_WIDTH),
        "--height",    str(FRAME_HEIGHT),
        "--framerate", str(FPS),
        "--nopreview",
        "--gain",      "2.0",
        "--awb",       "indoor",
        "--inline",
        "-o",          "-",
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)


class CameraReader(threading.Thread):
    """
    Background thread: continuously drains rpicam-vid stdout and keeps
    the most recent decoded frame ready.
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
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                    with self._lock:
                        self._frame = frame

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None


class OpticalFlowNode(Node):

    def __init__(self):
        super().__init__('optical_flow_node')

        # ── Camera intrinsics ─────────────────────────────────────────────────
        self.fx = 400.0
        self.fy = 400.0

        # ── State ────────────────────────────────────────────────────────────
        self.altitude    = 1.0      # raw TFMini range (metres)
        self._imu        = None     # latest sensor_msgs/Imu message
        self.prev_gray   = None
        self.prev_pts    = None
        self._last_stamp = None     # rclpy.time.Time of previous callback

        # Integrated x/y position (metres, drone-local horizontal plane).
        # z comes directly from tilt-corrected TFMini — no integration there.
        self._pos_x = 0.0
        self._pos_y = 0.0

        # Position covariance grows with distance to signal drift to an EKF.
        self._BASE_POS_COV   = 0.01
        self._COV_GROW_RATE  = 0.0002
        self._pos_cov        = self._BASE_POS_COV
        self._dist_travelled = 0.0

        # ── rpicam-vid subprocess + reader thread ─────────────────────────────
        self._proc = make_rpicam_proc(CAMERA_INDEX)
        self._cam  = CameraReader(self._proc)
        self._cam.start()
        self.get_logger().info(
            f"rpicam-vid started (camera {CAMERA_INDEX}, "
            f"{FRAME_WIDTH}x{FRAME_HEIGHT} @ {FPS} fps)"
        )
        
        # QoS for best-effort topics (camera data, IMU, rangefinder)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.create_subscription(
            Range, '/uav/mavros/rangefinder_sub',
            lambda msg: setattr(self, 'altitude', msg.range),
            10,
        )

        # Raw IMU — attitude quaternion for tilt correction,
        # angular_velocity for rotational flow compensation.
        self.create_subscription(
            Imu, '/uav/mavros/imu/data',
            lambda msg: setattr(self, '_imu', msg),
            qos,
        )

        self.pub_vel = self.create_publisher(
            TwistStamped, '/drone/optical_flow_vel', 10)

        self.pub_pose = self.create_publisher(
            PoseStamped, '/uav/mavros/vision_pose/pose', 10)

        self.create_service(
            Empty, '/drone/reset_pose', self._reset_pose_cb)

        self.timer = self.create_timer(1.0 / FPS, self._flow_callback)

    # ── Main processing loop ──────────────────────────────────────────────────

    def _flow_callback(self):
        frame = self._cam.get_frame()
        if frame is None:
            return

        now  = self.get_clock().now()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # First frame — seed features and clock
        if self.prev_gray is None:
            self.prev_gray   = gray
            self.prev_pts    = self._detect_features(gray)
            self._last_stamp = now
            return

        # Actual dt — avoids velocity spikes from timer jitter
        dt = (now - self._last_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1.0 / FPS

        if self.prev_pts is None or len(self.prev_pts) < 10:
            self.prev_pts    = self._detect_features(gray)
            self.prev_gray   = gray
            self._last_stamp = now
            return

        # ── Lucas-Kanade sparse optical flow ──────────────────────────────────
        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None)

        good_prev = self.prev_pts[status == 1]
        good_curr = curr_pts[status == 1]

        if len(good_prev) < 10:
            self.prev_pts    = self._detect_features(gray)
            self.prev_gray   = gray
            self._last_stamp = now
            return

        flow      = good_curr - good_prev
        mean_flow = np.mean(flow, axis=0)   # (dx_px, dy_px)

        # ── IMU corrections ───────────────────────────────────────────────────
        if self._imu is not None:
            q = self._imu.orientation

            # Quaternion → roll / pitch (ZYX, right-hand convention)
            sinr = 2.0 * (q.w * q.x + q.y * q.z)
            cosr = 1.0 - 2.0 * (q.x ** 2 + q.y ** 2)
            roll  = float(np.arctan2(sinr, cosr))

            sinp = 2.0 * (q.w * q.y - q.z * q.x)
            pitch = float(np.arcsin(np.clip(sinp, -1.0, 1.0)))

            # 1. Tilt-corrected altitude
            #    TFMini measures slant range when tilted — project to vertical.
            true_alt = float(self.altitude * np.cos(roll) * np.cos(pitch))

            # 2. Rotational flow compensation
            #    Apparent velocity error = omega (rad/s) x true_alt (m).
            #    Focal length cancels in the derivation so it is not needed here.
            #    Same axis remap as optical flow: pitch_rate -> X, roll_rate -> Y.
            omega      = self._imu.angular_velocity
            rot_comp_x = omega.y * true_alt   # pitch rate -> fake forward (X)
            rot_comp_y = omega.x * true_alt   # roll  rate -> fake lateral (Y)
        else:
            true_alt   = self.altitude
            rot_comp_x = 0.0
            rot_comp_y = 0.0

        # ── Scale to m/s ──────────────────────────────────────────────────────
        # Axis remap: image-x -> body-Y (left), image-y -> body-X (forward).
        # Signs negated to match observed drone motion direction.
        # Rotational compensation subtracted after scaling.
        vx = -float((mean_flow[1] / self.fy) * true_alt / dt) - rot_comp_x
        vy = -float((mean_flow[0] / self.fx) * true_alt / dt) - rot_comp_y

        # ── Integrate position ────────────────────────────────────────────────
        dx = vx * dt
        dy = vy * dt
        self._pos_x += dx
        self._pos_y += dy

        step = float(np.hypot(dx, dy))
        self._dist_travelled += step
        self._pos_cov += step * self._COV_GROW_RATE

        stamp = now.to_msg()

        # ── Velocity topic ────────────────────────────────────────────────────
        twist = TwistStamped()
        twist.header.stamp    = stamp
        twist.header.frame_id = 'drone_base_link'
        twist.twist.linear.x  = vx
        twist.twist.linear.y  = vy
        self.pub_vel.publish(twist)

        # ── Vision pose topic ─────────────────────────────────────────────────
        # Sent to MAVROS vision_pose so the FCU can fuse it in its EKF.
        # z uses tilt-corrected altitude — absolute and drift-free.
        # Orientation is identity; yaw is not estimated from optical flow alone.
        pose = PoseStamped()
        pose.header.stamp    = stamp
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self._pos_x
        pose.pose.position.y = self._pos_y
        pose.pose.position.z = true_alt
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)

        # ── Roll forward ──────────────────────────────────────────────────────
        self.prev_gray   = gray
        self.prev_pts    = good_curr.reshape(-1, 1, 2)
        self._last_stamp = now

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

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _detect_features(gray):
        return cv2.goodFeaturesToTrack(
            gray, maxCorners=200, qualityLevel=0.01, minDistance=10)

    def destroy_node(self):
        self._proc.terminate()
        self.get_logger().info('rpicam-vid subprocess terminated.')
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