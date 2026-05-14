#!/usr/bin/env python3
import subprocess
import threading
import numpy as np
import cv2
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range, Imu
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_srvs.srv import Empty

# ── Load Camera Configuration ────────────────────────────────────────────────
# Using the specific absolute path from your previous message
NPZ_PATH = "/home/uav/UAV-GPS-Denied/src/uav_demo/scripts/test_only/calib_final_036/calib_intrinsics.npz"

if os.path.exists(NPZ_PATH):
    with np.load(NPZ_PATH) as data:
        # Match the keys used in your ChArUco calibration script
        K = data['K_l'].astype(np.float32)
        D = data['D_l'].astype(np.float32)
    print(f"Successfully loaded K_l and D_l from {NPZ_PATH}")
else:
    print(f"CRITICAL: {NPZ_PATH} not found!")
    # Fallback to prevent crash, but values will be incorrect
    K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]], dtype=np.float32)
    D = np.zeros(5, dtype=np.float32)

# ── Camera settings ───────────────────────────────────────────────────────────
# Ensure these match the resolution used in your calibration script (640x480)
CAMERA_INDEX = 1 # Your node uses index 1, your calib script used 0/1
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
FPS          = 30

def make_rpicam_proc(camera_index: int) -> subprocess.Popen:
    cmd = [
        "rpicam-vid", "--camera", str(camera_index), "--codec", "mjpeg",
        "-t", "0", "--width", str(FRAME_WIDTH), "--height", str(FRAME_HEIGHT),
        "--framerate", str(FPS), "--nopreview", "--gain", "2.0", "-o", "-",
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

class CameraReader(threading.Thread):
    def __init__(self, proc: subprocess.Popen):
        super().__init__(daemon=True)
        self._proc = proc
        self._buffer = b""
        self._frame = None
        self._lock = threading.Lock()

    def run(self):
        while True:
            chunk = self._proc.stdout.read(4096)
            if not chunk: break
            self._buffer += chunk
            a, b = self._buffer.find(b'\xff\xd8'), self._buffer.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = self._buffer[a:b + 2]
                self._buffer = self._buffer[b + 2:]
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    # RPi Camera 3 Wide often mounted upside down on drones
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                    with self._lock: self._frame = frame

    def get_frame(self):
        with self._lock: return self._frame.copy() if self._frame is not None else None

class OpticalFlowNode(Node):
    def __init__(self):
        super().__init__('optical_flow_node')
        
        self.altitude = 0.2
        self._imu = None
        self.prev_gray = None
        self._last_stamp = None
        self._pos_x, self._pos_y = 0.0, 0.0

        self._proc = make_rpicam_proc(CAMERA_INDEX)
        self._cam = CameraReader(self._proc)
        self._cam.start()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(Range, '/uav/mavros/rangefinder_sub', self._alt_cb, 10)
        self.create_subscription(Imu, '/uav/mavros/imu/data', self._imu_cb, qos)
        
        self.pub_vel = self.create_publisher(TwistStamped, '/drone/optical_flow_vel', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/uav/mavros/vision_pose/pose', 10)
        self.create_service(Empty, '/drone/reset_pose', self._reset_pose_cb)
        self.timer = self.create_timer(1.0 / FPS, self._flow_callback)

    def _alt_cb(self, msg): 
        self.altitude = max(msg.range, 0.1)

    def _imu_cb(self, msg): 
        self._imu = msg

    def _flow_callback(self):
        frame = self._cam.get_frame()
        if frame is None or self._imu is None: return

        now = self.get_clock().now()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            self._last_stamp = now
            return

        dt = (now - self._last_stamp).nanoseconds * 1e-9
        if dt <= 0.001: return

        # Feature tracking
        raw_prev_pts = cv2.goodFeaturesToTrack(self.prev_gray, 100, 0.01, 10)
        if raw_prev_pts is None:
            self.prev_gray = gray
            return

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, raw_prev_pts, None)
        good_old = raw_prev_pts[status == 1]
        good_new = curr_pts[status == 1]

        if len(good_old) < 8:
            self.prev_gray = gray
            return

        # ── NORMALIZED UNDISTORTION ───────────────────────────────────────────
        # Omitting P=K converts pixel motion into "Normalized Image Coordinates"
        # This is where the 600m jump is fixed: flow is now in radians, not pixels.
        undist_old = cv2.undistortPoints(good_old.reshape(-1,1,2), K, D).reshape(-1,2)
        undist_new = cv2.undistortPoints(good_new.reshape(-1,1,2), K, D).reshape(-1,2)
        
        # Mean visual flow (radians-like displacement)
        flow_norm = np.mean(undist_new - undist_old, axis=0)

        # ── IMU Compensation ──────────────────────────────────────────────────
        q = self._imu.orientation
        roll = np.arctan2(2.0*(q.w*q.x + q.y*q.z), 1.0 - 2.0*(q.x*q.x + q.y*q.y))
        pitch = np.arcsin(np.clip(2.0*(q.w*q.y - q.z*q.x), -1.0, 1.0))
        true_alt = self.altitude * np.cos(roll) * np.cos(pitch)

        COMP_GAIN = 0.7
        # Subtract flow caused purely by rotation (omega * dt)
        w = self._imu.angular_velocity
        flow_norm[0] -= (w.y * dt) * COMP_GAIN
        flow_norm[1] -= (-w.x * dt) * COMP_GAIN

        # ── VELOCITY IN METERS ────────────────────────────────────────────────
        # VX = vertical image flow * alt
        # VY = horizontal image flow * alt
        vx = -(flow_norm[1] * true_alt) / dt
        vy = -(flow_norm[0] * true_alt) / dt

        # Integration
        self._pos_x += vx * dt
        self._pos_y += vy * dt

        # Publish
        stamp = now.to_msg()
        tw = TwistStamped()
        tw.header.stamp, tw.header.frame_id = stamp, 'drone_base_link'
        tw.twist.linear.x, tw.twist.linear.y = float(vx), float(vy)
        self.pub_vel.publish(tw)

        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = stamp, 'odom'
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = self._pos_x, self._pos_y, float(true_alt)
        ps.pose.orientation.w = 1.0
        self.pub_pose.publish(ps)

        self.prev_gray = gray
        self._last_stamp = now

    def _reset_pose_cb(self, _req, res):
        self._pos_x = self._pos_y = 0.0
        return res

    def destroy_node(self):
        self._proc.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()