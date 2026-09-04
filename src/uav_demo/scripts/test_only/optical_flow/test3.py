#!/usr/bin/env python3
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
NPZ_PATH = "/home/uav/UAV-GPS-Denied/src/uav_demo/scripts/test_only/calib_final_036/calib_intrinsics.npz"

if os.path.exists(NPZ_PATH):
    with np.load(NPZ_PATH) as data:
        K = data['K_l'].astype(np.float32)
        D = data['D_l'].astype(np.float32)
else:
    K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]], dtype=np.float32)
    D = np.zeros(5, dtype=np.float32)

# ── Camera settings ───────────────────────────────────────────────────────────
# USB webcams usually appear as /dev/video0, /dev/video1, etc.
CAMERA_INDEX = 0
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
FPS          = 30

class CameraReader(threading.Thread):
    def __init__(self, camera_index: int):
        super().__init__(daemon=True)
        self._cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f"Failed to open USB camera index {camera_index}")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self._cap.set(cv2.CAP_PROP_FPS, FPS)
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        self._frame = None
        self._lock = threading.Lock()

    def run(self):
        while True:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                break
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            with self._lock: self._frame = frame

    def get_frame(self):
        with self._lock: return self._frame.copy() if self._frame is not None else None

    def release(self):
        self._cap.release()

class OpticalFlowNode(Node):
    def __init__(self):
        super().__init__('optical_flow_node')
        
        self.altitude = 0.2
        self._imu: Imu = None
        self.prev_gray = None
        self._last_stamp = None
        
        # World-frame positions (North/East)
        self._pos_n = 0.0
        self._pos_e = 0.0

        self._cam = CameraReader(CAMERA_INDEX)
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

    def _publish_pose(self, stamp, altitude, orientation):
        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = stamp, 'odom'
        ps.pose.position.x = self._pos_n
        ps.pose.position.y = self._pos_e
        ps.pose.position.z = float(altitude)
        ps.pose.orientation = orientation
        self.pub_pose.publish(ps)

    def _flow_callback(self):
        now = self.get_clock().now()
        if self._imu is None: return

        q = self._imu.orientation

        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
        true_alt = self.altitude * np.cos(roll) * np.cos(pitch)

        frame = self._cam.get_frame()
        if frame is None:
            self._publish_pose(now.to_msg(), true_alt, q)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            self._last_stamp = now
            self._publish_pose(now.to_msg(), true_alt, q)
            return

        dt = (now - self._last_stamp).nanoseconds * 1e-9
        if dt <= 0.001:
            self._publish_pose(now.to_msg(), true_alt, q)
            return

        raw_prev_pts = cv2.goodFeaturesToTrack(self.prev_gray, 100, 0.01, 10)
        if raw_prev_pts is None:
            self.prev_gray = gray
            self._last_stamp = now
            self._publish_pose(now.to_msg(), true_alt, q)
            return

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, raw_prev_pts, None)
        good_old, good_new = raw_prev_pts[status == 1], curr_pts[status == 1]

        if len(good_old) < 8:
            self.prev_gray = gray
            self._last_stamp = now
            self._publish_pose(now.to_msg(), true_alt, q)
            return

        undist_old = cv2.undistortPoints(good_old.reshape(-1,1,2), K, D).reshape(-1,2)
        undist_new = cv2.undistortPoints(good_new.reshape(-1,1,2), K, D).reshape(-1,2)
        flow_norm = np.mean(undist_new - undist_old, axis=0)

        # Convert quaternion to yaw for world-frame integration.
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # ── Rotational Compensation ───────────────────────────────────────────
        COMP_GAIN = 0.5
        w = self._imu.angular_velocity
        # Compensation applied to normalized flow
        flow_norm[1] -= (w.y * dt) * COMP_GAIN
        flow_norm[0] += (w.x * dt) * COMP_GAIN * 0.7

        # ── Body-Frame Velocity (m/s) ─────────────────────────────────────────
        vx_body = -(flow_norm[1] * true_alt) / dt
        vy_body = -(flow_norm[0] * true_alt) / dt

        # ── World-Frame Velocity (Global Rotation) ────────────────────────────
        # Rotate body velocities by Yaw to get World N/E velocities
        v_north = vx_body * np.cos(yaw) - vy_body * np.sin(yaw)
        v_east  = vx_body * np.sin(yaw) + vy_body * np.cos(yaw)

        # Integration in World Frame
        self._pos_n += v_north * dt
        self._pos_e += v_east * dt

        # ── Publish ───────────────────────────────────────────────────────────
        stamp = now.to_msg()
        
        # Velocity usually published in body frame for controllers
        tw = TwistStamped()
        tw.header.stamp, tw.header.frame_id = stamp, 'drone_base_link'
        tw.twist.linear.x, tw.twist.linear.y = float(vx_body), float(vy_body)
        self.pub_vel.publish(tw)

        # Pose published in 'odom' (World-fixed North/East)
        self._publish_pose(stamp, true_alt, q)

        self.prev_gray, self._last_stamp = gray, now

    def _reset_pose_cb(self, _req, res):
        self._pos_n = self._pos_e = 0.0
        return res

    def destroy_node(self):
        self._cam.release()
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