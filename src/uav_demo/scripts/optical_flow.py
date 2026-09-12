#!/usr/bin/env python3
import threading
import numpy as np
import cv2
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range, Imu
from geometry_msgs.msg import Quaternion, TwistStamped, PoseStamped
from std_srvs.srv import Empty

# ── Load Camera Configuration ────────────────────────────────────────────────
# NPZ_PATH = "/home/uav/UAV-GPS-Denied/src/uav_demo/scripts/test_only/calib_final_036/calib_intrinsics.npz"
NPZ_PATH = ""

if os.path.exists(NPZ_PATH):
    with np.load(NPZ_PATH) as data:
        K = data['K_l'].astype(np.float32)
        D = data['D_l'].astype(np.float32)
else:
    K = np.array([[343.0, 0, 320], [0, 343.0, 320], [0, 0, 1]], dtype=np.float32)
    D = np.zeros(5, dtype=np.float32)

# ── Camera settings ───────────────────────────────────────────────────────────
# Arducam UC-A43REV.A exposes the Pi camera through UVC.
CAMERA_INDEX = 0
FRAME_WIDTH  = 1280
FRAME_HEIGHT = 720
FPS          = 30
PROCESS_SIZE = 640
FOCUS_ABSOLUTE = 205  # Approx. 0.5 m on the 1..1023 UVC focus scale.

class CameraReader(threading.Thread):
    def __init__(self, camera_index: int):
        super().__init__(daemon=True)
        self._cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f"Failed to open USB camera index {camera_index}")

        self._cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self._cap.set(cv2.CAP_PROP_FOCUS, FOCUS_ABSOLUTE)

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
            height, width = frame.shape[:2]
            crop_size = min(height, width)
            left = (width - crop_size) // 2
            top = (height - crop_size) // 2
            frame = frame[top:top + crop_size, left:left + crop_size]
            frame = cv2.resize(
                frame,
                (PROCESS_SIZE, PROCESS_SIZE),
                interpolation=cv2.INTER_AREA,
            )
            with self._lock: self._frame = frame

    def get_frame(self):
        with self._lock: return self._frame.copy() if self._frame is not None else None

    def release(self):
        self._cap.release()

class OpticalFlowNode(Node):
    def __init__(self):
        super().__init__('optical_flow_node')
        
        self.altitude = 0.2
        self.altitude_speed = 0.0
        self._last_altitude = None
        self._last_altitude_time = None
        self._altitude_motion_until = None
        self._imu: Imu = None
        self.prev_gray = None
        self._last_stamp = None
        self._imu_yaw_offset = None
        self._filtered_vx = 0.0
        self._filtered_vy = 0.0
        self._velocity_filter_alpha = 0.2
        self._velocity_deadband = 0.03
        self._rotation_compensation_gain = 1.0
        
        # World-frame positions (North/West)
        self._pos_n = 0.0
        self._pos_w = 0.0

        self._cam = CameraReader(CAMERA_INDEX)
        self._cam.start()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(Range, '/uav/mavros/rangefinder_sub', self._alt_cb, 10)
        self.create_subscription(Imu, '/uav/mavros/imu/data', self._imu_cb, qos)
        
        self.pub_vel = self.create_publisher(
            TwistStamped,
            '/uav/mavros/vision_speed/speed_twist',
            10,
        )
        self.pub_pose = self.create_publisher(PoseStamped, '/uav/mavros/vision_pose/pose', 10)
        self.create_service(Empty, '/drone/reset_pose', self._reset_pose_cb)
        self.timer = self.create_timer(1.0 / FPS, self._flow_callback)

    def _alt_cb(self, msg):
        altitude = max(msg.range, 0.1)
        now = self.get_clock().now()
        if self._last_altitude is not None and self._last_altitude_time is not None:
            dt = (now - self._last_altitude_time).nanoseconds * 1e-9
            if dt > 0.001:
                self.altitude_speed = (altitude - self._last_altitude) / dt
                if abs(self.altitude_speed) > 0.15:
                    self._altitude_motion_until = now.nanoseconds + 300_000_000
        self.altitude = altitude
        self._last_altitude = altitude
        self._last_altitude_time = now

    def _imu_cb(self, msg): 
        self._imu = msg

    def _publish_pose(self, stamp, altitude, orientation):
        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = stamp, 'odom'
        ps.pose.position.x = self._pos_n
        ps.pose.position.y = self._pos_w
        ps.pose.position.z = float(altitude)
        ps.pose.orientation = orientation
        self.pub_pose.publish(ps)

    def _orientation_from_rpy(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        orientation = Quaternion()
        orientation.w = cr * cp * cy + sr * sp * sy
        orientation.x = sr * cp * cy - cr * sp * sy
        orientation.y = cr * sp * cy + sr * cp * sy
        orientation.z = cr * cp * sy - sr * sp * cy
        return orientation

    def _publish_velocity(self, stamp, vx, vy):
        tw = TwistStamped()
        tw.header.stamp, tw.header.frame_id = stamp, 'drone_base_link'
        tw.twist.linear.x = float(vx)
        tw.twist.linear.y = float(vy)
        tw.twist.linear.z = float(self.altitude_speed)
        self.pub_vel.publish(tw)

    def _flow_callback(self):
        now = self.get_clock().now()
        if self._imu is None: return

        q = self._imu.orientation

        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        imu_yaw = np.arctan2(siny_cosp, cosy_cosp)
        if self._imu_yaw_offset is None:
            self._imu_yaw_offset = imu_yaw
        yaw = np.arctan2(
            np.sin(imu_yaw - self._imu_yaw_offset),
            np.cos(imu_yaw - self._imu_yaw_offset),
        )
        true_alt = self.altitude * np.cos(roll) * np.cos(pitch)
        visual_orientation = self._orientation_from_rpy(
            roll,
            pitch,
            yaw,
        )

        frame = self._cam.get_frame()
        if frame is None:
            self._publish_velocity(now.to_msg(), 0.0, 0.0)
            self._publish_pose(now.to_msg(), true_alt, visual_orientation)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            self._last_stamp = now
            self._publish_velocity(now.to_msg(), 0.0, 0.0)
            self._publish_pose(now.to_msg(), true_alt, visual_orientation)
            return

        dt = (now - self._last_stamp).nanoseconds * 1e-9
        if dt <= 0.001:
            self._publish_velocity(now.to_msg(), 0.0, 0.0)
            self._publish_pose(now.to_msg(), true_alt, visual_orientation)
            return

        if self._altitude_motion_until is not None and now.nanoseconds < self._altitude_motion_until:
            self.prev_gray = gray
            self._last_stamp = now
            self._filtered_vx = 0.0
            self._filtered_vy = 0.0
            self._publish_velocity(now.to_msg(), 0.0, 0.0)
            self._publish_pose(now.to_msg(), true_alt, visual_orientation)
            return

        raw_prev_pts = cv2.goodFeaturesToTrack(self.prev_gray, 100, 0.01, 10)
        if raw_prev_pts is None:
            self.prev_gray = gray
            self._last_stamp = now
            self._publish_velocity(now.to_msg(), 0.0, 0.0)
            self._publish_pose(now.to_msg(), true_alt, visual_orientation)
            return

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, raw_prev_pts, None)
        good_old, good_new = raw_prev_pts[status == 1], curr_pts[status == 1]

        if len(good_old) < 8:
            self.prev_gray = gray
            self._last_stamp = now
            self._publish_velocity(now.to_msg(), 0.0, 0.0)
            self._publish_pose(now.to_msg(), true_alt, visual_orientation)
            return

        undist_old = cv2.undistortPoints(good_old.reshape(-1,1,2), K, D).reshape(-1,2)
        undist_new = cv2.undistortPoints(good_new.reshape(-1,1,2), K, D).reshape(-1,2)

        _, inlier_mask = cv2.estimateAffinePartial2D(
            undist_old,
            undist_new,
            method=cv2.RANSAC,
            ransacReprojThreshold=0.01,
        )

        if inlier_mask is not None and np.count_nonzero(inlier_mask) >= 8:
            inliers = inlier_mask.ravel().astype(bool)
            flow_norm = np.median(
                undist_new[inliers] - undist_old[inliers],
                axis=0,
            )
        else:
            flow_norm = np.median(undist_new - undist_old, axis=0)

        # ── Rotational Compensation ───────────────────────────────────────────
        w = self._imu.angular_velocity
        # Compensation applied to normalized flow
        flow_norm[1] -= (w.y * dt) * self._rotation_compensation_gain
        flow_norm[0] += (w.x * dt) * self._rotation_compensation_gain

        # ── Body-Frame Velocity (m/s) ─────────────────────────────────────────
        raw_vx_body = -(flow_norm[1] * true_alt) / dt
        raw_vy_body = -(flow_norm[0] * true_alt) / dt
        alpha = self._velocity_filter_alpha
        self._filtered_vx = alpha * raw_vx_body + (1.0 - alpha) * self._filtered_vx
        self._filtered_vy = alpha * raw_vy_body + (1.0 - alpha) * self._filtered_vy
        vx_body = (
            0.0
            if abs(self._filtered_vx) < self._velocity_deadband
            else self._filtered_vx
        )
        vy_body = (
            0.0
            if abs(self._filtered_vy) < self._velocity_deadband
            else self._filtered_vy
        )

        # ── World-Frame Velocity (Global Rotation) ────────────────────────────
        # Rotate body velocities by Yaw to get World N/E velocities
        v_north = vx_body * np.cos(yaw) - vy_body * np.sin(yaw)
        v_east  = vx_body * np.sin(yaw) + vy_body * np.cos(yaw)

        # Integration in World Frame
        self._pos_n += v_north * dt
        self._pos_w += v_east * dt

        # ── Publish ───────────────────────────────────────────────────────────
        stamp = now.to_msg()
        visual_orientation = self._orientation_from_rpy(roll, pitch, yaw)
        
        # Velocity usually published in body frame for controllers
        self._publish_velocity(stamp, vx_body, vy_body)

        # Pose published in 'odom' (World-fixed North/East)
        self._publish_pose(stamp, true_alt, visual_orientation)
        
        # print(
        #     f"[Optical Flow] dt={dt:6.3f}s, \n"
        #     f"vx_body={vx_body:+8.2f} m/s, vy_body={vy_body:+8.2f} m/s, "
        #     f"vz={self.altitude_speed:+8.2f} m/s, \n"
        #     f"pos_n={self._pos_n:+8.2f} m, pos_w={self._pos_w:+8.2f} m, "
        #     f"pos_z={true_alt:+8.2f} m, \norientation_euler_deg="
        #     f"(roll={np.degrees(roll):+8.2f}, pitch={np.degrees(pitch):+8.2f}, "
        #     f"yaw={np.degrees(yaw):+8.2f})\n"
        # )

        self.prev_gray, self._last_stamp = gray, now

    def _reset_pose_cb(self, _req, res):
        self._pos_n = self._pos_w = 0.0
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