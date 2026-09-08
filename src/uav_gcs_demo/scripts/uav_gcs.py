#!/usr/bin/env python3

import math
from queue import Empty
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool, Float32, String
from nav_msgs.msg import Path
from datetime import datetime
from pynput import keyboard
from pynput.keyboard import Key, KeyCode

class KEY:
    def __init__(self, k, flag=False):
        self.k = k
        self.flag = flag
        self.prev = self.flag

    def once(self, key):
        if self.k != key:
            return False
        self.flag = True
        if self.flag != self.prev:
            self.prev = True
            return True
        return False
    
    def toggle(self, state, key):
        if self.k != key:
            return False
        self.flag = state
        self.prev = state
        return True

    def release(self, key):
        if self.k != key:
            return False
        self.flag = False
        self.prev = False
        return True

class UAVGCSNode(Node):
    forward_m_s: float = 0.0
    left_rad_s: float = 0.0
    forward_in_s: float = 0.0
    left_deg_s: float = 0.0
    start_time_sys = None
    start_time_ros = None
    reached_destination: bool = False
    destination_receipt = None
    generated_path = None
    estop_status: bool = True
    CANCEL_E_STOP_1_KEY = KEY(Key.esc)
    CANCEL_E_STOP_2_KEY = KEY(Key.ctrl_l)
    CANCEL_E_STOP_3_KEY = KEY(Key.ctrl_r)
    E_STOP_KEY = KEY(Key.space)
    E_STOP_ALT_KEY = KEY(Key.enter)

    def __init__(self) -> None:
        super().__init__("uav_gcs_node")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # or RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # uses mavros odom vel
        self.uav_vel_sub = self.create_subscription(
            TwistStamped,
            # "/uav/mavros/local_position/velocity_body",
            "/uav/velocity/average",
            self.uav_vel_callback,
            qos_profile
        )
        
        # self.uav_destination_sub = self.create_subscription(
        #     PoseStamped,
        #     "/goal_pose",
        #     self.uav_destination_callback,
        #     qos_profile
        # )
        
        # self.path_sub = self.create_subscription(
        #     Path,
        #     "/plan",
        #     self.path_callback,
        #     qos_profile
        # )
        
        self.uav_heartbeat_sub = self.create_subscription(
            Bool,
            '/uav/heartbeat',
            self.uav_heartbeat_callback,
            10
        )
        
        self.gcs_estop = self.create_subscription(
            Bool,
            '/gcs/estop',
            self.gcs_estop_callback,
            10
        )
        
        self.estop_pub = self.create_publisher(
            Bool,
            "/uav/estop",
            10
        )
        
        self.force_disarm_pub = self.create_publisher(
            Bool,
            "/uav/force_disarm",
            10
        )
        
        # self.estop_sub = self.create_subscription(
        #     Bool,
        #     "/uav/estop",
        #     self.estop_callback,
        #     10
        # )
        
        self.gcs_heartbeat_pub = self.create_publisher(
            Bool,
            "/gcs/heartbeat",
            10
        )
        
        self.ugv_msg_sub = self.create_subscription(
            String,
            '/ugv_to_uav',
            self.ugv_msg_callback,
            qos_profile
        )

        self.start_time_sys = datetime.now()
        
        self.lock = threading.Lock()
        
        self.heartbeat_threshold = 10
        self.uav_heartbeat_count = 0
        self.estop_wait_duration = 0.5
        self.uav_start_time = False
        self.time_since_start = 0
        self.ugv_landed_flag = False
        self.start_challenge = False
        
        # run publish at 10 Hz
        self.print_out_timer = self.create_timer(0.3, self.print_out)
        self.heartbeat_timer = self.create_timer(0.1, self.gcs_heartbeat_publisher)
        # self.estop_timer = self.create_timer(0.1, self.estop_publisher)
        self.check_uav_heartbeat_timer = self.create_timer(0.3, self.check_uav_heartbeat) # Check heartbeat at 3 Hz, should be sufficient to detect loss within 0.6 second
        self.release_estop_timer = self.create_timer(self.estop_wait_duration, self.release_estop_uav, autostart=False) # Arm UAV after 2 seconds, count starts when both cancel estop keys are pressed and held
        self.release_estop_timer_started = False

        # Collect events until released
        self.my_keyboard = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.my_keyboard.start()
    
    def ugv_msg_callback(self, msg: String):
        if msg.data == 'START_MOVING':
            self.get_logger().warn('Received START_MOVING message.')
            self.ugv_landed_flag = False
            self.start_challenge = True
            self.estop_status = False
            self.estop_publisher()
        # elif msg.data == 'ugv_LANDED':
        #     self.get_logger().warn('Received ugv_LANDED message.')
        #     self.ugv_landed_flag = True
        else:
            self.get_logger().warn(f'Received unexpected message on /ugv_to_uav: {msg.data}. Ignoring.')
            self.estop_status = False
            self.estop_publisher()
    
    def gcs_estop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warn(f"GCS E-STOP received, activating E-STOP ...")
            self.estop_status = True
            self.gcs_estop = True
            self.estop_publisher()
        else:
            self.get_logger().warn(f"GCS E-STOP cleared, deactivating E-STOP ...")
            self.estop_status = False
            self.gcs_estop = False
            self.estop_publisher()
    
    def release_estop_uav(self):
        if self.CANCEL_E_STOP_1_KEY.flag and (self.CANCEL_E_STOP_2_KEY.flag or self.CANCEL_E_STOP_3_KEY.flag):
            self.estop_status = False
            self.get_logger().warn(f"E-STOP Deactivated!")
            self.estop_publisher()
            self.estop_publisher()
            self.estop_publisher()
            self.estop_publisher()
            self.estop_publisher()
        self.release_estop_timer_started = False
        self.release_estop_timer.cancel()
        
    def on_press(self, key):
        if self.E_STOP_KEY.once(key):
            self.estop_status = True
            self.get_logger().warn(f"E-STOP Space pressed")
            self.estop_publisher()
        self.CANCEL_E_STOP_1_KEY.once(key)
        self.CANCEL_E_STOP_2_KEY.once(key)
        self.CANCEL_E_STOP_3_KEY.once(key)
        if self.CANCEL_E_STOP_1_KEY.flag and (self.CANCEL_E_STOP_2_KEY.flag or self.CANCEL_E_STOP_3_KEY.flag) and not self.release_estop_timer_started:
            self.get_logger().warn(f"Both cancel E-STOP keys pressed, starting timer to release E-STOP ...")
            self.release_estop_timer.reset()
            self.release_estop_timer_started = True
    
    def on_release(self, key):
        self.E_STOP_KEY.release(key)
        self.E_STOP_ALT_KEY.release(key)
        if self.CANCEL_E_STOP_1_KEY.release(key) or self.CANCEL_E_STOP_2_KEY.release(key) or self.CANCEL_E_STOP_3_KEY.release(key):
            self.get_logger().warn(f"One of the cancel E-STOP keys released, stopping timer to release E-STOP ...")
            self.release_estop_timer_started = False
            self.release_estop_timer.cancel()

    def uav_heartbeat_callback(self, msg: Bool):
        self.uav_start_time = msg.data
        with self.lock:
            if self.uav_heartbeat_count < self.heartbeat_threshold:
                self.uav_heartbeat_count += 1
        
    # def estop_callback(self, msg: Bool) -> None:
    #     self.estop_status = msg.data
    #     if msg.data == False:
    #         self.get_logger().warn("ESTOP ACTIVATED!")
    #     else:
    #         self.get_logger().warn("ESTOP DEACTIVATED.")
        
    # def path_callback(self, msg: Path) -> None:
    #     self.generated_path = msg.poses
    #     self.get_logger().info(f"Received path with {len(msg.poses)} poses")
    
    # def uav_destination_callback(self, msg: PoseStamped) -> None:
    #     self.destination_receipt = True
    #     self.goal_header = msg.header
    #     self.goal_pose = msg.pose
    #     self.get_logger().info(f"Received UAV destination: {self.goal_pose.position.x}, {self.goal_pose.position.y}")
        
    def uav_vel_callback(self, msg: TwistStamped) -> None:
        forward_m_s = msg.twist.linear.x
        left_rad_s = msg.twist.linear.y

        self.forward_in_s = round(forward_m_s * 39.3701, 2)
        self.left_deg_s = round(math.degrees(left_rad_s), 2)

        self.forward_m_s = round(forward_m_s, 3)
        self.left_rad_s = round(left_rad_s, 3)
    
    def check_uav_heartbeat(self):
        with self.lock:
            if self.uav_heartbeat_count > 0:
                self.uav_heartbeat_count -= 1
            if self.uav_heartbeat_count == 0:
                self.get_logger().warn(f"UAV heartbeat lost, activating E-STOP ...")
                self.estop_status = True
                self.estop_publisher()
            else:
                self.get_logger().warn(f"UAV heartbeat received, total stored: {self.uav_heartbeat_count} ...")

    def estop_publisher(self) -> None:
        # print("Publishing E-STOP status: ", self.estop_status)
        self.estop_pub.publish(Bool(data=self.estop_status))
    
    def gcs_heartbeat_publisher(self):
        self.gcs_heartbeat_pub.publish(Bool(data=True))

    def print_out(self):
        forward_in_s = ("F " + str(self.forward_in_s) if self.forward_in_s > 0 else "B " + str(-self.forward_in_s) if self.forward_in_s < 0 else "_") + " in/s"
        left_deg_s = ("L " + str(self.left_deg_s) if self.left_deg_s > 0 else "R " + str(-self.left_deg_s) if self.left_deg_s < 0 else "_") + " deg/s"
        
        # time_since_start = f"{(self.get_clock().now() - self.start_time_ros).nanoseconds / 1e9:.2f}"
        # time_since_start = time_since_start if not self.uav_start_time else 0
        if self.uav_start_time:
            if self.time_since_start == 0:
                self.start_time_ros = self.get_clock().now()
            self.time_since_start = f"{(self.get_clock().now() - self.start_time_ros).nanoseconds / 1e9:.2f}"

        self.get_logger().info("---- UAV GCS Telemetry ----")
        self.get_logger().info(f"Start time: {self.start_time_sys}")
        self.get_logger().info(f"Time since start: {self.time_since_start} seconds")
        self.get_logger().info(f"Destination reached: {'Yes' if self.reached_destination else 'No'}")
        self.get_logger().info(f"Destination receipt: {'Received' if self.destination_receipt else 'Not received'}")
        self.get_logger().info(f"Generated path: {'Yes' if self.generated_path is not None else 'No'}")
        
        # self.get_logger().info(f"Received UAV velocity:\n\t{forward_m_s}\t{forward_in_s}\n\t{left_rad_s}\t{left_deg_s}")
        # self.get_logger().info(f"Received UAV velocity:\n\t{forward_m_s}\n\t{left_rad_s}")
        self.get_logger().info(f"UAV E-STOP Status: {self.estop_status}")
        self.get_logger().info(f"Received UAV velocity: {forward_in_s} {left_deg_s}")

def main(args=None):
    rclpy.init(args=args)
    node = UAVGCSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()

if __name__ == "__main__":
    main()