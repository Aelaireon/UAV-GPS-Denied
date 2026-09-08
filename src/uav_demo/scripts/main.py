#!/usr/bin/env python3
"""
Purpose of this file is to control the drone by calling on various nodes and listening to feedback. 
This script should handle all flight operations such as takeoff, move, and land using related nodes.

"""

import math
import threading

import rclpy
from land_force_disarm import ConstantVelocityLanding
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node, QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.srv import CommandLong
from utils import PID, filterValue, INCHES_TO_METERS, METERS_TO_INCHES
import time
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from std_msgs.msg import Bool, Empty, String

class UAVSubsystem(Node):
    curr_speed_m_s = 0.0
    curr_x_vel_m_s = 0.0
    curr_y_vel_m_s = 0.0
    wheelbase = 0.34925 # meters, based measurements between center of front and rear wheels (~13.75 inches)
    track_width = 0.23495 # meters, based on measurements between left and right wheels (~10.75 inches)

    def __init__(self):
        """
        Initialize the UAV node, set up subscribers for uav_cmd_vel and velocity feedback, and initialize the motor and servo controllers with their respective PID controllers.
        """
        super().__init__('uav_node')
        
        self.init_pub_sub()
        
        self.land_disarm_command_node = ConstantVelocityLanding()
        
        self.goal_pose = None
        self.prev_goal_pose = None
        self.exit_flag = False
        self.estop_flag = True # Start in E-STOP state until we receive a valid goal pose or an explicit E-STOP release command, this is a safety measure to prevent the UAV from moving unexpectedly on startup before the system is fully initialized and ready to receive commands.
        
        self.lock = threading.Lock()
        
        self.heartbeat_threshold = 5
        self.gcs_heartbeat_count = 0
        self.encountered_estop = False
        self.encountered_heartbeat = False
        self.start_flag = False
        
        self._last_cmd_time = self.get_clock().now()
        self._cmd_min_interval = rclpy.duration.Duration(nanoseconds=50_000_000)  # 20 Hz max
        
        self.check_gcs_heartbeat_timer = self.create_timer(0.3, self.check_gcs_heartbeat) # Check heartbeat at 3 Hz, should be sufficient to detect loss within 0.6 second
        self.uav_hearbeat_timer = self.create_timer(0.1, self.uav_heartbeat_publisher)
        
    def init_pub_sub(self):
        self.velocity_body_sub = self.create_subscription(
            TwistStamped,
            # '/uav/mavros/local_position/velocity_body',
            "/uav/velocity/average",
            self.velocity_body_callback,
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=1
            )
        )
        
        # self.goal_pose_sub = self.create_subscription(
        #     PoseStamped,
        #     '/goal_pose',
        #     self.goal_pose_callback,
        #     10
        # )
        
        self.estop_sub = self.create_subscription(
            Bool,
            '/uav/estop',
            self.estop_callback,
            10
        )
        
        self.gcs_heartbeat_sub = self.create_subscription(
            Bool,
            '/gcs/uav/heartbeat',
            self.gcs_heartbeat_callback,
            10
        )
        
        self.uav_heartbeat_pub = self.create_publisher(
            Bool,
            '/uav/heartbeat',
            10
        )
        
        # self.uav_landed_boost_sub = self.create_subscription(
        #     String,
        #     '/uav_to_ugv',
        #     self.uav_msg_cb,
        #     10
        # )
    
    def uav_msg_pub(self, msg: String):
        # if msg.data == 'UAV_LANDED': # what ugv sees/requires/needs from uav
        #     # self.motor.offset_pwm = 0
        #     self.motor.offset_pwm = 50 # this is probably not the best way to do variable weight adaptation and dynamic motor performance
        pass
    
    def uav_heartbeat_publisher(self):
        self.uav_heartbeat_pub.publish(Bool(data=self.start_flag))
    
    # def goal_pose_callback(self, msg: PoseStamped):
    #     self.goal_pose = msg.pose
    #     if self.prev_goal_pose != self.goal_pose:
    #         self.estop_flag = False
    #         self.get_logger().warn(f"E-STOP state reverted, running UAV ...")
    #     self.prev_goal_pose = self.goal_pose

    def gcs_heartbeat_callback(self, msg: Bool):
        with self.lock:
            if self.gcs_heartbeat_count < self.heartbeat_threshold:
                self.gcs_heartbeat_count += 1
    
    def check_gcs_heartbeat(self):
        with self.lock:
            if self.gcs_heartbeat_count > 0:
                self.gcs_heartbeat_count -= 1
            if self.gcs_heartbeat_count == 0:
                if not self.encountered_heartbeat:
                    self.encountered_heartbeat = True
                    self.land_and_disarm()
                    self.get_logger().warn(f"GCS heartbeat lost, E-STOP on, stopping UAV ...")
                self.estop_flag = True
            else:
                if self.encountered_heartbeat:
                    self.encountered_heartbeat = False
                    self.get_logger().warn(f"GCS heartbeat received, total stored: {self.gcs_heartbeat_count} ...")
    
    def estop_callback(self, msg: Bool):
        with self.lock:
            self.estop_flag = msg.data
            if self.estop_flag:
                if not self.encountered_estop:
                    self.encountered_estop = True
                    self.land_and_disarm()
                    self.get_logger().warn(f"E-STOP detected: {self.estop_flag}, stopping UAV ...")
            else:
                if self.encountered_estop:
                    self.encountered_estop = False
                    self.get_logger().warn(f"E-STOP detected: {self.estop_flag}, running UAV ...")
    
    def land_and_disarm(self):
        # Calls node in land.force.disarm.py file to start the landing process, expect a code returned
        self.land_disarm_command_node.start_landing()
    
    # def cmd_vel_callback(self, msg: Twist):
    #     cmd_linear_x = msg.linear.x
    #     if not self.allow_reverse:
    #         cmd_linear_x = max(0, cmd_linear_x)
    #     self.target_vel_m_s = cmd_linear_x
    #     self.target_yaw_rad_s = -msg.angular.z if self.target_vel_m_s > 0 else msg.angular.z
    #     if not self.exit_flag: self.drive(self.target_yaw_rad_s, self.target_vel_m_s, self.wheelbase)
    #     else: self.land_and_disarm()

    def velocity_body_callback(self, msg: TwistStamped):
        # probably not accurate, but close enough
        self.curr_speed_m_s = math.sqrt(math.pow(msg.twist.linear.x, 2) + math.pow(msg.twist.linear.y, 2))
        self.curr_x_vel_m_s = msg.twist.linear.x
        self.curr_y_vel_m_s = msg.twist.linear.y

    def run(self):
        self.get_logger().info("UAV Subsystem Node is running. Waiting for uav_cmd_vel messages...")
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.add_node(self.land_disarm_command_node)
        executor.spin()
        self.get_logger().info("UAV Subsystem Node is shutting down.")

def main(args=None):
    rclpy.init(args=args)
    node = UAVSubsystem()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        # Wait for the stop commands to complete before shutting down the node
        node.exit_flag = True
        node.land_disarm_command_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
