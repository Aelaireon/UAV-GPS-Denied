#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import time
import serial

from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped


class TFMiniNode(Node):
    def __init__(self):
        super().__init__('tfmini_node')

        # Serial Setup
        self.sensor = None
        self.publish_fake_vision = False # use until optical flow is working
        self.ground_offset = 0.13
        self.current_range_m = 0.0

        try:
            self.sensor = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)
            if not self.sensor.is_open:
                self.sensor.open()
            self.get_logger().info("TFMini Plus initialized on /dev/ttyAMA0")
        except Exception as e:
            self.get_logger().error(f"TFMini serial init failed: {e}")

        # Publishers
        self.range_pub = self.create_publisher(Range, '/uav/mavros/rangefinder_sub', 10)
        if self.publish_fake_vision:
            self.vision_pub = self.create_publisher(PoseStamped, '/uav/mavros/vision_pose/pose', 10)

        # Background reader thread
        self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.reader_thread.start()

        # Publish at 20Hz
        self.timer = self.create_timer(0.1, self.publish_data)

    def read_loop(self):
        """Continuously parse TFMini frames in a background thread."""
        while True:
            try:
                if self.sensor is None:
                    time.sleep(0.1)
                    continue

                if self.sensor.read(1) != b'\x59':
                    continue
                if self.sensor.read(1) != b'\x59':
                    continue

                data = self.sensor.read(7)
                if len(data) < 7:
                    continue

                dist_l, dist_h, str_l, str_h, temp_l, temp_h, checksum = data

                calculated = (0x59 + 0x59 + dist_l + dist_h + str_l + str_h + temp_l + temp_h) & 0xFF
                if calculated != checksum:
                    continue

                distance_m = (dist_l + (dist_h << 8)) / 100.0

                if self.ground_offset is None:
                    self.ground_offset = distance_m
                    self.get_logger().info(f"Ground offset captured: {self.ground_offset:.3f}m")

                self.current_range_m = distance_m - self.ground_offset
                self.get_logger().info(f"Height: {self.current_range_m}")
            except Exception as e:
                self.get_logger().warn(f"TFMini read error: {e}")
                time.sleep(0.01)

    def publish_data(self):
        if self.sensor is None:
            return

        stamp = self.get_clock().now().to_msg()
        current_range = self.current_range_m

        # Rangefinder message
        rng = Range()
        rng.header.stamp = stamp
        rng.header.frame_id = "lidar"
        rng.radiation_type = 1
        rng.field_of_view = 0.0628
        rng.min_range = 0.1
        # rng.max_range = 12.0 # indoors/high reflectivity
        rng.max_range = 7.0 # outdoors/low reflectivity/football field grass
        rng.range = float(current_range)
        self.range_pub.publish(rng)

        if self.publish_fake_vision:
            # Vision pose (Z = ground-corrected altitude)
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "map"
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = float(current_range)
            pose.pose.orientation.w = 1.0
            self.vision_pub.publish(pose)


def main():
    rclpy.init()
    node = TFMiniNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
