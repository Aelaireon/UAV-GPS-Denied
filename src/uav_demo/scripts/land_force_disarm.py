#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandLong
from rclpy.node import Node
from sensor_msgs.msg import Range


class ConstantVelocityLanding(Node):
	def __init__(self):
		super().__init__('constant_velocity_landing')

		self.declare_parameter('descent_speed', 0.10)
		self.declare_parameter('ground_height', 0.15)
		self.declare_parameter('landing_timeout', 60.0)

		self.descent_speed = abs(self.get_parameter('descent_speed').value)
		self.ground_height = self.get_parameter('ground_height').value
		self.landing_timeout = self.get_parameter('landing_timeout').value
		self.altitude = None
		self.landing_started = None
		self.ground_detected_at = None
		self.disarm_requested = False
		self.landing_timer = None

		self.velocity_pub = self.create_publisher(
			TwistStamped,
			'/uav/mavros/setpoint_velocity/cmd_vel',
			10,
		)
		self.create_subscription(
			Range,
			'/uav/mavros/rangefinder_sub',
			self._range_callback,
			10,
		)
		self.disarm_client = self.create_client(
			CommandLong,
			'/uav/mavros/cmd/command',
		)
		self.landing_timer = self.create_timer(0.05, self._landing_callback, autostart=False)

	def start_landing(self):
		self.landing_started = self.get_clock().now()
		self.ground_detected_at = None
		self.disarm_requested = False
		self.get_logger().warn("UAV STARTING LANDING NOW")
		self.landing_timer.reset()

	def _range_callback(self, message):
		if message.range >= message.min_range and message.range <= message.max_range:
			self.altitude = message.range

	def _publish_descent(self):
		command = TwistStamped()
		command.header.stamp = self.get_clock().now().to_msg()
		command.header.frame_id = 'drone_base_link'
		command.twist.linear.z = -self.descent_speed
		self.velocity_pub.publish(command)

	def _publish_stop(self):
		command = TwistStamped()
		command.header.stamp = self.get_clock().now().to_msg()
		command.header.frame_id = 'drone_base_link'
		self.velocity_pub.publish(command)

	def _request_force_disarm(self):
		if self.disarm_requested:
			return
		self.disarm_requested = True
		self._publish_stop()
		if not self.disarm_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().error('Arming service is unavailable; vehicle was stopped but not disarmed')
			self.landing_timer.cancel()
			return

		request = CommandLong.Request()
		request.broadcast = False
		request.command = 400
		request.confirmation = 0
		request.param1 = 0.0
		request.param2 = 21196.0
		future = self.disarm_client.call_async(request)
		future.add_done_callback(self._disarm_response_callback)

	def _disarm_response_callback(self, future):
		try:
			response = future.result()
			if response.success:
				self.get_logger().info('Force disarm command accepted')
			else:
				self.get_logger().error(f'Force disarm command rejected: {response.result}')
		except Exception as error:
			self.get_logger().error(f'Force disarm service call failed: {error}')
		finally:
			self.landing_timer.cancel()

	def _landing_callback(self):
		if self.disarm_requested:
			self.get_logger().warning("Already disarming")
			return

		elapsed = (self.get_clock().now() - self.landing_started).nanoseconds * 1e-9
		if self.altitude is not None and self.altitude <= self.ground_height:
			now = self.get_clock().now()
			if self.ground_detected_at is None:
				self.ground_detected_at = now
				self.get_logger().info(
					f'Ground height reached ({self.altitude:.2f} m); '
					'waiting 1 second before force disarm'
				)
			ground_duration = (now - self.ground_detected_at).nanoseconds * 1e-9
			if ground_duration >= 1.0:
				self._request_force_disarm()
			else:
				self._publish_stop()
		elif elapsed >= self.landing_timeout:
			self.get_logger().error('Landing timeout reached; stopping and requesting force disarm')
			self._request_force_disarm()
		else:
			self.ground_detected_at = None
			self._publish_descent()


def main(args=None):
	rclpy.init(args=args)
	node = ConstantVelocityLanding()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node._publish_stop()
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
