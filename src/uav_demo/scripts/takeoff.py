#!/usr/bin/env python3

import rclpy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node


class Takeoff(Node):
	def __init__(self):
		super().__init__('takeoff')

		self.mode_client = self.create_client(
			SetMode,
			'/uav/mavros/set_mode',
		)
		self.arm_client = self.create_client(
			CommandBool,
			'/uav/mavros/cmd/arming',
		)
		self.takeoff_client = self.create_client(
			CommandTOL,
			'/uav/mavros/cmd/takeoff',
		)

	def _call(self, client, request, service_name):
		if not client.wait_for_service(timeout_sec=5.0):
			self.get_logger().error(f'{service_name} service is unavailable')
			return None

		future = client.call_async(request)
		rclpy.spin_until_future_complete(self, future)
		if future.exception() is not None:
			self.get_logger().error(f'{service_name} call failed: {future.exception()}')
			return None
		return future.result()

	def run(self):
		mode_request = SetMode.Request()
		mode_request.base_mode = 0
		mode_request.custom_mode = 'GUIDED'
		mode_response = self._call(
			self.mode_client,
			mode_request,
			'/uav/mavros/set_mode',
		)
		if mode_response is None or not mode_response.mode_sent:
			self.get_logger().error('GUIDED mode was not accepted')
			return False

		arm_request = CommandBool.Request()
		arm_request.value = True
		arm_response = self._call(
			self.arm_client,
			arm_request,
			'/uav/mavros/cmd/arming',
		)
		if arm_response is None or not arm_response.success:
			self.get_logger().error('Arming was not accepted')
			return False

		takeoff_request = CommandTOL.Request()
		takeoff_request.altitude = 1.0
		takeoff_response = self._call(
			self.takeoff_client,
			takeoff_request,
			'/uav/mavros/cmd/takeoff',
		)
		if takeoff_response is None or not takeoff_response.success:
			self.get_logger().error('Takeoff was not accepted')
			return False

		self.get_logger().info('Takeoff command accepted at 1.0 m')
		return True


def main(args=None):
	rclpy.init(args=args)
	node = Takeoff()
	try:
		node.run()
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
