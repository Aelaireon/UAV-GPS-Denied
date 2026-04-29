# Notes:
# create ros2 package: ros2 pkg create <pkg_name> --build-type ament_cmake --dependencies rclcpp rclpy 

all:
#	colcon build --symlink-install --packages-skip
	colcon build --symlink-install
open-serial:
	sudo chmod 777 /dev/ttyAMA0
	sudo chmod 777 /dev/ttyACM0
gcs-deactivate-estop:
	ros2 topic pub --once /gcs_estop std_msgs/msg/Bool "{data: false}"
challenge-1:
	python3 src/challenge_demo/src/challenge_1.py
pos-lab:
	ros2 service call /uav/mavros/global_position/set_origin_global mavros_msgs/srv/SetOrigin "{latitude: 32.733417, longitude: -97.113556, altitude: 0.0}"
mavros-arm:
	ros2 service call /uav/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"	
mavros-disarm:
	ros2 service call /uav/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
mavros-launch:
	ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 config_yaml:=~/UAV/src/uav_demo/config/apm_config.yaml
viewframes:
	ros2 run tf2_tools view_frames
gcs:
# 	ros2 launch gcs_demo bringup.launch.py # Not color decorated for some reason, but it works
	python3 ./src/gcs_demo/src/gcs.py # This is naturally color decorated to make reading warnings and errors easier
estop:
	ros2 topic pub --once /uav_estop std_msgs/msg/Bool "{data: true}" &
	ros2 topic pub --once /uav_estop std_msgs/msg/Bool "{data: true}" &
	ros2 topic pub --once /uav_estop std_msgs/msg/Bool "{data: true}"
estop-revert-state:
	ros2 topic pub --once /uav_estop std_msgs/msg/Bool "{data: false}"
keyboard-control:
	make estop-revert-state
	ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/uav_cmd_vel
	ros2 topic pub --once /uav_estop std_msgs/msg/Bool "{data: true}" &
	ros2 topic pub --once /uav_estop std_msgs/msg/Bool "{data: true}" &
	ros2 topic pub --once /uav_estop std_msgs/msg/Bool "{data: true}"
clean:
	rm -rf build/ install/ log/
