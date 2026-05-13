# Notes:
# create ros2 package: ros2 pkg create <pkg_name> --build-type ament_cmake --dependencies rclcpp rclpy 

all:
#	colcon build --symlink-install --packages-skip
	colcon build --symlink-install
open-serial:
	sudo chmod 777 /dev/ttyAMA0
	sudo chmod 777 /dev/ttyACM0
# 	sudo chmod 777 /dev/ttyFCU

tfmini:
	ros2 run uav_demo tfmini.py
gcs-deactivate-estop:
	ros2 topic pub --once /gcs_estop std_msgs/msg/Bool "{data: false}"
challenge-1:
	python3 src/challenge_demo/src/challenge_1.py
pos-lab:
	ros2 service call /uav/mavros/global_position/set_origin_global mavros_msgs/srv/SetOrigin "{latitude: 32.733417, longitude: -97.113556, altitude: 0.0}"
arm:
	ros2 service call /uav/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"	
disarm:
	ros2 service call /uav/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
guided-nogps:
	ros2 service call /uav/mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED_NOGPS'}"
guided:
	ros2 service call /uav/mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
alt-hold:
	ros2 service call /uav/mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'ALT_HOLD'}"
land:
	ros2 service call /uav/mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'LAND'}"
kill:
# 	This works 100% of the time, but it is long
# 	ros2 service call /uav/mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 400, param1: 0.0, param2: 21196.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
# 	This also seems to work 100% of the time, and is much shorter
	ros2 service call /uav/mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 400, param2: 21196.0}"
takeoff:
	make guided
	make arm
	ros2 service call /uav/mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 1.0}"
setpoint-2.0m:
	ros2 topic pub --rate 20 --times 100 /uav/mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}, orientation: {w: 1.0}}}"
setpoint-0.3m:
	ros2 topic pub --rate 20 --times 100 /uav/mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}"
test-mavros:
	echo -e "\033[1;36m############### TAKING OFF TO 1 METER ...###############\033[0m"
	make takeoff | grep -E "success=|mode_sent=|GUIDED|arming"
	sleep 5
	echo -e "\033[1;36m################# GOING TO 2 METERS ...##################\033[0m"
	make setpoint-2.0m | grep "beginning"
	echo -e "\033[1;36m################ GOING TO 0.3 METERS ...#################\033[0m"
	make setpoint-0.3m | grep "beginning"
	echo -e "\033[1;36m###################### LANDING ...#######################\033[0m"
	make land | grep "mode_sent="
stereo-test:
	ros2 run uav_demo stereo.test.py
stereo-dist:
	ros2 run uav_demo stereo.dist.py
stereo-calib:
	ros2 run uav_demo stereo.calib.py
stereo-calib-chk:
	ros2 run uav_demo stereo.calib.chk.py
stereo-vision-calib:
	ros2 run uav_demo stereo.vision.calib.py
stereo-vision-dist:
	ros2 run uav_demo stereo.vision.dist.py
mavros:
#	make open-serial
	ros2 launch uav_demo apm.launch
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
