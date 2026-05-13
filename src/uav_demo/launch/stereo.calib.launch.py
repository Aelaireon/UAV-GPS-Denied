from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Left Camera Node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='left_cam',
            namespace='stereo/left',
            parameters=[{
                'video_device': '/dev/video0', # Check which is which
                'camera_frame_id': 'left_camera_optical_frame',
            }]
        ),
        # Right Camera Node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='right_cam',
            namespace='stereo/right',
            parameters=[{
                'video_device': '/dev/video2', # Usually video0 and video2 on Pi 5
                'camera_frame_id': 'right_camera_optical_frame',
            }]
        ),
    ])