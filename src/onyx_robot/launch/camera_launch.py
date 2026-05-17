from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '23'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('LIBCAMERA_LOG_LEVELS', '*:3'),

        Node(
            package='ros_bot_camera',
            executable='camera_begin',
            name='camera_node',
            output='screen',
            respawn=True,
            respawn_delay=3.0
        )
    ])
