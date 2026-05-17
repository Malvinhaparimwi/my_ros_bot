import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_camera',
            default_value='true',
            description='Start the Pi camera node with the rest of the robot.'
        ),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '23'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        Node(
            package='arduino_ros',
            executable='controller',
            name='motor_controller',
            output='screen'
            ),

        Node(
            package='pump_controller',
            executable='controller',
            name='pump_controller',
            output='screen'
            ),

        Node(
            package='ros_bot_imu',
            executable='imu_begin',
            name='imu_node',
            output='screen'
           ),

        Node(
            package='ros_bot_camera',
            executable='camera_begin',
            name='camera_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_camera')),
            respawn=True,
            respawn_delay=3.0
            )])
