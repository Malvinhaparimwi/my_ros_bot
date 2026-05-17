import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
            output='screen')])
