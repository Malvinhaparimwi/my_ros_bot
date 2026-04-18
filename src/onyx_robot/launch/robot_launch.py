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
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {
                    'width': 640,
                    'height': 480,
                    'frame_rate': 30.0,
                    'format': 'BGR888',
                    'auto_exposure': 0,
                    'exposure_time': 15000,
                    'buffer_count': 4}])
        ])
