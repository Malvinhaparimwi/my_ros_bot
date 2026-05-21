from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("display_debug", default_value="true"),
        DeclareLaunchArgument("max_linear_speed", default_value="0.10"),
        DeclareLaunchArgument("steering_gain", default_value="0.85"),
        DeclareLaunchArgument(
            "segmentation_model_path",
            default_value=(
                "/home/eath/my_ros_bot/src/move_base/tools/models/tiny_row_det.onnx"
            ),
        ),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "23"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        Node(
            package="move_base",
            executable="drive_robot",
            name="row_follower",
            output="screen",
            parameters=[{
                "dry_run": LaunchConfiguration("dry_run"),
                "display_debug": LaunchConfiguration("display_debug"),
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "steering_gain": LaunchConfiguration("steering_gain"),
                "segmentation_model_path": LaunchConfiguration(
                    "segmentation_model_path"
                ),
            }],
        ),
    ])
