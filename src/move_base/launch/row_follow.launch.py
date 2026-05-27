from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("display_debug", default_value="true"),
        DeclareLaunchArgument("process_every_n_frames", default_value="1"),
        DeclareLaunchArgument("max_linear_speed", default_value="0.055"),
        DeclareLaunchArgument("max_angular_speed", default_value="0.06"),
        DeclareLaunchArgument("steering_gain", default_value="0.10"),
        DeclareLaunchArgument("steering_deadband", default_value="0.10"),
        DeclareLaunchArgument("steering_smoothing", default_value="0.06"),
        DeclareLaunchArgument("left_turn_scale", default_value="0.60"),
        DeclareLaunchArgument("forward_right_bias", default_value="0.017"),
        DeclareLaunchArgument("target_x_offset_ratio", default_value="-0.06"),
        DeclareLaunchArgument("imu_turn_delta_topic", default_value="/imu/angle_swept"),
        DeclareLaunchArgument("imu_reset_service", default_value="/reset_imu"),
        DeclareLaunchArgument("end_sequence_enabled", default_value="true"),
        DeclareLaunchArgument("end_confirm_sec", default_value="0.25"),
        DeclareLaunchArgument("end_advance_distance_m", default_value="0.75"),
        DeclareLaunchArgument("end_second_advance_distance_m", default_value="0.20"),
        DeclareLaunchArgument("end_sequence_forward_speed", default_value="0.10"),
        DeclareLaunchArgument("advance_wheel_diameter_m", default_value="0.11"),
        DeclareLaunchArgument("advance_seconds_per_wheel_rev", default_value="12.0"),
        DeclareLaunchArgument("advance_calibration_command_speed", default_value="0.10"),
        DeclareLaunchArgument("end_heading_reset_wait_sec", default_value="0.30"),
        DeclareLaunchArgument("end_turn_speed", default_value="0.07"),
        DeclareLaunchArgument("end_right_turn_yaw_delta_deg", default_value="-45.0"),
        DeclareLaunchArgument("end_turn_tolerance_deg", default_value="3.0"),
        DeclareLaunchArgument("end_turn_timeout_sec", default_value="8.0"),
        DeclareLaunchArgument(
            "detector_model_path",
            default_value=(
                "/home/eath/my_ros_bot/src/move_base/tools/yolov8n_best.pt"
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
                "process_every_n_frames": LaunchConfiguration("process_every_n_frames"),
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                "steering_gain": LaunchConfiguration("steering_gain"),
                "steering_deadband": LaunchConfiguration("steering_deadband"),
                "steering_smoothing": LaunchConfiguration("steering_smoothing"),
                "left_turn_scale": LaunchConfiguration("left_turn_scale"),
                "forward_right_bias": LaunchConfiguration("forward_right_bias"),
                "target_x_offset_ratio": LaunchConfiguration("target_x_offset_ratio"),
                "imu_turn_delta_topic": LaunchConfiguration("imu_turn_delta_topic"),
                "imu_reset_service": LaunchConfiguration("imu_reset_service"),
                "end_sequence_enabled": LaunchConfiguration("end_sequence_enabled"),
                "end_confirm_sec": LaunchConfiguration("end_confirm_sec"),
                "end_advance_distance_m": LaunchConfiguration("end_advance_distance_m"),
                "end_second_advance_distance_m": LaunchConfiguration(
                    "end_second_advance_distance_m"
                ),
                "end_sequence_forward_speed": LaunchConfiguration(
                    "end_sequence_forward_speed"
                ),
                "advance_wheel_diameter_m": LaunchConfiguration(
                    "advance_wheel_diameter_m"
                ),
                "advance_seconds_per_wheel_rev": LaunchConfiguration(
                    "advance_seconds_per_wheel_rev"
                ),
                "advance_calibration_command_speed": LaunchConfiguration(
                    "advance_calibration_command_speed"
                ),
                "end_heading_reset_wait_sec": LaunchConfiguration(
                    "end_heading_reset_wait_sec"
                ),
                "end_turn_speed": LaunchConfiguration("end_turn_speed"),
                "end_right_turn_yaw_delta_deg": LaunchConfiguration(
                    "end_right_turn_yaw_delta_deg"
                ),
                "end_turn_tolerance_deg": LaunchConfiguration("end_turn_tolerance_deg"),
                "end_turn_timeout_sec": LaunchConfiguration("end_turn_timeout_sec"),
                "detector_model_path": LaunchConfiguration(
                    "detector_model_path"
                ),
            }],
        ),
    ])
