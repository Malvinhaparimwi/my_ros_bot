from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dry_run",         default_value="false"),
        DeclareLaunchArgument("display_debug",   default_value="true"),
        DeclareLaunchArgument(
            "detector_model_path",
            default_value=(
                "/home/eath/my_ros_bot/src/move_base/tools/yolov8n_best.pt"
            ),
        ),

        # ── vision / steering ────────────────────────────────────────────────
        DeclareLaunchArgument("move_base_process_every_n_frames",  default_value="1"),
        DeclareLaunchArgument("move_base_max_linear_speed",        default_value="0.055"),
        DeclareLaunchArgument("move_base_max_angular_speed",       default_value="0.06"),
        DeclareLaunchArgument("move_base_steering_gain",           default_value="0.10"),
        DeclareLaunchArgument("move_base_steering_deadband",       default_value="0.06"),
        DeclareLaunchArgument("move_base_steering_smoothing",      default_value="0.06"),
        DeclareLaunchArgument("move_base_left_turn_scale",         default_value="0.60"),
        DeclareLaunchArgument("move_base_forward_right_bias",      default_value="0.010"),
        DeclareLaunchArgument("move_base_target_x_offset_ratio",   default_value="-0.01"),

        # ── IMU ──────────────────────────────────────────────────────────────
        DeclareLaunchArgument("imu_turn_delta_topic", default_value="/imu/angle_swept"),
        DeclareLaunchArgument("imu_reset_service",    default_value="/reset_imu"),

        # ── row-end trigger ──────────────────────────────────────────────────
        DeclareLaunchArgument("end_confirm_sec",      default_value="0.25"),

        # ── advance distances ────────────────────────────────────────────────
        # 75 cm advance at the end of every row (all three rows)
        DeclareLaunchArgument("end_advance_distance_m",               default_value="0.75"),
        # 20 cm between the two RIGHT turns (row 0 → row 1)
        DeclareLaunchArgument("end_between_turns_distance_m",         default_value="0.17"),
        # 40 cm between the two LEFT  turns (row 1 → row 2)
        DeclareLaunchArgument("end_between_turns_distance_left_m",    default_value="0.50"),

        # ── forward speed used during all advance phases ─────────────────────
        DeclareLaunchArgument("end_sequence_forward_speed",           default_value="0.10"),

        # ── wheel / advance calibration ──────────────────────────────────────
        DeclareLaunchArgument("advance_wheel_diameter_m",             default_value="0.11"),
        DeclareLaunchArgument("advance_seconds_per_wheel_rev",        default_value="12.0"),
        DeclareLaunchArgument("advance_calibration_command_speed",    default_value="0.10"),

        # ── IMU heading reset before each turn ───────────────────────────────
        DeclareLaunchArgument("end_heading_reset_wait_sec",           default_value="3.0"),

        # ── turn parameters ──────────────────────────────────────────────────
        DeclareLaunchArgument("end_turn_speed",                       default_value="0.07"),
        # Right turn target (row 0→1): negative = clockwise
        DeclareLaunchArgument("end_right_turn_yaw_delta_deg",         default_value="-80.0"),
        # Left  turn target (row 1→2): positive = counter-clockwise
        DeclareLaunchArgument("end_left_turn_yaw_delta_deg",          default_value="85.0"),
        DeclareLaunchArgument("end_turn_tolerance_deg",               default_value="2.0"),
        DeclareLaunchArgument("end_turn_timeout_sec",                 default_value="8.0"),
        # Degrees before target where speed drops to end_turn_slow_speed (overshoot fix)
        DeclareLaunchArgument("end_turn_slow_zone_deg",               default_value="8.0"),
        # Reduced speed used inside the slow zone
        DeclareLaunchArgument("end_turn_slow_speed",                  default_value="0.035"),
        # Freshness window for /imu/angle_swept during turns (longer than imu_timeout_sec)
        DeclareLaunchArgument("turn_imu_timeout_sec",                 default_value="1.0"),

        SetEnvironmentVariable("ROS_DOMAIN_ID",      "23"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),

        Node(
            package="move_base",
            executable="drive_robot",
            name="row_follower",
            output="screen",
            parameters=[{
                # vision / steering
                "dry_run":                  LaunchConfiguration("dry_run"),
                "display_debug":            LaunchConfiguration("display_debug"),
                "detector_model_path":      LaunchConfiguration("detector_model_path"),
                "process_every_n_frames":   LaunchConfiguration("move_base_process_every_n_frames"),
                "max_linear_speed":         LaunchConfiguration("move_base_max_linear_speed"),
                "max_angular_speed":        LaunchConfiguration("move_base_max_angular_speed"),
                "steering_gain":            LaunchConfiguration("move_base_steering_gain"),
                "steering_deadband":        LaunchConfiguration("move_base_steering_deadband"),
                "steering_smoothing":       LaunchConfiguration("move_base_steering_smoothing"),
                "left_turn_scale":          LaunchConfiguration("move_base_left_turn_scale"),
                "forward_right_bias":       LaunchConfiguration("move_base_forward_right_bias"),
                "target_x_offset_ratio":    LaunchConfiguration("move_base_target_x_offset_ratio"),

                # IMU
                "imu_turn_delta_topic":     LaunchConfiguration("imu_turn_delta_topic"),
                "imu_reset_service":        LaunchConfiguration("imu_reset_service"),

                # end sequence — always on
                "end_sequence_enabled":     True,
                "end_confirm_sec":          LaunchConfiguration("end_confirm_sec"),

                # advances
                "end_advance_distance_m":               LaunchConfiguration("end_advance_distance_m"),
                "end_between_turns_distance_m":         LaunchConfiguration("end_between_turns_distance_m"),
                "end_between_turns_distance_left_m":    LaunchConfiguration("end_between_turns_distance_left_m"),
                "end_sequence_forward_speed":           LaunchConfiguration("end_sequence_forward_speed"),

                # wheel calibration
                "advance_wheel_diameter_m":             LaunchConfiguration("advance_wheel_diameter_m"),
                "advance_seconds_per_wheel_rev":        LaunchConfiguration("advance_seconds_per_wheel_rev"),
                "advance_calibration_command_speed":    LaunchConfiguration("advance_calibration_command_speed"),

                # IMU heading reset
                "end_heading_reset_wait_sec":           LaunchConfiguration("end_heading_reset_wait_sec"),

                # turns
                "end_turn_speed":                       LaunchConfiguration("end_turn_speed"),
                "end_right_turn_yaw_delta_deg":         LaunchConfiguration("end_right_turn_yaw_delta_deg"),
                "end_left_turn_yaw_delta_deg":          LaunchConfiguration("end_left_turn_yaw_delta_deg"),
                "end_turn_tolerance_deg":               LaunchConfiguration("end_turn_tolerance_deg"),
                "end_turn_timeout_sec":                 LaunchConfiguration("end_turn_timeout_sec"),
                "end_turn_slow_zone_deg":               LaunchConfiguration("end_turn_slow_zone_deg"),
                "end_turn_slow_speed":                  LaunchConfiguration("end_turn_slow_speed"),
                "turn_imu_timeout_sec":                 LaunchConfiguration("turn_imu_timeout_sec"),
            }],
        ),
    ])