from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Standard Environment Configuration
        SetEnvironmentVariable('ROS_DOMAIN_ID', '23'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        # Customizable Launch Arguments
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/right/compressed',
            description='Topic to subscribe to side camera compressed images'
        ),
        DeclareLaunchArgument(
            'pump_topic',
            default_value='/pump/controller',
            description='Topic to publish pump control commands (on/off)'
        ),
        DeclareLaunchArgument(
            'model_threshold',
            default_value='0.50',
            description='YOLOv8 confidence score threshold for plant detection'
        ),
        DeclareLaunchArgument(
            'trigger_mode',
            default_value='immediate',
            description='Spraying trigger mode: immediate or cross_line'
        ),
        DeclareLaunchArgument(
            'spray_duration',
            default_value='4.0',
            description='Spraying duration in seconds per detected plant'
        ),
        DeclareLaunchArgument(
            'publish_debug',
            default_value='true',
            description='Whether to publish real-time compressed visual debug overlay feed'
        ),
        DeclareLaunchArgument(
            'display_debug',
            default_value='false',
            description='Whether to show a local OpenCV debug window'
        ),
        DeclareLaunchArgument(
            'process_every_n_frames',
            default_value='4',
            description='Process every N-th camera frame to save CPU'
        ),
        DeclareLaunchArgument(
            'model_input_size',
            default_value='320',
            description='Resolution grid size to run YOLOv8 inference (320 or 640)'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cpu',
            description='PyTorch execution device (cpu, cuda, etc.)'
        ),
        DeclareLaunchArgument(
            'trigger_delay',
            default_value='0.0',
            description='Calibration delay in seconds before triggering the pump'
        ),
        DeclareLaunchArgument(
            'trigger_x_ratio',
            default_value='0.50',
            description='Vertical trigger line ratio in image frame (0.0 to 1.0)'
        ),
        DeclareLaunchArgument(
            'direction_of_motion',
            default_value='either',
            description='Direction of motion of plants in the camera frame'
        ),

        # Start Pump Logic Node
        Node(
            package='pump_logic',
            executable='detector',
            name='pump_logic_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'pump_topic': LaunchConfiguration('pump_topic'),
                'model_threshold': LaunchConfiguration('model_threshold'),
                'trigger_mode': LaunchConfiguration('trigger_mode'),
                'spray_duration': LaunchConfiguration('spray_duration'),
                'publish_debug': LaunchConfiguration('publish_debug'),
                'display_debug': LaunchConfiguration('display_debug'),
                'process_every_n_frames': LaunchConfiguration('process_every_n_frames'),
                'model_input_size': LaunchConfiguration('model_input_size'),
                'device': LaunchConfiguration('device'),
                'trigger_delay': LaunchConfiguration('trigger_delay'),
                'trigger_x_ratio': LaunchConfiguration('trigger_x_ratio'),
                'direction_of_motion': LaunchConfiguration('direction_of_motion'),
            }]
        )
    ])
