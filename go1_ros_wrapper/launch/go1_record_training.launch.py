from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('go1_ros_wrapper'),
                'launch',
                'go1_sensors.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('include_sensors')),
        launch_arguments={
            'enable_realsense': LaunchConfiguration('enable_realsense'),
            'enable_ellipse': LaunchConfiguration('enable_ellipse'),
            'use_ntrip': LaunchConfiguration('use_ntrip'),
            'ellipse_port': LaunchConfiguration('ellipse_port'),
            'ellipse_baud': LaunchConfiguration('ellipse_baud'),
            'realsense_enable_color': LaunchConfiguration('realsense_enable_color'),
            'realsense_enable_imu': LaunchConfiguration('realsense_enable_imu'),
        }.items(),
    )

    recorder = Node(
        package='go1_ros_wrapper',
        executable='go1_training_recorder.py',
        name='go1_training_recorder',
        output='screen',
        parameters=[{
            'run_type': LaunchConfiguration('run_type'),
            'output_root': LaunchConfiguration('output_root'),
            'record_mode': LaunchConfiguration('record_mode'),
            'gps_topic': LaunchConfiguration('gps_topic'),
            'orientation_topic': LaunchConfiguration('orientation_topic'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'rgb_info_topic': LaunchConfiguration('rgb_info_topic'),
            'depth_info_topic': LaunchConfiguration('depth_info_topic'),
            'record_camera_info': ParameterValue(LaunchConfiguration('record_camera_info'), value_type=bool),
            'record_camera_info_once': ParameterValue(LaunchConfiguration('record_camera_info_once'), value_type=bool),
            'hash_image_payloads': ParameterValue(LaunchConfiguration('hash_image_payloads'), value_type=bool),
            'include_topic_prefixes': ['/go1'],
            'include_tf': ParameterValue(LaunchConfiguration('include_tf'), value_type=bool),
            'discovery_period_s': 1.0,
            'max_inline_sequence': 128,
            'flush_every_n_rows': 100,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_type',
            description='Training label for this run. Must be safe or fast.',
        ),
        DeclareLaunchArgument('output_root', default_value='~/go1_training_runs'),
        DeclareLaunchArgument('record_mode', default_value='selected'),
        DeclareLaunchArgument('gps_topic', default_value='/go1/sensors/gps/fix'),
        DeclareLaunchArgument('orientation_topic', default_value='/go1/sensors/ellipse/imu/data'),
        DeclareLaunchArgument('rgb_topic', default_value='/go1/sensors/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/go1/sensors/camera/depth/image_raw'),
        DeclareLaunchArgument('rgb_info_topic', default_value='/go1/sensors/camera/color/camera_info'),
        DeclareLaunchArgument('depth_info_topic', default_value='/go1/sensors/camera/depth/camera_info'),
        DeclareLaunchArgument('record_camera_info', default_value='true'),
        DeclareLaunchArgument('record_camera_info_once', default_value='true'),
        DeclareLaunchArgument('hash_image_payloads', default_value='false'),
        DeclareLaunchArgument('include_sensors', default_value='false'),
        DeclareLaunchArgument('include_tf', default_value='false'),
        DeclareLaunchArgument('enable_realsense', default_value='true'),
        DeclareLaunchArgument('enable_ellipse', default_value='true'),
        DeclareLaunchArgument('use_ntrip', default_value='false'),
        DeclareLaunchArgument('ellipse_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('ellipse_baud', default_value='921600'),
        DeclareLaunchArgument('realsense_enable_color', default_value='false'),
        DeclareLaunchArgument('realsense_enable_imu', default_value='false'),
        sensor_launch,
        recorder,
    ])
