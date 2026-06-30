import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def env_or_empty(name):
    return os.environ.get(name, '')


def bool_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def int_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_realsense', default_value='true'),
        DeclareLaunchArgument('enable_ellipse', default_value='true'),
        DeclareLaunchArgument('use_ntrip', default_value='false'),

        DeclareLaunchArgument('realsense_camera_namespace', default_value='/go1/sensors/realsense'),
        DeclareLaunchArgument('realsense_camera_name', default_value='d435i'),
        DeclareLaunchArgument('realsense_serial_no', default_value="''"),
        DeclareLaunchArgument('realsense_depth_profile', default_value='640,480,15'),
        DeclareLaunchArgument('realsense_color_profile', default_value='640,480,15'),
        DeclareLaunchArgument('realsense_enable_color', default_value='false'),
        DeclareLaunchArgument('realsense_enable_imu', default_value='false'),

        DeclareLaunchArgument('ellipse_namespace', default_value='/go1/sensors/ellipse'),
        DeclareLaunchArgument('ellipse_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('ellipse_baud', default_value='921600'),
        DeclareLaunchArgument('ellipse_frame_id', default_value='ellipse_link'),

        DeclareLaunchArgument('ntrip_namespace', default_value='/go1/sensors/ellipse/ntrip_client'),
        DeclareLaunchArgument('ntrip_host', default_value=env_or_empty('MACORS_HOST')),
        DeclareLaunchArgument('ntrip_port', default_value=os.environ.get('MACORS_PORT', '2101')),
        DeclareLaunchArgument('ntrip_mountpoint', default_value=env_or_empty('MACORS_MOUNTPOINT')),
        DeclareLaunchArgument('ntrip_username', default_value=env_or_empty('MACORS_USERNAME')),
        DeclareLaunchArgument('ntrip_password', default_value=env_or_empty('MACORS_PASSWORD')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py',
                ])
            ),
            condition=IfCondition(LaunchConfiguration('enable_realsense')),
            launch_arguments={
                'camera_namespace': LaunchConfiguration('realsense_camera_namespace'),
                'camera_name': LaunchConfiguration('realsense_camera_name'),
                'serial_no': LaunchConfiguration('realsense_serial_no'),
                'device_type': 'd435i',
                'enable_depth': 'true',
                'enable_color': LaunchConfiguration('realsense_enable_color'),
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_gyro': LaunchConfiguration('realsense_enable_imu'),
                'enable_accel': LaunchConfiguration('realsense_enable_imu'),
                'unite_imu_method': '2',
                'enable_sync': 'false',
                'depth_module.depth_profile': LaunchConfiguration('realsense_depth_profile'),
                'rgb_camera.color_profile': LaunchConfiguration('realsense_color_profile'),
                'align_depth.enable': 'false',
                'pointcloud.enable': 'false',
                'spatial_filter.enable': 'false',
                'temporal_filter.enable': 'false',
                'hole_filling_filter.enable': 'false',
            }.items(),
        ),

        Node(
            package='sbg_driver',
            executable='sbg_device',
            namespace=LaunchConfiguration('ellipse_namespace'),
            name='sbg_device',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_ellipse')),
            parameters=[{
                'driver': {'frequency': 400},
                'odometry': {
                    'enable': False,
                    'publishTf': False,
                    'odomFrameId': 'odom',
                    'baseFrameId': 'base_link',
                    'initFrameId': 'map',
                },
                'confWithRos': False,
                'uartConf': {
                    'portName': LaunchConfiguration('ellipse_port'),
                    'baudRate': int_parameter('ellipse_baud'),
                    'portID': 0,
                },
                'output': {
                    'time_reference': 'ros',
                    'ros_standard': True,
                    'use_enu': True,
                    'frame_id': LaunchConfiguration('ellipse_frame_id'),
                    'log_status': 8,
                    'log_imu_data': 1,
                    'log_ekf_quat': 1,
                    'log_ekf_nav': 1,
                    'log_utc_time': 8,
                    'log_mag': 8,
                    'log_gps1_vel': 10001,
                    'log_gps1_pos': 10001,
                    'log_gps1_hdt': 10001,
                    'log_gps1_raw': 10001,
                },
                'rtcm': {
                    'subscribe': bool_parameter('use_ntrip'),
                    'namespace': 'ntrip_client',
                    'topic_name': 'rtcm',
                },
                'nmea': {
                    'publish': bool_parameter('use_ntrip'),
                    'namespace': 'ntrip_client',
                    'topic_name': 'nmea',
                },
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ntrip_client'),
                    'launch',
                    'ntrip_client_launch.py',
                ])
            ),
            condition=IfCondition(LaunchConfiguration('use_ntrip')),
            launch_arguments={
                'namespace': LaunchConfiguration('ntrip_namespace'),
                'node_name': 'ntrip_client',
                'host': LaunchConfiguration('ntrip_host'),
                'port': LaunchConfiguration('ntrip_port'),
                'mountpoint': LaunchConfiguration('ntrip_mountpoint'),
                'authenticate': 'true',
                'username': LaunchConfiguration('ntrip_username'),
                'password': LaunchConfiguration('ntrip_password'),
                'rtcm_message_package': 'rtcm_msgs',
            }.items(),
        ),
    ])
