import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def workspace_dir():
    override = os.environ.get('HCRL_LAB_CONTROLLER_DIR')
    if override:
        return Path(override).expanduser().resolve()

    share_dir = Path(get_package_share_directory('go1_ros_wrapper')).resolve()
    for parent in share_dir.parents:
        if (parent / 'go1_pact_mujoco').exists():
            return parent

    return share_dir.parents[3]


def package_config_path(filename):
    return Path(get_package_share_directory('go1_ros_wrapper')) / 'config' / filename


def read_yaml(path):
    with open(path, 'r', encoding='utf-8') as config_file:
        return yaml.safe_load(config_file) or {}


def yaml_value(config, section, key, default):
    value = config.get(section, {}).get(key, default)
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def env_or_yaml(config, section, key, environment_name, default):
    environment_value = os.environ.get(environment_name)
    if environment_value is not None and environment_value != '':
        return environment_value
    value = config.get(section, {}).get(key)
    if value is not None and value != '':
        return str(value)
    return str(default)


def bool_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def sim_backend_condition():
    return IfCondition(PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]))


def hardware_backend_condition():
    return IfCondition(PythonExpression(["'", LaunchConfiguration('backend'), "' == 'hardware'"]))


def physical_sensor_condition():
    return IfCondition(LaunchConfiguration('start_sensors'))


def gamepad_backend_condition(backend):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('start_gamepad'), "' == 'true' and '",
        LaunchConfiguration('gamepad_backend'), f"' == '{backend}'",
    ]))


def simulated_sensor_parameter(enable_argument):
    return ParameterValue(
        PythonExpression([
            "'", LaunchConfiguration(enable_argument), "' == 'true' and '",
            LaunchConfiguration('start_sensors'), "' != 'true'",
        ]),
        value_type=bool,
    )


def generate_launch_description():
    root = workspace_dir()
    mujoco_dir = root / 'go1_pact_mujoco'
    model_path = mujoco_dir / 'Resources' / 'scene.xml'
    pact_config_path = mujoco_dir / 'go1_pact_deploy' / 'params' / 'pact_config.yaml'
    sensors_config = read_yaml(package_config_path('sensors.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='sim', description='sim or hardware'),
        DeclareLaunchArgument('start_controller', default_value='true'),
        DeclareLaunchArgument('start_gamepad', default_value='true'),
        DeclareLaunchArgument('gamepad_backend', default_value='usb', description='usb, udp, or none'),
        DeclareLaunchArgument(
            'start_sensors',
            default_value='false',
            description='Start physical RealSense/Ellipse drivers with either robot backend',
        ),
        DeclareLaunchArgument('enable_sim_camera', default_value='true'),
        DeclareLaunchArgument('enable_sim_gps', default_value='true'),

        DeclareLaunchArgument('sim_config', default_value=str(package_config_path('mujoco_sim.yaml'))),
        DeclareLaunchArgument('hardware_config', default_value=str(package_config_path('hardware_bridge.yaml'))),
        DeclareLaunchArgument('controller_config', default_value=str(package_config_path('controller.yaml'))),
        DeclareLaunchArgument('gamepad_config', default_value=str(package_config_path('gamepad.yaml'))),
        DeclareLaunchArgument('sensors_config', default_value=str(package_config_path('sensors.yaml'))),
        DeclareLaunchArgument(
            'sensor_adapter_config',
            default_value=str(package_config_path('sensor_adapter.yaml')),
        ),

        DeclareLaunchArgument('model_path', default_value=str(model_path)),
        DeclareLaunchArgument('pact_config_path', default_value=str(pact_config_path)),
        DeclareLaunchArgument('mujoco_project_dir', default_value=str(mujoco_dir)),
        DeclareLaunchArgument('enable_viewer', default_value='true'),
        DeclareLaunchArgument('send_commands', default_value='false'),
        DeclareLaunchArgument('controller_initial_mode', default_value='damping'),
        DeclareLaunchArgument('debug_commands', default_value='false'),

        DeclareLaunchArgument('enable_realsense', default_value=yaml_value(sensors_config, 'sensors', 'enable_realsense', True)),
        DeclareLaunchArgument('enable_ellipse', default_value=yaml_value(sensors_config, 'sensors', 'enable_ellipse', True)),
        DeclareLaunchArgument('use_ntrip', default_value=yaml_value(sensors_config, 'sensors', 'use_ntrip', False)),
        DeclareLaunchArgument(
            'realsense_camera_namespace',
            default_value=yaml_value(sensors_config, 'realsense', 'camera_namespace', '/go1/sensors/realsense'),
        ),
        DeclareLaunchArgument(
            'realsense_camera_name',
            default_value=yaml_value(sensors_config, 'realsense', 'camera_name', 'd435i'),
        ),
        DeclareLaunchArgument(
            'realsense_serial_no',
            default_value=yaml_value(sensors_config, 'realsense', 'serial_no', "''"),
        ),
        DeclareLaunchArgument(
            'realsense_depth_profile',
            default_value=yaml_value(sensors_config, 'realsense', 'depth_profile', '640,480,15'),
        ),
        DeclareLaunchArgument(
            'realsense_color_profile',
            default_value=yaml_value(sensors_config, 'realsense', 'color_profile', '640,480,15'),
        ),
        DeclareLaunchArgument(
            'realsense_enable_color',
            default_value=yaml_value(sensors_config, 'realsense', 'enable_color', False),
        ),
        DeclareLaunchArgument(
            'realsense_enable_imu',
            default_value=yaml_value(sensors_config, 'realsense', 'enable_imu', False),
        ),
        DeclareLaunchArgument(
            'ellipse_namespace',
            default_value=yaml_value(sensors_config, 'ellipse', 'namespace', '/go1/sensors/ellipse'),
        ),
        DeclareLaunchArgument(
            'ellipse_port',
            default_value=yaml_value(sensors_config, 'ellipse', 'port', '/dev/ttyUSB0'),
        ),
        DeclareLaunchArgument(
            'ellipse_baud',
            default_value=yaml_value(sensors_config, 'ellipse', 'baud', 921600),
        ),
        DeclareLaunchArgument(
            'ellipse_frame_id',
            default_value=yaml_value(sensors_config, 'ellipse', 'frame_id', 'ellipse_link'),
        ),
        DeclareLaunchArgument(
            'ntrip_namespace',
            default_value=yaml_value(sensors_config, 'ntrip', 'namespace', '/go1/sensors/ellipse/ntrip_client'),
        ),
        DeclareLaunchArgument(
            'ntrip_host',
            default_value=env_or_yaml(
                sensors_config,
                'ntrip',
                'host',
                'MACORS_HOST',
                'macorsrtk.massdot.state.ma.us',
            ),
        ),
        DeclareLaunchArgument(
            'ntrip_port',
            default_value=env_or_yaml(sensors_config, 'ntrip', 'port', 'MACORS_PORT', 10000),
        ),
        DeclareLaunchArgument(
            'ntrip_mountpoint',
            default_value=env_or_yaml(
                sensors_config,
                'ntrip',
                'mountpoint',
                'MACORS_MOUNTPOINT',
                'RTCM3_MAX',
            ),
        ),
        DeclareLaunchArgument(
            'ntrip_username',
            default_value=env_or_yaml(sensors_config, 'ntrip', 'username', 'MACORS_USERNAME', ''),
        ),
        DeclareLaunchArgument(
            'ntrip_password',
            default_value=env_or_yaml(sensors_config, 'ntrip', 'password', 'MACORS_PASSWORD', ''),
        ),
        DeclareLaunchArgument(
            'ntrip_reconnect_attempt_max',
            default_value=yaml_value(sensors_config, 'ntrip', 'reconnect_attempt_max', 120),
        ),
        DeclareLaunchArgument(
            'ntrip_reconnect_attempt_wait_seconds',
            default_value=yaml_value(
                sensors_config,
                'ntrip',
                'reconnect_attempt_wait_seconds',
                5,
            ),
        ),
        DeclareLaunchArgument(
            'ntrip_rtcm_timeout_seconds',
            default_value=yaml_value(sensors_config, 'ntrip', 'rtcm_timeout_seconds', 10),
        ),

        Node(
            package='go1_ros_wrapper',
            executable='go1_mujoco_sim_node',
            name='go1_mujoco_sim',
            output='screen',
            condition=sim_backend_condition(),
            parameters=[
                LaunchConfiguration('sim_config'),
                {
                    'model_path': LaunchConfiguration('model_path'),
                    'enable_viewer': bool_parameter('enable_viewer'),
                    'enable_sim_camera': simulated_sensor_parameter('enable_sim_camera'),
                    'enable_sim_gps': simulated_sensor_parameter('enable_sim_gps'),
                },
            ],
            remappings=[
                ('joint_states', '/go1/joint_states'),
                ('imu', '/go1/imu'),
                ('camera/color/image_raw', '/go1/sensors/camera/color/image_raw'),
                ('camera/color/camera_info', '/go1/sensors/camera/color/camera_info'),
                ('gps/fix', '/go1/sensors/gps/fix'),
                ('ground_truth/odom', '/go1/ground_truth/odom'),
                ('low_cmd', '/go1/low_cmd'),
                ('low_cmd_echo', '/go1/low_cmd_echo'),
            ],
        ),
        Node(
            package='go1_ros_wrapper',
            executable='go1_unitree_bridge_node',
            name='go1_unitree_bridge',
            output='screen',
            condition=hardware_backend_condition(),
            parameters=[
                LaunchConfiguration('hardware_config'),
                {'send_commands': bool_parameter('send_commands')},
            ],
            remappings=[
                ('joint_states', '/go1/joint_states'),
                ('imu', '/go1/imu'),
                ('low_cmd', '/go1/low_cmd'),
                ('low_cmd_echo', '/go1/low_cmd_echo'),
            ],
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=gamepad_backend_condition('usb'),
            parameters=[LaunchConfiguration('gamepad_config')],
            remappings=[('joy', '/joy')],
        ),
        Node(
            package='go1_ros_wrapper',
            executable='go1_joy_gamepad_node',
            name='go1_joy_gamepad',
            output='screen',
            condition=gamepad_backend_condition('usb'),
            parameters=[LaunchConfiguration('gamepad_config')],
            remappings=[
                ('joy', '/joy'),
                ('gamepad', '/go1/gamepad'),
            ],
        ),
        Node(
            package='go1_ros_wrapper',
            executable='go1_udp_gamepad_node',
            name='go1_udp_gamepad',
            output='screen',
            condition=gamepad_backend_condition('udp'),
            parameters=[LaunchConfiguration('gamepad_config')],
            remappings=[('gamepad', '/go1/gamepad')],
        ),
        Node(
            package='go1_ros_wrapper',
            executable='go1_pact_controller_node',
            name='go1_pact_controller',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_controller')),
            parameters=[
                LaunchConfiguration('controller_config'),
                {
                    'config_path': LaunchConfiguration('pact_config_path'),
                    'mujoco_project_dir': LaunchConfiguration('mujoco_project_dir'),
                    'initial_mode': LaunchConfiguration('controller_initial_mode'),
                    'debug_commands': bool_parameter('debug_commands'),
                },
            ],
            remappings=[
                ('joint_states', '/go1/joint_states'),
                ('imu', '/go1/imu'),
                ('gamepad', '/go1/gamepad'),
                ('controller_mode', '/go1/controller_mode'),
                ('velocity_command', '/go1/velocity_command'),
                ('low_cmd', '/go1/low_cmd'),
            ],
        ),
        Node(
            package='go1_ros_wrapper',
            executable='go1_sensor_adapter_node',
            name='go1_sensor_adapter',
            output='screen',
            condition=physical_sensor_condition(),
            parameters=[
                LaunchConfiguration('sensor_adapter_config'),
                {
                    'enable_camera': bool_parameter('enable_realsense'),
                    'enable_gps': bool_parameter('enable_ellipse'),
                },
            ],
            remappings=[
                ('camera/color/image_raw', '/go1/sensors/camera/color/image_raw'),
                ('camera/color/camera_info', '/go1/sensors/camera/color/camera_info'),
                ('camera/depth/image_raw', '/go1/sensors/camera/depth/image_raw'),
                ('camera/depth/camera_info', '/go1/sensors/camera/depth/camera_info'),
                ('gps/fix', '/go1/sensors/gps/fix'),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('go1_ros_wrapper'),
                    'launch',
                    'go1_sensors.launch.py',
                ])
            ),
            condition=physical_sensor_condition(),
            launch_arguments={
                'enable_realsense': LaunchConfiguration('enable_realsense'),
                'enable_ellipse': LaunchConfiguration('enable_ellipse'),
                'use_ntrip': LaunchConfiguration('use_ntrip'),
                'realsense_camera_namespace': LaunchConfiguration('realsense_camera_namespace'),
                'realsense_camera_name': LaunchConfiguration('realsense_camera_name'),
                'realsense_serial_no': LaunchConfiguration('realsense_serial_no'),
                'realsense_depth_profile': LaunchConfiguration('realsense_depth_profile'),
                'realsense_color_profile': LaunchConfiguration('realsense_color_profile'),
                'realsense_enable_color': LaunchConfiguration('realsense_enable_color'),
                'realsense_enable_imu': LaunchConfiguration('realsense_enable_imu'),
                'ellipse_namespace': LaunchConfiguration('ellipse_namespace'),
                'ellipse_port': LaunchConfiguration('ellipse_port'),
                'ellipse_baud': LaunchConfiguration('ellipse_baud'),
                'ellipse_frame_id': LaunchConfiguration('ellipse_frame_id'),
                'ntrip_namespace': LaunchConfiguration('ntrip_namespace'),
                'ntrip_host': LaunchConfiguration('ntrip_host'),
                'ntrip_port': LaunchConfiguration('ntrip_port'),
                'ntrip_mountpoint': LaunchConfiguration('ntrip_mountpoint'),
                'ntrip_username': LaunchConfiguration('ntrip_username'),
                'ntrip_password': LaunchConfiguration('ntrip_password'),
                'ntrip_reconnect_attempt_max': LaunchConfiguration('ntrip_reconnect_attempt_max'),
                'ntrip_reconnect_attempt_wait_seconds': LaunchConfiguration(
                    'ntrip_reconnect_attempt_wait_seconds'
                ),
                'ntrip_rtcm_timeout_seconds': LaunchConfiguration('ntrip_rtcm_timeout_seconds'),
            }.items(),
        ),
    ])
