import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def workspace_dir():
    override = os.environ.get('HCRL_LAB_CONTROLLER_DIR')
    if override:
        return Path(override).expanduser().resolve()

    share_dir = Path(get_package_share_directory('go1_ros_wrapper')).resolve()
    for parent in share_dir.parents:
        if (parent / 'go1_pact_mujoco').exists():
            return parent

    return share_dir.parents[3]


def generate_launch_description():
    root = workspace_dir()
    mujoco_dir = root / 'go1_pact_mujoco'
    config_path = mujoco_dir / 'go1_pact_deploy' / 'params' / 'pact_config.yaml'

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.123.10'),
        DeclareLaunchArgument('local_port', default_value='8090'),
        DeclareLaunchArgument('robot_port', default_value='8007'),
        DeclareLaunchArgument('send_commands', default_value='false'),
        DeclareLaunchArgument('controller_initial_mode', default_value='damping'),
        DeclareLaunchArgument('gamepad_udp_port', default_value='26760'),
        DeclareLaunchArgument('debug_commands', default_value='false'),
        DeclareLaunchArgument('config_path', default_value=str(config_path)),
        DeclareLaunchArgument('mujoco_project_dir', default_value=str(mujoco_dir)),
        Node(
            package='go1_ros_wrapper',
            executable='go1_unitree_bridge_node',
            name='go1_unitree_bridge',
            output='screen',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'local_port': LaunchConfiguration('local_port'),
                'robot_port': LaunchConfiguration('robot_port'),
                'udp_rate_hz': 500.0,
                'publish_rate_hz': 100.0,
                'command_timeout_s': 0.2,
                'power_protect_level': 9,
                'send_damping_on_timeout': True,
                'timeout_damping_kd': 3.0,
                'send_commands': LaunchConfiguration('send_commands'),
            }],
            remappings=[
                ('joint_states', '/go1/joint_states'),
                ('imu', '/go1/imu'),
                ('low_cmd', '/go1/low_cmd'),
                ('low_cmd_echo', '/go1/low_cmd_echo'),
            ],
        ),
        Node(
            package='go1_ros_wrapper',
            executable='go1_udp_gamepad_node',
            name='go1_udp_gamepad',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('gamepad_udp_port'),
                'poll_rate_hz': 200.0,
                'publish_rate_hz': 100.0,
            }],
            remappings=[
                ('gamepad', '/go1/gamepad'),
            ],
        ),
        Node(
            package='go1_ros_wrapper',
            executable='go1_pact_controller_node',
            name='go1_pact_controller',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'mujoco_project_dir': LaunchConfiguration('mujoco_project_dir'),
                'control_rate_hz': 100.0,
                'initial_mode': LaunchConfiguration('controller_initial_mode'),
                'damping_kd': 3.0,
                'debug_commands': LaunchConfiguration('debug_commands'),
            }],
            remappings=[
                ('joint_states', '/go1/joint_states'),
                ('imu', '/go1/imu'),
                ('gamepad', '/go1/gamepad'),
                ('controller_mode', '/go1/controller_mode'),
                ('velocity_command', '/go1/velocity_command'),
                ('low_cmd', '/go1/low_cmd'),
            ],
        ),
    ])
