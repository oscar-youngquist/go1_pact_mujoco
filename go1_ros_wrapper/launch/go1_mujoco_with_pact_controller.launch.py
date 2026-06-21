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
    model_path = mujoco_dir / 'Resources' / 'scene.xml'
    config_path = mujoco_dir / 'go1_pact_deploy' / 'params' / 'pact_config.yaml'

    return LaunchDescription([
        DeclareLaunchArgument('enable_viewer', default_value='true'),
        DeclareLaunchArgument('controller_initial_mode', default_value='damping'),
        DeclareLaunchArgument('gamepad_udp_port', default_value='26760'),
        DeclareLaunchArgument('debug_commands', default_value='false'),
        DeclareLaunchArgument('model_path', default_value=str(model_path)),
        DeclareLaunchArgument('config_path', default_value=str(config_path)),
        DeclareLaunchArgument('mujoco_project_dir', default_value=str(mujoco_dir)),
        Node(
            package='go1_ros_wrapper',
            executable='go1_mujoco_sim_node',
            name='go1_mujoco_sim',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'sim_rate_hz': 500.0,
                'publish_rate_hz': 100.0,
                'command_timeout_s': 0.2,
                'publish_when_paused': True,
                'enable_viewer': LaunchConfiguration('enable_viewer'),
                'render_rate_hz': 60.0,
                'viewer_width': 1200,
                'viewer_height': 900,
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
                ('low_cmd', '/go1/low_cmd'),
            ],
        ),
    ])