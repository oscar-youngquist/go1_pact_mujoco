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
    model_path = root / 'go1_pact_mujoco' / 'Resources' / 'scene.xml'

    return LaunchDescription([
        DeclareLaunchArgument('enable_viewer', default_value='true'),
        DeclareLaunchArgument('model_path', default_value=str(model_path)),
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
            executable='go1_pd_controller_node',
            name='go1_pd_controller',
            output='screen',
            parameters=[{
                'control_rate_hz': 100.0,
                'kp': 20.0,
                'kd': 0.8,
            }],
            remappings=[
                ('joint_states', '/go1/joint_states'),
                ('low_cmd', '/go1/low_cmd'),
            ],
        ),
    ])