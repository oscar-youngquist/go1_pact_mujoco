from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go1_ros_wrapper',
            executable='go1_unitree_bridge_node',
            name='go1_unitree_bridge',
            output='screen',
            parameters=[{
                'robot_ip': '192.168.123.10',
                'local_port': 8090,
                'robot_port': 8007,
                'udp_rate_hz': 500.0,
                'publish_rate_hz': 100.0,
                'command_timeout_s': 0.2,
                'send_damping_on_timeout': True,
                'timeout_damping_kd': 3.0,
                'send_commands': False,
            }],
            remappings=[
                ('joint_states', '/go1/joint_states'),
                ('imu', '/go1/imu'),
                ('low_cmd', '/go1/low_cmd'),
                ('low_cmd_echo', '/go1/low_cmd_echo'),
            ],
        ),
    ])
