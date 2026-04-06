from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='system_monitor',
            executable='monitoring_dashboard',
            name='monitoring_dashboard',
            output='screen',
            emulate_tty=True,   # needed for ANSI colors in terminal
        )
    ])
