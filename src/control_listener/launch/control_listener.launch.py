from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    control_listener_node = Node(
        package='control_listener',
        executable='control_listener_node',
        name='control_listener_node',
        output='screen',
    )

    return LaunchDescription([
        control_listener_node,
    ])
