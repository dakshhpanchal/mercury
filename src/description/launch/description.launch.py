from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    urdf_file = os.path.join(
        os.getenv('HOME'),
        'localization/mercury/src/description/urdf/robot.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        )

    ])
