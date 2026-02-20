from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('description'),
                'launch',
                'display.launch.py'
            )
        )
    )

    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 256000,
            'frame_id': 'laser_link',
            'scan_mode': 'Standard',
            'angle_compensate': False
        }],
        output='screen'
    )

    return LaunchDescription([
        robot,
        lidar
    ])