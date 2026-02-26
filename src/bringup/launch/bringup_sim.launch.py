from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_desc = get_package_share_directory('description')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')

    doc = xacro.process_file(
        xacro_file,
        mappings={}
    )
    
    doc = doc.toxml()

    robot_description = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': doc,
                'use_sim_time': True
            }],
            output='screen'
        )
     
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('simulation'),
                'launch',
                'simulation.launch.py'
            ])
        )
    )

    # base = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(               ##DONT REMOVE
    #         PathJoinSubstitution([
    #             FindPackageShare('bringup'),
    #             'launch',
    #             'bringup_base.launch.py'
    #         ])
    #     )
    # )

    return LaunchDescription([
        simulation,
        robot_description
        #base
    ])