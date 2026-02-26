from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    pkg_desc = get_package_share_directory('description')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')

    doc = xacro.process_file(
        xacro_file,
        mappings={}
    )

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'skid_steer_bot'
            ],
            output='screen'
        ),
    ])