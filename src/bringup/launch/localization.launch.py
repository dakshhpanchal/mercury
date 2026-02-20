from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    fake_encoder = Node(
        package="localization",
        executable="fake_encoder",
        name="fake_encoder",
        output="screen"
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py"
            ])
        ),
        launch_arguments={
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2"
        }.items()
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("localization"),
                "config",
                "ekf.yaml"
            ])
        ]
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("bringup"),
                "launch",
                "lidar.launch.py"
            ])
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py"
            ])
        ),
        launch_arguments={
            "odom_topic": "/odometry/filtered",
            "scan_topic": "/scan",
            "base_frame": "base_link",
            "odom_frame": "odom",
            "map_frame": "map",
            "laser_frame": "laser"
        }.items()
    )

    return LaunchDescription([
        fake_encoder,
        realsense_launch,
        ekf,
        lidar_launch,
        slam
    ])