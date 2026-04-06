"""
monitoring_all.launch.py
========================
Launches all new monitoring components together:
  - system_monitor_node   → /system_status
  - watchdog_node         → /system_alerts
  - waypoint_detector_node → /waypoint_reached, /waypoint_status
  - control_listener_node  (read-only integration)

Usage:
  ros2 launch system_monitor monitoring_all.launch.py

Optional overrides:
  monitor_interval:=2.0
  topic_timeout:=5.0
  arrival_radius:=0.5
  odom_topic:=/diff_drive_controller/odom
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────────────
    declare_monitor_interval = DeclareLaunchArgument(
        'monitor_interval', default_value='2.0',
        description='System monitor check interval (s)'
    )
    declare_topic_timeout = DeclareLaunchArgument(
        'topic_timeout', default_value='5.0',
        description='Topic inactivity timeout (s)'
    )
    declare_arrival_radius = DeclareLaunchArgument(
        'arrival_radius', default_value='0.5',
        description='Waypoint arrival radius (m)'
    )
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic', default_value='/diff_drive_controller/odom',
        description='Odometry topic for waypoint detection'
    )

    # ── Sub-launches ──────────────────────────────────────────────────────────
    system_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('system_monitor'),
                'launch',
                'system_monitor.launch.py'
            ])
        ),
        launch_arguments={
            'monitor_interval': LaunchConfiguration('monitor_interval'),
        }.items()
    )

    watchdog = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('watchdog'),
                'launch',
                'watchdog.launch.py'
            ])
        ),
        launch_arguments={
            'topic_timeout': LaunchConfiguration('topic_timeout'),
        }.items()
    )

    waypoint_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('waypoint_manager'),
                'launch',
                'waypoint_manager.launch.py'
            ])
        ),
        launch_arguments={
            'arrival_radius': LaunchConfiguration('arrival_radius'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }.items()
    )

    control_listener = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('control_listener'),
                'launch',
                'control_listener.launch.py'
            ])
        )
    )

    return LaunchDescription([
        declare_monitor_interval,
        declare_topic_timeout,
        declare_arrival_radius,
        declare_odom_topic,
        system_monitor,
        watchdog,
        waypoint_manager,
        control_listener,
    ])
