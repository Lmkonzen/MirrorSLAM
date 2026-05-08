#!/usr/bin/env python3
"""
Laptop-side bringup: lidar driver, RTAB-Map, Nav2, RViz, optional gap_explorer.
TimerActions sequence each stage so its dependencies are up by the time it
starts. Kobuki base runs separately on the robot SBC over SSH (see README).

  ros2 launch mirror_slam_bringup mirror_slam_full.launch.py
  ros2 launch mirror_slam_bringup mirror_slam_full.launch.py gap_explorer:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gap_explorer_arg = DeclareLaunchArgument(
        'gap_explorer',
        default_value='false',
        description='Set true to launch gap_explorer wall-follower.',
    )

    nav2_bringup_share  = FindPackageShare('nav2_bringup')
    unitree_lidar_share = FindPackageShare('unitree_lidar_ros2')
    mirror_slam_share   = FindPackageShare('mirror_slam_bringup')

    nav2_params_file = PathJoinSubstitution([
        mirror_slam_share, 'params', 'nav2_params.yaml'
    ])

    # ---- Unitree L1 driver ----
    # Permission denied on /dev/ttyUSB0? add yourself to dialout and re-login:
    #   sudo usermod -aG dialout $USER
    unitree_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([unitree_lidar_share, 'launch.py'])
        ]),
    )

    # ---- /cmd_vel_smoothed -> /commands/velocity ----
    # Both gap_explorer and nav2's velocity_smoother publish to /cmd_vel_smoothed.
    # We tap that directly instead of /cmd_vel_safe — collision_monitor was
    # tripping constantly on lidar self-returns and we shelved it. Re-enable
    # the safe path by switching the source to /cmd_vel_safe once the polygons
    # / scan slice are dialed in so the monitor stops firing on the robot itself.
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_to_kobuki_relay',
        arguments=['/cmd_vel_smoothed', '/commands/velocity'],
        output='screen',
    )

    # ---- RTAB-Map (custom launch) ----
    # Local copy of upstream lidar3d_assemble.launch.py with two changes:
    #   1) Grid/MaxObstacleHeight=1.5, Grid/MaxGroundHeight=0.1 baked in
    #   2) base_link -> unilidar_lidar static TF created inline
    # Don't add a separate static_transform_publisher for the lidar frame;
    # you'll get duplicate edges in the TF tree.
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                mirror_slam_share, 'launch', 'lidar3d_assemble_custom.launch.py'
            ])
        ]),
        launch_arguments={
            'lidar_topic': '/unilidar/cloud',
            'imu_topic':   '/unilidar/imu',
            'frame_id':    'base_link',
        }.items(),
    )

    # ---- 3D cloud -> 2D scan for Nav2 ----
    # Nav2's costmap obstacle layer + collision_monitor consume LaserScan,
    # not PointCloud2. Slice between min/max_height to drop the floor and
    # the ceiling/arm. Tighten range_min if you start seeing self-returns.
    pc_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/assembled_cloud'),
            ('scan',     '/unilidar/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'min_height':   -0.3,
            'max_height':    0.75,
            'range_min':     0.01,
            'range_max':    20.0,
        }],
        output='screen',
    )

    # ---- Nav2 ----
    # No SLAM bundled — RTAB-Map provides /map and the map->odom TF.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                nav2_bringup_share, 'launch', 'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': nav2_params_file,
            'autostart':   'True',
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                nav2_bringup_share, 'launch', 'rviz_launch.py'
            ])
        ]),
    )

    # ---- gap_explorer (off by default; pass gap_explorer:=true to enable) ----
    gap_explorer = Node(
        package='gap_explorer',
        executable='gap_explorer_node',
        name='gap_explorer',
        parameters=[{'scan_topic': '/unilidar/scan'}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gap_explorer')),
    )

    # ---- Startup order ----
    # Lidar driver and the cmd_vel relay come up at T+0. Then in sequence:
    #   +5s   RTAB-Map               (needs /unilidar/cloud)
    #   +5s   pointcloud_to_scan     (needs /assembled_cloud — queued; fine)
    #   +10s  Nav2                   (needs /map, TF, /unilidar/scan)
    #   +10s  RViz                   (start late so it sees everything)
    #   +10s  gap_explorer           (optional; needs Nav2 fully up)
    return LaunchDescription([
        gap_explorer_arg,
        unitree_lidar,
        cmd_vel_relay,
        TimerAction(period=5.0,  actions=[rtabmap]),
        TimerAction(period=5.0,  actions=[pc_to_scan]),
        TimerAction(period=10.0, actions=[nav2]),
        TimerAction(period=10.0, actions=[rviz]),
        TimerAction(period=10.0, actions=[gap_explorer]),
    ])