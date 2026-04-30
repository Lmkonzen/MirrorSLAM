#!/usr/bin/env python3
"""
MirrorSLAM full bringup (laptop side).

Mirrors the documented manual terminal sequence as a single launch file with
TimerAction delays to sequence dependencies. Each stage waits for the previous
to settle before starting.

Assumes Kobuki is launched separately on the robot's onboard computer over SSH.

Usage:
  ros2 launch mirror_slam_bringup mirror_slam_full.launch.py

Optional:
  ros2 launch mirror_slam_bringup mirror_slam_full.launch.py gap_explorer:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------
    gap_explorer_arg = DeclareLaunchArgument(
        'gap_explorer',
        default_value='false',
        description='Set true to launch gap_explorer wall-follower.',
    )

    # -------------------------------------------------------------------------
    # Paths
    # -------------------------------------------------------------------------
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    unitree_lidar_share = FindPackageShare('unitree_lidar_ros2')
    # CHANGE 1: removed rtabmap_examples_share — we use our custom launch now
    mirror_slam_share = FindPackageShare('mirror_slam_bringup')

    nav2_params_file = PathJoinSubstitution([
        mirror_slam_share, 'params', 'nav2_params.yaml'
    ])

    # -------------------------------------------------------------------------
    # T+0s — Unitree lidar driver
    # NOTE: This launch file cannot run `sudo chmod 666 /dev/ttyUSB0`.
    # Either run it manually before launching, or add yourself to dialout group:
    #   sudo usermod -aG dialout $USER  (then log out and back in)
    # -------------------------------------------------------------------------
    unitree_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([unitree_lidar_share, 'launch.py'])
        ]),
    )

    # -------------------------------------------------------------------------
    # CHANGE 2: REMOVED static_tf_lidar Node entirely.
    # The custom RTAB-Map launch now creates this static TF internally.
    # Having it twice = TF tree conflict.
    # -------------------------------------------------------------------------

    # -------------------------------------------------------------------------
    # T+0s — cmd_vel -> Kobuki bridge
    # Nav2 publishes /cmd_vel; Kobuki listens on /commands/velocity.
    # -------------------------------------------------------------------------
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_to_kobuki_relay',
        arguments=['/cmd_vel_safe', '/commands/velocity'],
        output='screen',
    )

    # -------------------------------------------------------------------------
    # T+5s — RTAB-Map (custom launch with Grid/* params injected)
    # CHANGE 3: switched from rtabmap_examples upstream to our custom version
    # at mirror_slam_bringup/launch/lidar3d_assemble_custom.launch.py.
    # That file is a copy of the upstream with Grid/MaxObstacleHeight=1.5
    # and Grid/MaxGroundHeight=0.1 baked into the rtabmap_parameters dict.
    # The base_link -> unilidar_lidar static TF is also created inside.
    # -------------------------------------------------------------------------
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                mirror_slam_share, 'launch', 'lidar3d_assemble_custom.launch.py'
            ])
        ]),
        launch_arguments={
            'lidar_topic': '/unilidar/cloud',
            'imu_topic': '/unilidar/imu',
            'frame_id': 'base_link',
        }.items(),
    )
    rtabmap_delayed = TimerAction(period=5.0, actions=[rtabmap])

    # -------------------------------------------------------------------------
    # T+5s — pointcloud_to_laserscan (slice from /assembled_cloud)
    # -------------------------------------------------------------------------
    pc_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/assembled_cloud'),
            ('scan', '/unilidar/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'min_height': -0.3,
            'max_height': 0.75,
            'range_min': 0.01,
            'range_max': 20.0,
        }],
        output='screen',
    )
    pc_to_scan_delayed = TimerAction(period=5.0, actions=[pc_to_scan])

    # -------------------------------------------------------------------------
    # T+15s — Nav2 (no SLAM; RTAB-Map provides /map and TF)
    # -------------------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                nav2_bringup_share, 'launch', 'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': nav2_params_file,
            'autostart': 'True',
        }.items(),
    )
    nav2_delayed = TimerAction(period=10.0, actions=[nav2])

    # -------------------------------------------------------------------------
    # T+20s — RViz (Nav2 preconfigured)
    # -------------------------------------------------------------------------
    rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    nav2_bringup_share, 'launch', 'rviz_launch.py'
                ])
            ]),
            launch_arguments={
                'rviz_config': PathJoinSubstitution([
                    mirror_slam_share, 'rviz', 'mirror_slam.rviz'
                ]),
            }.items(),
        )
    rviz_delayed = TimerAction(period=10.0, actions=[rviz])

    # -------------------------------------------------------------------------
    # T+25s (OPTIONAL) — gap_explorer wall-follower
    # -------------------------------------------------------------------------
    gap_explorer = Node(
        package='gap_explorer',
        executable='gap_explorer_node',
        name='gap_explorer',
        parameters=[{'scan_topic': '/unilidar/scan'}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gap_explorer')),
    )
    gap_explorer_delayed = TimerAction(period=10.0, actions=[gap_explorer])

    # -------------------------------------------------------------------------
    return LaunchDescription([
        gap_explorer_arg,

        # T+0s — sensors and basic relays (no dependencies)
        unitree_lidar,
        # CHANGE 2: static_tf_lidar removed from this list too
        cmd_vel_relay,

        # T+5s — RTAB-Map (depends on lidar)
        rtabmap_delayed,

        # T+10s — pointcloud_to_laserscan (depends on /assembled_cloud)
        pc_to_scan_delayed,

        # T+15s — Nav2 (depends on /map, TF, /unilidar/scan)
        nav2_delayed,

        # T+20s — RViz (start last so it sees everything)
        rviz_delayed,

        # T+25s — gap_explorer (optional)
        gap_explorer_delayed,
    ])