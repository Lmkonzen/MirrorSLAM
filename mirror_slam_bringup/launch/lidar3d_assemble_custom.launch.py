#!/usr/bin/env python3
"""
MirrorSLAM custom version of rtabmap_examples/lidar3d_assemble.launch.py.

Identical to upstream except for added Grid/* parameters that are otherwise
unreachable through the upstream launch file's arguments. This is the cleanest
way to inject these — RTAB-Map's Grid/* params get cached at startup, so they
must be set during node construction, not after.

Customizations marked with `# CUSTOM` comments:
  - Grid/MaxObstacleHeight: 1.5  (skip ceiling)
  - Grid/MaxGroundHeight: 0.1    (treat below 10cm as ground/free)

Also: the static TF for base_link -> unilidar_lidar with z=0.18 (not the 0.15
default in upstream).
"""
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args, **kwargs):

    frame_id = LaunchConfiguration('frame_id')
    external_odom_frame_id = LaunchConfiguration('external_odom_frame_id').perform(context)
    fixed_frame_from_imu = False
    fixed_frame_id = LaunchConfiguration('fixed_frame_id').perform(context)
    if not fixed_frame_id:
        if external_odom_frame_id:
            fixed_frame_id = external_odom_frame_id
        else:
            fixed_frame_from_imu = True
            fixed_frame_id = frame_id.perform(context) + "_stabilized"

    imu_topic = LaunchConfiguration('imu_topic')

    rgbd_image_topic = LaunchConfiguration('rgbd_image_topic')
    rgbd_images_topic = LaunchConfiguration('rgbd_images_topic')
    rgbd_image_used = rgbd_image_topic.perform(context) != '' or rgbd_images_topic.perform(context) != ''
    rgbd_cameras = 0 if rgbd_images_topic.perform(context) != '' else 1

    lidar_topic = LaunchConfiguration('lidar_topic')
    lidar_topic_value = lidar_topic.perform(context)
    lidar_topic_deskewed = lidar_topic_value + "/deskewed"

    voxel_size = LaunchConfiguration('voxel_size')
    voxel_size_value = float(voxel_size.perform(context))

    use_sim_time = LaunchConfiguration('use_sim_time')

    localization = LaunchConfiguration('localization').perform(context)
    localization = localization == 'true' or localization == 'True'

    deskewing_slerp = LaunchConfiguration('deskewing_slerp').perform(context)
    deskewing_slerp = deskewing_slerp == 'true' or deskewing_slerp == 'True'

    max_correspondence_distance = voxel_size_value * 10.0

    shared_parameters = {
        'use_sim_time': use_sim_time,
        'frame_id': frame_id,
        'qos': LaunchConfiguration('qos'),
        'approx_sync': rgbd_image_used,
        'wait_for_transform': 0.2,
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '10',
        'Icp/VoxelSize': str(voxel_size_value),
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/MaxTranslation': '3',
        'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.7',
    }

    icp_odometry_parameters = {
        'expected_update_rate': LaunchConfiguration('expected_update_rate'),
        'wait_imu_to_init': True,
        'odom_frame_id': 'odom',
        'guess_frame_id': fixed_frame_id,
        'Odom/ScanKeyFrameThr': '0.4',
        'OdomF2M/ScanSubtractRadius': str(voxel_size_value),
        'OdomF2M/ScanMaxSize': '15000',
        'OdomF2M/BundleAdjustment': 'false',
        'Icp/CorrespondenceRatio': '0.01',
    }

    rtabmap_parameters = {
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_odom_info': not external_odom_frame_id,
        'subscribe_scan_cloud': True,
        'map_frame_id': 'map',
        'odom_frame_id': (external_odom_frame_id if external_odom_frame_id else ""),
        'odom_sensor_sync': True,
        'Rtabmap/DetectionRate': '0',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/CreateOccupancyGrid': 'true',
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
        'Reg/Strategy': '1',
        'Icp/CorrespondenceRatio': str(LaunchConfiguration('min_loop_closure_overlap').perform(context)),

        # CUSTOM — Grid params for occupancy grid generation:
        'Grid/MaxObstacleHeight': '1.5',
        'Grid/RayTracing': 'true',           # ADD: clears free space along ray paths
        'Grid/MaxGroundHeight': '0.1',
    }

    remappings = [('imu', imu_topic), ('odom', 'odom')]
    if rgbd_image_used:
        if rgbd_cameras == 1:
            remappings.append(('rgbd_image', LaunchConfiguration('rgbd_image_topic')))
        else:
            remappings.append(('rgbd_images', LaunchConfiguration('rgbd_images_topic')))

    arguments = []
    if localization:
        rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
        rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
    else:
        arguments.append('-d')

    if external_odom_frame_id:
        viz_topic = lidar_topic_deskewed
    else:
        viz_topic = 'odom_filtered_input_scan'

    nodes = [
        # CUSTOM — base_link -> unilidar_lidar with z=0.18 (was 0.15 upstream)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.05',
                '--y', '0.0',
                '--z', '0.18',
                '--yaw', '0.0',
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'unilidar_lidar',
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--yaw', '0.0',
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'unilidar_lidar',
                '--child-frame-id', 'unilidar_imu',
            ],
            output='screen',
        ),
        Node(
            package='rtabmap_util', executable='lidar_deskewing', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'fixed_frame_id': fixed_frame_id,
                'wait_for_transform': 0.2,
                'slerp': deskewing_slerp,
            }],
            remappings=[('input_cloud', lidar_topic)],
        ),
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'assembling_time': LaunchConfiguration('assembling_time'),
                'fixed_frame_id': (external_odom_frame_id if external_odom_frame_id else ""),
            }],
            remappings=[('cloud', lidar_topic_deskewed), ('odom', 'odom')],
        ),
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[shared_parameters, rtabmap_parameters,
                        {'subscribe_rgbd': rgbd_image_used,
                         'rgbd_cameras': rgbd_cameras,
                         'topic_queue_size': 40,
                         'sync_queue_size': 40}],
            remappings=remappings + [('scan_cloud', 'assembled_cloud'),
                                     ('gps/fix', LaunchConfiguration('gps_topic'))],
            arguments=arguments,
        ),
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[shared_parameters, rtabmap_parameters,
                        {'odometry_node_name': "icp_odometry"}],
            remappings=remappings + [('scan_cloud', viz_topic)],
        ),
    ]

    if not external_odom_frame_id:
        nodes.append(
            Node(
                package='rtabmap_odom', executable='icp_odometry', output='screen',
                parameters=[shared_parameters, icp_odometry_parameters],
                remappings=remappings + [('scan_cloud', lidar_topic_deskewed)],
            )
        )

    if fixed_frame_from_imu:
        nodes.append(
            Node(
                package='rtabmap_util', executable='imu_to_tf', output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'fixed_frame_id': fixed_frame_id,
                    'base_frame_id': frame_id,
                    'wait_for_transform_duration': 0.001,
                }],
                remappings=[('imu/data', imu_topic)],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulated clock.'),
        DeclareLaunchArgument('frame_id', default_value='velodyne',
                              description='Base frame of the robot.'),
        DeclareLaunchArgument('fixed_frame_id', default_value='',
                              description='Fixed frame used for lidar deskewing.'),
        DeclareLaunchArgument('external_odom_frame_id', default_value='',
                              description='Provide external odometry with TF, disabling icp_odometry.'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localization mode.'),
        DeclareLaunchArgument('lidar_topic', default_value='/velodyne_points',
                              description='Name of the lidar PointCloud2 topic.'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data',
                              description='Name of an IMU topic.'),
        DeclareLaunchArgument('gps_topic', default_value='/gps/fix',
                              description='Name of a GPS topic.'),
        DeclareLaunchArgument('rgbd_image_topic', default_value='',
                              description='RGBD image topic (ignored if empty).'),
        DeclareLaunchArgument('rgbd_images_topic', default_value='',
                              description='RGBD images topic (ignored if empty).'),
        DeclareLaunchArgument('voxel_size', default_value='0.1',
                              description='Voxel size (m) of the downsampled lidar point cloud.'),
        DeclareLaunchArgument('min_loop_closure_overlap', default_value='0.2',
                              description='Minimum scan overlap percentage to accept a loop closure.'),
        DeclareLaunchArgument('expected_update_rate', default_value='15.0',
                              description='Expected lidar frame rate.'),
        DeclareLaunchArgument('assembling_time', default_value='1.0',
                              description='How much time (sec) to assemble lidar scans.'),
        DeclareLaunchArgument('deskewing_slerp', default_value='true',
                              description='Use fast slerp interpolation for deskewing.'),
        DeclareLaunchArgument('qos', default_value='1',
                              description='Quality of Service: 0=system default, 1=reliable, 2=best effort.'),
        OpaqueFunction(function=launch_setup),
    ])