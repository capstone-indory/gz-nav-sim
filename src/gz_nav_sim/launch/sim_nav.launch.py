"""Isaac Sim v2 + Nav2 + SLAM launch for XLeRobot Hospital."""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def _optional_launch_arg(context, name: str, cast=None):
    value = LaunchConfiguration(name).perform(context).strip()
    if value == '':
        return None
    return cast(value) if cast is not None else value


def _package_available(name: str) -> bool:
    try:
        get_package_prefix(name)
        return True
    except PackageNotFoundError:
        return False


def _launch(context, *_args, **_kwargs):
    pkg = get_package_share_directory('gz_nav_sim')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    isaac_host = LaunchConfiguration('isaac_host').perform(context).strip() or '127.0.0.1'
    isaac_transport = LaunchConfiguration('isaac_transport').perform(context).strip().lower() or 'xlerobot_ros'
    if isaac_transport not in ('xlerobot_ros', 'rosbridge_v2', 'zmq_v1'):
        raise RuntimeError(
            "isaac_transport must be 'xlerobot_ros', 'rosbridge_v2', or 'zmq_v1', "
            f"got: {isaac_transport}")
    ros_localhost_only_value = LaunchConfiguration('ros_localhost_only').perform(context).strip().lower()
    ros_localhost_only = '1' if ros_localhost_only_value in ('1', 'true', 'yes', 'on') else '0'

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    use_sim_time_arg = 'true' if use_sim_time else 'false'
    foxglove = LaunchConfiguration('use_foxglove').perform(context).lower() == 'true'
    foxglove_profile = LaunchConfiguration('foxglove_profile').perform(context).strip().lower() or 'full'
    if foxglove_profile not in ('full', 'map'):
        raise RuntimeError(
            "foxglove_profile must be 'full' or 'map', "
            f"got: {foxglove_profile}")
    use_da3 = LaunchConfiguration('use_da3').perform(context).lower() == 'true'
    use_nvblox = LaunchConfiguration('use_nvblox').perform(context).lower() == 'true'
    use_vggt_slam = LaunchConfiguration('use_vggt_slam').perform(context).lower() == 'true'
    use_semantic_vlm = LaunchConfiguration('use_semantic_vlm').perform(context).lower() == 'true'
    use_semantic_ocr = LaunchConfiguration('use_semantic_ocr').perform(context).lower() == 'true'
    use_slam_toolbox = LaunchConfiguration('use_slam_toolbox').perform(context).lower() == 'true'
    use_rtabmap = LaunchConfiguration('use_rtabmap').perform(context).lower() == 'true'
    use_imu = LaunchConfiguration('use_imu').perform(context).lower() == 'true'
    rtabmap_odom_source_arg = (
        LaunchConfiguration('rtabmap_odom_source').perform(context).strip().lower()
        or 'fusion'
    )
    rtabmap_fusion_mode = rtabmap_odom_source_arg == 'fusion'
    rtabmap_odom_source = 'rgbd' if rtabmap_fusion_mode else rtabmap_odom_source_arg
    if rtabmap_odom_source not in ('rgbd', 'icp'):
        raise RuntimeError(
            "rtabmap_odom_source must be 'fusion', 'rgbd', or 'icp', "
            f"got: {rtabmap_odom_source_arg}")
    rtabmap_localization = LaunchConfiguration('rtabmap_localization').perform(context).lower() == 'true'
    rtabmap_db = LaunchConfiguration('rtabmap_db').perform(context).strip()
    use_explore = LaunchConfiguration('use_explore').perform(context).lower() == 'true'
    direct_depth = LaunchConfiguration('direct_depth').perform(context).lower() == 'true'
    use_hardware_lidar = LaunchConfiguration('use_hardware_lidar').perform(context).lower() == 'true'
    use_lidar_odom = LaunchConfiguration('use_lidar_odom').perform(context).lower() == 'true'
    enable_base_odom_bridge = (
        LaunchConfiguration('enable_base_odom_bridge').perform(context).lower() == 'true'
    )
    use_nav_goal_bridge = LaunchConfiguration('use_nav_goal_bridge').perform(context).lower() == 'true'
    use_nav_scan_filter = LaunchConfiguration('use_nav_scan_filter').perform(context).lower() == 'true'
    use_slam_scan_filter = LaunchConfiguration('use_slam_scan_filter').perform(context).lower() == 'true'
    use_depth_scan_fallback = LaunchConfiguration('use_depth_scan_fallback').perform(context).lower() == 'true'
    use_binary_rgbd_bridge = LaunchConfiguration('use_binary_rgbd_bridge').perform(context).lower() == 'true'
    use_rtsp_camera_bridge = LaunchConfiguration('use_rtsp_camera_bridge').perform(context).lower() == 'true'
    raw_scan_topic = '/scan_raw' if use_nav_scan_filter else '/scan'
    nav_scan_topic = '/scan'
    slam_input_scan_topic = '/scan_slam' if use_slam_scan_filter else '/scan'

    nvblox_requested = use_nvblox
    nvblox_available = _package_available('nvblox_ros')
    foxglove_available = _package_available('foxglove_bridge')
    explore_available = _package_available('explore_lite')
    rtabmap_conflict_msg = ''
    if use_nvblox and not nvblox_available:
        use_nvblox = False
    if foxglove and not foxglove_available:
        foxglove = False
    if use_explore and not explore_available:
        use_explore = False
    if use_rtabmap and use_slam_toolbox:
        rtabmap_conflict_msg = (
            '[sim_nav] use_rtabmap=true and use_slam_toolbox=true requested; '
            'using RTAB-Map as the single map backend.')
        use_slam_toolbox = False
    if use_rtabmap and rtabmap_odom_source in ('rgbd', 'icp') and use_lidar_odom:
        rtabmap_conflict_msg = (
            '[sim_nav] use_rtabmap=true with rtabmap_odom_source='
            f'{rtabmap_odom_source}; disabling the local Python LiDAR odom node.')
        use_lidar_odom = False
    if use_rtabmap:
        missing_rtabmap = [
            name for name in ('rtabmap_odom', 'rtabmap_slam', 'rtabmap_msgs')
            if not _package_available(name)
        ]
        if missing_rtabmap:
            raise RuntimeError(
                'RTAB-Map requested but packages are missing: '
                + ', '.join(missing_rtabmap)
                + '. Build/install rtabmap_odom, rtabmap_slam, and rtabmap_msgs; '
                + 'this stack no longer falls back to slam_toolbox in RTAB mode.')
    rtabmap_sensor_odom = bool(use_rtabmap and rtabmap_odom_source in ('rgbd', 'icp'))
    rtabmap_use_rgbd = rtabmap_odom_source == 'rgbd'
    rtabmap_use_scan = bool(
        use_rtabmap and (rtabmap_fusion_mode or rtabmap_odom_source == 'icp' or use_slam_scan_filter))

    workspace_root = os.path.abspath(os.path.join(pkg, '..', '..', '..', '..'))
    da3_repo = os.path.join(workspace_root, 'src', 'Depth-Anything-3')
    vggt_slam_repo = os.path.join(workspace_root, 'src', 'VGGT-SLAM')
    publish_raw_rgb = bool(
        use_da3 or use_semantic_vlm or use_semantic_ocr or use_rtabmap or use_nvblox)
    publish_direct_depth = bool(direct_depth)

    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'base_footprint',
        ],
        output='screen',
    )

    top_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='top_base_link_tf',
        arguments=[
            '--x', '0.200', '--y', '0.0', '--z', '0.730',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'top_base_link',
        ],
        output='screen',
    )

    head_pan_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='head_pan_link_tf',
        arguments=[
            '--x', '-0.178', '--y', '0.0', '--z', '0.0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'top_base_link', '--child-frame-id', 'head_pan_link',
        ],
        output='screen',
    )

    head_tilt_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='head_tilt_link_tf',
        arguments=[
            '--x', '0.031', '--y', '0.0', '--z', '0.43815',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'head_pan_link', '--child-frame-id', 'head_tilt_link',
        ],
        output='screen',
    )

    head_camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='head_camera_link_tf',
        arguments=[
            '--x', '0.055', '--y', '0.0', '--z', '0.0225',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'head_tilt_link', '--child-frame-id', 'head_camera_link',
        ],
        output='screen',
    )

    front_camera_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_tf',
        arguments=[
            '--x', LaunchConfiguration('robot_camera_x'),
            '--y', LaunchConfiguration('robot_camera_y'),
            '--z', LaunchConfiguration('robot_camera_z'),
            '--roll', LaunchConfiguration('robot_camera_roll'),
            '--pitch', LaunchConfiguration('robot_camera_pitch'),
            '--yaw', LaunchConfiguration('robot_camera_yaw'),
            '--frame-id', 'head_camera_link', '--child-frame-id', 'camera_frame',
        ],
        output='screen',
    )

    depth_camera_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_camera_frame_tf',
        arguments=[
            '--x', '0.0', '--y', '0.045', '--z', '0.0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'head_camera_link', '--child-frame-id', 'depth_camera_frame',
        ],
        output='screen',
    )

    depth_camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_camera_optical_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.57079632679', '--pitch', '0', '--yaw', '-1.57079632679',
            '--frame-id', 'depth_camera_frame', '--child-frame-id', 'depth_camera_optical_frame',
        ],
        output='screen',
    )

    front_camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.57079632679', '--pitch', '0', '--yaw', '-1.57079632679',
            '--frame-id', 'camera_frame', '--child-frame-id', 'camera_optical_frame',
        ],
        output='screen',
    )

    front_camera_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_imu_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'camera_frame', '--child-frame-id', 'camera_imu_frame',
        ],
        output='screen',
    )

    lidar_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_tf',
        arguments=[
            '--x', LaunchConfiguration('robot_lidar_x'),
            '--y', LaunchConfiguration('robot_lidar_y'),
            '--z', LaunchConfiguration('robot_lidar_z'),
            '--roll', LaunchConfiguration('robot_lidar_roll'),
            '--pitch', LaunchConfiguration('robot_lidar_pitch'),
            '--yaw', LaunchConfiguration('robot_lidar_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', LaunchConfiguration('xlerobot_scan_frame'),
        ],
        output='screen',
    )

    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')
    da3_params = os.path.join(pkg, 'config', 'da3_params.yaml')
    if direct_depth:
        vlm_depth_topic = '/depth/image_raw'
        vlm_camera_info_topic = '/depth/camera_info'
    else:
        vlm_depth_topic = '/depth/image_raw'
        vlm_camera_info_topic = '/depth/camera_info'

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params, {
            'use_sim_time': use_sim_time,
            'use_lifecycle_manager': False,
            'scan_topic': slam_input_scan_topic,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }],
    )
    slam_configure = TimerAction(
        period=3.0,
        actions=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ))],
    )
    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] Slamtoolbox node is activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    nav2_params = os.path.join(pkg, 'config', 'nav2_params_depth_sensor.yaml')
    if not os.path.exists(nav2_params):
        nav2_params = os.path.join(nav2_pkg, 'params', 'nav2_params.yaml')

    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time_arg,
            'autostart': 'true',
            'params_file': nav2_params,
            'use_composition': 'False',
            'container_name': 'nav2_container',
        }.items(),
    )

    da3_overrides = {
        'use_sim_time': use_sim_time,
        'da3_repo_path': da3_repo,
    }
    for key, cast in (
        ('model_id', str),
        ('process_res', int),
        ('process_res_method', str),
        ('inference_rate_hz', float),
        ('input_views', int),
        ('point_cloud_stride', int),
        ('point_cloud_frame', str),
    ):
        value = _optional_launch_arg(context, f'da3_{key}', cast)
        if value is not None:
            da3_overrides[key] = value

    da3_node = Node(
        package='gz_nav_sim',
        executable='da3_depth_node.py',
        name='da3_depth_node',
        output='screen',
        parameters=[da3_params, da3_overrides],
    )

    semantic_vlm_node = Node(
        package='gz_nav_sim',
        executable='semantic_vlm_node.py',
        name='semantic_vlm_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': '/camera/image_raw',
            'depth_topic': vlm_depth_topic,
            'camera_info_topic': vlm_camera_info_topic,
            'detections_topic': '/semantic_vlm/detections',
            'markers_topic': '/semantic_vlm/markers',
            'task_mode': LaunchConfiguration('vlm_task_mode'),
            'model_name': LaunchConfiguration('vlm_model'),
            'device': LaunchConfiguration('vlm_device'),
            'frame_interval': ParameterValue(
                LaunchConfiguration('vlm_frame_interval'), value_type=int),
            'max_new_tokens': ParameterValue(
                LaunchConfiguration('vlm_max_new_tokens'), value_type=int),
            'crop_requery': False,
            'vram_budget_mb': 12288.0,
            'target_frame': 'map',
            'fallback_target_frame': 'odom',
            'confirm_min_observations': 3,
            'confirm_window_s': 120.0,
            'match_radius_m': 1.0,
        }],
    )

    semantic_ocr_node = Node(
        package='gz_nav_sim',
        executable='semantic_ocr_node.py',
        name='semantic_ocr_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': '/camera/image_raw',
            'depth_topic': vlm_depth_topic,
            'camera_info_topic': vlm_camera_info_topic,
            'detections_topic': '/semantic_ocr/detections',
            'markers_topic': '/semantic_ocr/markers',
            'image_annotations_topic': '/semantic_ocr/image_annotations',
            'ocr_backend': LaunchConfiguration('ocr_backend'),
            'ocr_use_gpu': ParameterValue(
                LaunchConfiguration('ocr_use_gpu'), value_type=bool),
            'frame_interval': ParameterValue(
                LaunchConfiguration('ocr_frame_interval'), value_type=int),
            'max_queue_size': ParameterValue(
                LaunchConfiguration('ocr_max_queue_size'), value_type=int),
            'ocr_max_side': ParameterValue(
                LaunchConfiguration('ocr_max_side'), value_type=int),
            'ocr_scales': LaunchConfiguration('ocr_scales'),
            'min_confidence': ParameterValue(
                LaunchConfiguration('ocr_min_confidence'), value_type=float),
            'floor_hint': LaunchConfiguration('ocr_floor_hint'),
            'floor_prior_mode': LaunchConfiguration('ocr_floor_prior_mode'),
            'target_frame': 'map',
            'fallback_target_frame': 'odom',
            'confirm_min_observations': 2,
            'track_max_gap_frames': ParameterValue(
                LaunchConfiguration('ocr_track_max_gap_frames'), value_type=int),
            'track_max_depth_diff_m': ParameterValue(
                LaunchConfiguration('ocr_track_max_depth_diff_m'), value_type=float),
        }],
    )

    pointcloud_visualizer_node = Node(
        package='gz_nav_sim',
        executable='pointcloud_visualizer_node.py',
        name='pointcloud_visualizer_node',
        output='screen',
        parameters=[{
            'input_topic': '/depth/points',
            'output_topic': '/depth/points_visual',
            'max_rate_hz': 1.0,
            'stride': 10,
            'max_points': 8000,
            'voxel_size_m': 0.15,
        }],
    )

    trajectory_path_node = Node(
        package='gz_nav_sim',
        executable='trajectory_path_node.py',
        name='trajectory_path_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/odom',
            'path_topic': '/trajectory',
            'target_frame': 'map',
            'max_poses': 2000,
            'min_translation_m': 0.05,
            'min_rotation_rad': 0.05,
        }],
    )

    vggt_slam_params = os.path.join(pkg, 'config', 'vggt_slam_params.yaml')
    vggt_slam_overrides = {
        'use_sim_time': use_sim_time,
        'server_repo': vggt_slam_repo,
    }
    for key, cast in (
        ('server_python', str),
        ('server_script', str),
        ('submap_size', int),
        ('min_disparity', float),
        ('pointcloud_stride', int),
        ('image_topic', str),
    ):
        value = _optional_launch_arg(context, f'vggt_slam_{key}', cast)
        if value is not None:
            vggt_slam_overrides[key] = value

    vggt_slam_node = Node(
        package='gz_nav_sim',
        executable='vggt_slam_bridge.py',
        name='vggt_slam_bridge',
        output='screen',
        parameters=[vggt_slam_params, vggt_slam_overrides],
    )

    rtabmap_params = os.path.join(pkg, 'config', 'rtabmap_params.yaml')
    rtabmap_db_path = rtabmap_db or os.path.expanduser('~/.ros/rtabmap.db')
    rtabmap_overrides = {
        'use_sim_time': use_sim_time,
        'database_path': rtabmap_db_path,
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'subscribe_depth': rtabmap_use_rgbd,
        'subscribe_rgb': rtabmap_use_rgbd,
        'subscribe_rgbd': False,
        'subscribe_scan': rtabmap_use_scan,
        'subscribe_imu': use_imu,
        'approx_sync': True,
        'approx_sync_max_interval': 0.20,
        'queue_size': 30,
        'sync_queue_size': 30,
        'topic_queue_size': 30,
        'wait_for_transform': 2.0,
        'map_always_update': True,
        'qos_image': 2,
        'qos_camera_info': 1,
        'qos_odom': 1,
        'qos_scan': 2,
        'qos_imu': 2,
        'Mem/IncrementalMemory': 'false' if rtabmap_localization else 'true',
        'Mem/InitWMWithAllNodes': 'true' if rtabmap_localization else 'false',
        'Rtabmap/StartNewMapOnLoopClosure': 'false',
        'Rtabmap/DetectionRate': '2.0',
        'Rtabmap/CreateIntermediateNodes': 'true',
        'Reg/Strategy': '2' if rtabmap_use_rgbd and rtabmap_use_scan else (
            '0' if rtabmap_use_rgbd else '1'),
        'Reg/Force3DoF': 'true',
        'Icp/PointToPlane': 'true',
        'Icp/MaxCorrespondenceDistance': '0.12',
        'Icp/Iterations': '10',
        'Icp/Epsilon': '0.001',
        'Icp/VoxelSize': '0.05',
        'Optimizer/Strategy': '1',
        'Optimizer/Iterations': '20',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/AngularUpdate': '0.2',
        'RGBD/LinearUpdate': '0.2',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '50',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'Mem/UseOdomFeatures': 'false',
        'Mem/DepthCompressionFormat': '.rvl',
        'Mem/ImageCompressionFormat': '.jpg',
        'Mem/RawDescriptorsKept': 'false',
        'Kp/MaxFeatures': '700',
        'Kp/DetectorStrategy': '6',
        'Vis/MinInliers': '25',
        'Vis/InlierDistance': '0.05',
        'Vis/EstimationType': '1',
        'Mem/MemoryThr': '5000',
        'Mem/ReduceGraph': 'true',
        'Rtabmap/MaxRetrieved': '4',
        'Grid/Sensor': '2' if rtabmap_use_rgbd and rtabmap_use_scan else (
            '1' if rtabmap_use_rgbd else '0'),
        'Grid/RangeMax': '10.0',
        'Grid/CellSize': '0.05',
        'Grid/3D': 'false',
        'Grid/RayTracing': 'true',
        'Grid/FootprintHeight': '0.4',
        'Grid/MaxObstacleHeight': '1.5',
        'Grid/NormalsSegmentation': 'false',
    }

    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'subscribe_rgbd': False,
            'subscribe_rgb': True,
            'subscribe_depth': True,
            'subscribe_imu': use_imu,
            'approx_sync': not use_binary_rgbd_bridge,
            'approx_sync_max_interval': 0.12,
            'queue_size': 30,
            'sync_queue_size': 30,
            'topic_queue_size': 30,
            'wait_for_transform': 0.2,
            'qos_image': 2,
            'qos_camera_info': 1,
            'qos_imu': 2,
            'Reg/Strategy': '0',
            'Reg/Force3DoF': 'true',
            'Icp/PointToPlane': 'true',
            'Icp/MaxCorrespondenceDistance': '0.12',
            'Icp/Iterations': '10',
            'Icp/Epsilon': '0.001',
            'Icp/VoxelSize': '0.05',
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
            'Odom/ResetCountdown': '3',
            'OdomF2M/MaxSize': '1000',
            'Kp/MaxFeatures': '700',
            'Kp/DetectorStrategy': '6',
            'Vis/MinInliers': '20',
            'Vis/InlierDistance': '0.05',
            'Vis/EstimationType': '1',
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', vlm_depth_topic),
            ('imu', '/imu/data'),
            ('odom', '/odom'),
        ],
    )

    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'subscribe_scan': True,
            'subscribe_scan_cloud': False,
            'approx_sync': True,
            'queue_size': 30,
            'sync_queue_size': 30,
            'topic_queue_size': 30,
            'wait_for_transform': 0.2,
            'qos_scan': 2,
            'Reg/Force3DoF': 'true',
            'Reg/Strategy': '1',
            'Icp/PointToPlane': 'true',
            'Icp/MaxCorrespondenceDistance': '0.12',
            'Icp/Iterations': '10',
            'Icp/Epsilon': '0.001',
            'Icp/VoxelSize': '0.05',
        }],
        remappings=[
            ('scan', slam_input_scan_topic),
            ('odom', '/odom'),
        ],
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        output='screen',
        parameters=[rtabmap_params, rtabmap_overrides],
        remappings=[
            ('scan', slam_input_scan_topic),
            ('odom', '/odom'),
            ('map', '/map'),
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', vlm_depth_topic),
            ('imu', '/imu/data'),
        ],
        arguments=['--ros-args', '--log-level', 'rtabmap:=info'],
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'twist_mux.yaml'), {
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cmd_vel_out', '/cmd_vel_mux'),
        ],
    )

    nav_destinations_file = LaunchConfiguration('nav_destinations_file').perform(context).strip()
    if not nav_destinations_file:
        nav_destinations_file = os.path.join(pkg, 'config', 'nav_destinations.yaml')

    nav_destination_node = Node(
        package='gz_nav_sim',
        executable='nav_destination_node.py',
        name='nav_destination_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'destinations_file': nav_destinations_file,
            'destination_topic': LaunchConfiguration('nav_destination_topic'),
            'goal_pose2d_topic': LaunchConfiguration('nav_goal_pose2d_topic'),
            'clicked_point_topic': '/clicked_point',
            'goal_topic': '/goal_pose',
            'frame_id': 'map',
            'enable_clicked_point_goal': ParameterValue(
                LaunchConfiguration('nav_goal_from_clicked_point'), value_type=bool),
            'clicked_point_yaw': ParameterValue(
                LaunchConfiguration('nav_clicked_point_yaw'), value_type=float),
        }],
    )

    scan_nav_filter_node = Node(
        package='gz_nav_sim',
        executable='scan_slam_filter_node.py',
        name='scan_nav_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': raw_scan_topic,
            'output_topic': nav_scan_topic,
            'min_range_m': ParameterValue(
                LaunchConfiguration('nav_scan_filter_min_range'), value_type=float),
            'max_range_m': ParameterValue(
                LaunchConfiguration('nav_scan_filter_max_range'), value_type=float),
            'remove_isolated_clusters': ParameterValue(
                LaunchConfiguration('nav_scan_filter_remove_isolated_clusters'), value_type=bool),
            'min_cluster_points': ParameterValue(
                LaunchConfiguration('nav_scan_filter_min_cluster_points'), value_type=int),
            'cluster_jump_m': ParameterValue(
                LaunchConfiguration('nav_scan_filter_cluster_jump_m'), value_type=float),
            'cluster_max_range_m': ParameterValue(
                LaunchConfiguration('nav_scan_filter_cluster_max_range'), value_type=float),
        }],
    )

    scan_slam_filter_node = Node(
        package='gz_nav_sim',
        executable='scan_slam_filter_node.py',
        name='scan_slam_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': raw_scan_topic,
            'output_topic': '/scan_slam',
            'min_range_m': ParameterValue(
                LaunchConfiguration('slam_scan_filter_min_range'), value_type=float),
            'max_range_m': ParameterValue(
                LaunchConfiguration('slam_scan_filter_max_range'), value_type=float),
            'remove_isolated_clusters': ParameterValue(
                LaunchConfiguration('slam_scan_filter_remove_isolated_clusters'), value_type=bool),
            'min_cluster_points': ParameterValue(
                LaunchConfiguration('slam_scan_filter_min_cluster_points'), value_type=int),
            'cluster_jump_m': ParameterValue(
                LaunchConfiguration('slam_scan_filter_cluster_jump_m'), value_type=float),
            'cluster_max_range_m': ParameterValue(
                LaunchConfiguration('slam_scan_filter_cluster_max_range'), value_type=float),
        }],
    )

    depth_scan_node = Node(
        package='gz_nav_sim',
        executable='depth_to_laserscan_node.py',
        name='depth_to_laserscan_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'depth_topic': '/depth/image_raw',
            'camera_info_topic': '/depth/camera_info',
            'scan_topic': raw_scan_topic,
            'scan_frame': 'base_link',
            'scan_height_px': 24,
            'range_min_m': ParameterValue(
                LaunchConfiguration('nav_scan_filter_min_range'), value_type=float),
            'range_max_m': ParameterValue(
                LaunchConfiguration('depth_sensor_pointcloud_max_depth'), value_type=float),
            'publish_rate_hz': ParameterValue(
                LaunchConfiguration('depth_scan_publish_rate_hz'), value_type=float),
        }],
    )

    laser_scan_odometry_node = Node(
        package='gz_nav_sim',
        executable='laser_scan_odometry_node.py',
        name='laser_scan_odometry_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_topic': slam_input_scan_topic,
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
            'min_range_m': ParameterValue(
                LaunchConfiguration('slam_scan_filter_min_range'), value_type=float),
            'max_range_m': ParameterValue(
                LaunchConfiguration('lidar_odom_max_range'), value_type=float),
            'max_points': ParameterValue(
                LaunchConfiguration('lidar_odom_max_points'), value_type=int),
            'icp_iterations': ParameterValue(
                LaunchConfiguration('lidar_odom_icp_iterations'), value_type=int),
            'max_correspondence_distance_m': ParameterValue(
                LaunchConfiguration('lidar_odom_max_correspondence_distance'), value_type=float),
            'min_pairs': ParameterValue(
                LaunchConfiguration('lidar_odom_min_pairs'), value_type=int),
            'max_translation_per_scan_m': ParameterValue(
                LaunchConfiguration('lidar_odom_max_translation_per_scan'), value_type=float),
            'max_rotation_per_scan_rad': ParameterValue(
                LaunchConfiguration('lidar_odom_max_rotation_per_scan'), value_type=float),
            'invert_delta': ParameterValue(
                LaunchConfiguration('lidar_odom_invert_delta'), value_type=bool),
        }],
    )

    hardware_lidar_node = Node(
        package='gz_nav_sim',
        executable='rplidar_c1_scan_node.py',
        name='rplidar_c1_scan_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'serial_port': LaunchConfiguration('hardware_lidar_serial'),
            'baud': ParameterValue(
                LaunchConfiguration('hardware_lidar_baud'), value_type=int),
            'scan_topic': '/scan',
            'frame_id': LaunchConfiguration('hardware_lidar_frame'),
            'samples_per_scan': ParameterValue(
                LaunchConfiguration('hardware_lidar_samples'), value_type=int),
            'angle_offset_deg': ParameterValue(
                LaunchConfiguration('hardware_lidar_angle_offset_deg'), value_type=float),
            'invert': ParameterValue(
                LaunchConfiguration('hardware_lidar_invert'), value_type=bool),
            'range_min': ParameterValue(
                LaunchConfiguration('hardware_lidar_range_min'), value_type=float),
            'range_max': ParameterValue(
                LaunchConfiguration('hardware_lidar_range_max'), value_type=float),
            'min_quality': ParameterValue(
                LaunchConfiguration('hardware_lidar_min_quality'), value_type=int),
        }],
    )

    depth_pointcloud_node = Node(
        package='gz_nav_sim',
        executable='depth_to_pointcloud_node.py',
        name='depth_sensor_to_pointcloud',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'depth_topic': '/depth/image_raw',
            'camera_info_topic': '/depth/camera_info',
            'points_topic': '/depth/points',
            'stride': ParameterValue(
                LaunchConfiguration('depth_sensor_pointcloud_stride'), value_type=int),
            'min_depth_m': ParameterValue(
                LaunchConfiguration('depth_sensor_pointcloud_min_depth'), value_type=float),
            'max_depth_m': ParameterValue(
                LaunchConfiguration('depth_sensor_pointcloud_max_depth'), value_type=float),
            'max_points': ParameterValue(
                LaunchConfiguration('depth_sensor_pointcloud_max_points'), value_type=int),
        }],
    )

    binary_rgbd_bridge_node = Node(
        package='gz_nav_sim',
        executable='binary_rgbd_bridge_node.py',
        name='binary_rgbd_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'listen_host': LaunchConfiguration('binary_rgbd_host'),
            'listen_port': ParameterValue(
                LaunchConfiguration('binary_rgbd_port'), value_type=int),
            'color_image_topic': '/camera/image_raw',
            'color_compressed_topic': '/camera/image_raw/compressed',
            'color_info_topic': '/camera/camera_info',
            'depth_image_topic': '/depth/image_raw',
            'depth_info_topic': '/depth/camera_info',
            'color_frame_id': 'camera_optical_frame',
            'depth_frame_id': 'camera_optical_frame',
            'publish_color_raw': publish_raw_rgb,
            'publish_color_compressed': True,
        }],
    )

    rtsp_camera_bridge_node = Node(
        package='gz_nav_sim',
        executable='rtsp_compressed_bridge_node.py',
        name='rtsp_compressed_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_names': LaunchConfiguration('rtsp_camera_names'),
            'publish_rate_hz': ParameterValue(
                LaunchConfiguration('rtsp_camera_publish_rate_hz'), value_type=float),
            'jpeg_quality': ParameterValue(
                LaunchConfiguration('rtsp_camera_jpeg_quality'), value_type=int),
            'base.url': LaunchConfiguration('rtsp_base_camera_url'),
            'base.image_topic': '/xlerobot/base_camera/image/compressed',
            'base.info_topic': '/xlerobot/base_camera/camera_info',
            'base.frame_id': 'base_camera_optical_frame',
            'wrist_left.url': LaunchConfiguration('rtsp_wrist_left_camera_url'),
            'wrist_left.image_topic': '/xlerobot/wrist_left_camera/image/compressed',
            'wrist_left.info_topic': '/xlerobot/wrist_left_camera/camera_info',
            'wrist_left.frame_id': 'wrist_left_camera_optical_frame',
            'wrist_right.url': LaunchConfiguration('rtsp_wrist_right_camera_url'),
            'wrist_right.image_topic': '/xlerobot/wrist_right_camera/image/compressed',
            'wrist_right.info_topic': '/xlerobot/wrist_right_camera/camera_info',
            'wrist_right.frame_id': 'wrist_right_camera_optical_frame',
        }],
    )

    if direct_depth:
        nvblox_depth_topic = '/depth/image_raw'
        nvblox_depth_info_topic = '/depth/camera_info'
    else:
        nvblox_depth_topic = '/depth/image_raw'
        nvblox_depth_info_topic = '/depth/camera_info'

    nvblox_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'nvblox.launch.py')),
        launch_arguments={
            'depth_topic': nvblox_depth_topic,
            'depth_info_topic': nvblox_depth_info_topic,
            'color_topic': '/camera/image_raw',
            'color_info_topic': '/camera/camera_info',
            'use_sim_time': use_sim_time_arg,
        }.items(),
    )

    nvblox_gltf_node = Node(
        package='gz_nav_sim',
        executable='nvblox_mesh_to_gltf.py',
        name='nvblox_mesh_to_gltf',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/nvblox_node/mesh',
            'output_topic': '/nvblox_node/scene',
            'accumulate_only': True,
        }],
    )

    isaac_robot_id = int(LaunchConfiguration('isaac_robot_id').perform(context).strip() or '0')
    isaac_camera_link = (LaunchConfiguration('isaac_camera_link')
                         .perform(context).strip() or 'head_tilt')
    isaac_zmq_bridge_node = Node(
        package='gz_nav_sim',
        executable='isaac_bridge.py',
        name='isaac_bridge',
        output='screen',
        parameters=[{
            'host': isaac_host,
            'pub_port': 5555,
            'push_port': 5556,
            'rep_port': 5557,
            'cmd_rate_hz': 100.0,
            'robot_id': isaac_robot_id,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'camera_optical_frame': 'camera_optical_frame',
            'lidar_frame': 'base_link',
            'camera_link_name': isaac_camera_link,
        }],
    )
    xlerobot_v2_bridge_node = Node(
        package='gz_nav_sim',
        executable='xlerobot_v2_bridge.py',
        name='xlerobot_v2_bridge',
        output='screen',
        parameters=[{
            'cmd_rate_hz': 100.0,
            'cmd_timeout_sec': 1.0,
            'max_linear_x': ParameterValue(
                LaunchConfiguration('cmd_max_linear_x'), value_type=float),
            'max_linear_y': ParameterValue(
                LaunchConfiguration('cmd_max_linear_y'), value_type=float),
            'max_angular_z': ParameterValue(
                LaunchConfiguration('cmd_max_angular_z'), value_type=float),
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'camera_optical_frame': 'camera_optical_frame',
            'imu_frame': 'camera_imu_frame',
            'scan_frame': LaunchConfiguration('xlerobot_scan_frame'),
            'enable_odom_bridge': (
                enable_base_odom_bridge and not use_lidar_odom and not rtabmap_sensor_odom),
            'enable_scan_bridge': not use_hardware_lidar,
            'enable_imu_bridge': use_imu,
            'enable_cmd_bridge': False,
            'cmd_vel_in_topic': '/cmd_vel_mux',
            'cmd_vel_in_topics': '/cmd_vel_teleop,/cmd_teleop,/cmd_vel_mux,/cmd_vel',
            'cmd_vel_out_topic': '/xlerobot/cmd_vel',
            'odom_in_topic': '/xlerobot/odom',
            'odom_out_topic': '/odom',
            'rgb_image_in_topic': '/xlerobot/head_camera/color/image_raw',
            'rgb_image_out_topic': '/camera/image_raw',
            'rgb_compressed_image_in_topic': (
                '' if use_binary_rgbd_bridge else '/xlerobot/head_camera/color/image'),
            'rgb_compressed_image_in_topics': (
                '' if use_binary_rgbd_bridge else
                '/xlerobot/head_camera/color/image'),
            'rgb_info_in_topic': (
                '' if use_binary_rgbd_bridge else '/xlerobot/head_camera/color/camera_info'),
            'rgb_info_in_topics': (
                '' if use_binary_rgbd_bridge else '/xlerobot/head_camera/color/camera_info'),
            'rgb_compressed_out_topic': '/camera/image_raw/compressed',
            'rgb_info_out_topic': '/camera/camera_info',
            'publish_rgb_raw': publish_raw_rgb and not use_binary_rgbd_bridge,
            'depth_image_in_topic': (
                '' if use_binary_rgbd_bridge else '/xlerobot/head_camera/depth/image_rect_raw'),
            'depth_image_in_topics': (
                '' if use_binary_rgbd_bridge else '/xlerobot/head_camera/depth/image_rect_raw'),
            'depth_compressed_image_in_topic': (
                '' if use_binary_rgbd_bridge else '/xlerobot/head_camera/depth/image'),
            'depth_compressed_image_in_topics': (
                '' if use_binary_rgbd_bridge else '/xlerobot/head_camera/depth/image'),
            'depth_info_in_topic': (
                '' if use_binary_rgbd_bridge else '/xlerobot/head_camera/depth/camera_info'),
            'depth_info_in_topics': (
                '' if use_binary_rgbd_bridge else '/xlerobot/head_camera/depth/camera_info'),
            'depth_image_out_topic': '/depth/image_raw',
            'depth_info_out_topic': '/depth/camera_info',
            'publish_depth_raw': publish_direct_depth and not use_binary_rgbd_bridge,
            'imu_in_topic': '/xlerobot/head_camera/imu',
            'imu_in_topics': '/xlerobot/head_camera/imu,/xlerobot/imu/data',
            'imu_out_topic': '/imu/data',
            'synthesize_camera_info': True,
            'scan_in_topic': '/xlerobot/scan',
            'scan_out_topic': raw_scan_topic,
        }],
    )
    low_latency_cmd_bridge_node = Node(
        package='gz_nav_sim',
        executable='low_latency_cmd_bridge.py',
        name='low_latency_cmd_bridge',
        output='screen',
        parameters=[{
            'cmd_vel_in_topics': '/cmd_vel_teleop,/cmd_teleop,/cmd_vel_mux,/cmd_vel',
            'cmd_vel_out_topic': '/xlerobot/cmd_vel',
            'cmd_timeout_sec': 1.0,
            'repeat_rate_hz': 250.0,
            'max_linear_x': ParameterValue(
                LaunchConfiguration('cmd_max_linear_x'), value_type=float),
            'max_linear_y': ParameterValue(
                LaunchConfiguration('cmd_max_linear_y'), value_type=float),
            'max_angular_z': ParameterValue(
                LaunchConfiguration('cmd_max_angular_z'), value_type=float),
        }],
    )

    launch_actions = [
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', ros_localhost_only),
    ]
    if rtabmap_conflict_msg:
        launch_actions.append(LogInfo(msg=rtabmap_conflict_msg))
    if isaac_transport == 'zmq_v1':
        launch_actions.append(LogInfo(
            msg=f'[sim_nav] Isaac backend: ZMQ v1 bridge to tcp://{isaac_host}:5555/5556/5557'))
        launch_actions.append(isaac_zmq_bridge_node)
    else:
        launch_actions.append(LogInfo(
            msg='[sim_nav] XLeRobot ROS topics under /xlerobot'))
        if use_binary_rgbd_bridge:
            launch_actions.append(LogInfo(
                msg='[sim_nav] depth sensor RGB-D transport: Pi binary TCP -> ROS 2 camera topics'))
            launch_actions.append(binary_rgbd_bridge_node)
        if use_rtsp_camera_bridge:
            launch_actions.append(LogInfo(
                msg='[sim_nav] RTSP camera previews: MediaMTX RTSP -> ROS 2 compressed image topics for Foxglove'))
            launch_actions.append(rtsp_camera_bridge_node)
        launch_actions.append(low_latency_cmd_bridge_node)
        launch_actions.append(xlerobot_v2_bridge_node)

    launch_actions.extend([
        base_footprint_tf,
        top_base_link_tf,
        head_pan_link_tf,
        head_tilt_link_tf,
        head_camera_link_tf,
        front_camera_frame_tf,
        front_camera_optical_tf,
        depth_camera_frame_tf,
        depth_camera_optical_tf,
        front_camera_imu_tf,
        lidar_frame_tf,
        pointcloud_visualizer_node,
        trajectory_path_node,
        twist_mux_node,
    ])
    if use_depth_scan_fallback:
        launch_actions.append(depth_scan_node)
        if use_lidar_odom:
            launch_actions.append(LogInfo(
                msg='[sim_nav] depth scan fallback enabled: depth sensor depth -> /scan_raw -> scan ICP /odom'))
    if use_nav_scan_filter:
        launch_actions.append(scan_nav_filter_node)
    if use_slam_scan_filter:
        launch_actions.append(scan_slam_filter_node)
    if use_lidar_odom:
        launch_actions.append(laser_scan_odometry_node)
    if use_slam_toolbox:
        slam_actions = [slam, slam_configure, slam_activate]
        if use_lidar_odom or use_slam_scan_filter:
            launch_actions.append(TimerAction(period=8.0, actions=slam_actions))
            launch_actions.append(LogInfo(
                msg='[sim_nav] delaying slam_toolbox until filtered scan/LiDAR odom are ready'))
        else:
            launch_actions.extend(slam_actions)
    if use_rtabmap:
        if rtabmap_fusion_mode:
            launch_actions.append(TimerAction(period=4.0, actions=[rgbd_odometry_node]))
            launch_actions.append(LogInfo(
                msg='[sim_nav] RTAB fusion: RGB-D visual odom + LiDAR scan occupancy/refinement; base/wheel odom bridge disabled'))
        elif rtabmap_odom_source == 'rgbd':
            launch_actions.append(TimerAction(period=4.0, actions=[rgbd_odometry_node]))
            launch_actions.append(LogInfo(
                msg='[sim_nav] RTAB odom source: RGB-D + IMU from depth sensor; base/wheel odom bridge disabled'))
        elif rtabmap_odom_source == 'icp':
            launch_actions.append(TimerAction(period=4.0, actions=[icp_odometry_node]))
            launch_actions.append(LogInfo(
                msg='[sim_nav] RTAB odom source: LiDAR ICP from scan; base/wheel odom bridge disabled'))
        launch_actions.append(TimerAction(period=6.0, actions=[rtabmap_node]))
    if use_hardware_lidar:
        launch_actions.append(hardware_lidar_node)
    if publish_direct_depth:
        launch_actions.append(depth_pointcloud_node)
    launch_actions.extend([nav2_container, navigation])
    if use_nav_goal_bridge:
        launch_actions.append(nav_destination_node)
    if use_da3:
        launch_actions.append(da3_node)
    if use_semantic_vlm:
        launch_actions.append(TimerAction(period=12.0, actions=[semantic_vlm_node]))
    elif LaunchConfiguration('use_semantic_vlm').perform(context).lower() == 'true':
        launch_actions.append(LogInfo(msg='[sim_nav] semantic VLM disabled'))
    if use_semantic_ocr:
        launch_actions.append(TimerAction(period=10.0, actions=[semantic_ocr_node]))
    if use_nvblox:
        launch_actions.append(TimerAction(period=8.0, actions=[nvblox_include]))
        launch_actions.append(TimerAction(period=10.0, actions=[nvblox_gltf_node]))
    elif LaunchConfiguration('use_nvblox').perform(context).lower() == 'true':
        launch_actions.append(LogInfo(msg='[sim_nav] nvblox_ros not found; continuing without nvblox'))
    if use_vggt_slam:
        launch_actions.append(TimerAction(period=5.0, actions=[vggt_slam_node]))
    if foxglove:
        full_foxglove_topics = [
            '/odom', '/scan', '/map', '/map_metadata', '/pose',
            '/scan_raw', '/scan_slam', '/xlerobot/scan',
            '/tf', '/tf_static', '/clock',
            '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_teleop', '/cmd_teleop',
            '/goal_pose', '/initialpose', '/clicked_point',
            '/nav/destination', '/nav/goal_pose2d',
            '/camera/image_raw', '/camera/image_raw/compressed', '/camera/camera_info',
            '/xlerobot/base_camera/image/compressed', '/xlerobot/base_camera/camera_info',
            '/xlerobot/wrist_left_camera/image/compressed', '/xlerobot/wrist_left_camera/camera_info',
            '/xlerobot/wrist_right_camera/image/compressed', '/xlerobot/wrist_right_camera/camera_info',
            '/depth/image_raw', '/depth/camera_info', '/depth/points',
            '/imu/data',
            '/rtabmap/cloud_map', '/rtabmap/cloud_ground', '/rtabmap/cloud_obstacles',
            '/local_costmap/costmap', '/global_costmap/costmap',
            '/plan', '/plan_smoothed', '/local_plan',
            '/semantic_vlm/detections', '/semantic_vlm/markers',
            '/semantic_vlm/image_annotations',
            '/semantic_ocr/detections', '/semantic_ocr/markers',
            '/semantic_ocr/image_annotations',
        ]
        map_foxglove_topics = [
            '/clock',
            '/tf', '/tf_static',
            '/odom', '/scan', '/scan_raw', '/scan_slam', '/xlerobot/scan',
            '/imu/data',
            '/map', '/map_metadata',
            '/camera/image_raw', '/camera/image_raw/compressed', '/camera/camera_info',
            '/xlerobot/base_camera/image/compressed', '/xlerobot/base_camera/camera_info',
            '/xlerobot/wrist_left_camera/image/compressed', '/xlerobot/wrist_left_camera/camera_info',
            '/xlerobot/wrist_right_camera/image/compressed', '/xlerobot/wrist_right_camera/camera_info',
            '/depth/image_raw', '/depth/camera_info', '/depth/points',
            '/local_costmap/costmap', '/global_costmap/costmap',
            '/plan', '/plan_smoothed', '/local_plan',
            '/goal_pose', '/initialpose', '/clicked_point',
            '/nav/destination', '/nav/goal_pose2d',
            '/cmd_vel', '/cmd_vel_mux', '/cmd_vel_nav', '/cmd_vel_teleop', '/cmd_teleop',
        ]
        foxglove_topics = (
            map_foxglove_topics if foxglove_profile == 'map'
            else full_foxglove_topics
        )
        launch_actions.append(Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'use_sim_time': use_sim_time,
                'max_qos_depth': 5,
                'send_buffer_limit': 100_000_000,
                'use_compression': True,
                'topic_whitelist': foxglove_topics,
            }],
            output='screen',
        ))
    elif LaunchConfiguration('use_foxglove').perform(context).lower() == 'true':
        launch_actions.append(LogInfo(msg='[sim_nav] foxglove_bridge not found; continuing without Foxglove'))
    if use_explore:
        launch_actions.append(ExecuteProcess(
            cmd=[
                'bash', '-lc',
                'source /opt/ros/humble/setup.bash && '
                'if [ -f /home/fnhid/lingbot-real-orc/install/explore_lite_msgs/share/explore_lite_msgs/package.bash ]; then '
                'source /home/fnhid/lingbot-real-orc/install/explore_lite_msgs/share/explore_lite_msgs/package.bash; fi && '
                'if [ -f /home/fnhid/lingbot-real-orc/install/explore_lite/share/explore_lite/package.bash ]; then '
                'source /home/fnhid/lingbot-real-orc/install/explore_lite/share/explore_lite/package.bash; fi && '
                'for i in $(seq 1 30); do '
                'STATE=$(ros2 lifecycle get /bt_navigator 2>/dev/null | head -1 || true); '
                'if echo "$STATE" | grep -q "active"; then '
                'echo "[sim_nav] starting explore_lite"; '
                f'exec ros2 launch explore_lite explore.launch.py use_sim_time:={use_sim_time_arg}; '
                'fi; '
                'sleep 2; '
                'done; '
                'echo "[sim_nav] explore_lite start skipped: bt_navigator not active in time"; '
            ],
            output='screen',
        ))
    elif LaunchConfiguration('use_explore').perform(context).lower() == 'true':
        launch_actions.append(LogInfo(msg='[sim_nav] explore_lite not found; continuing without auto exploration'))
    return launch_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('isaac_host', default_value='127.0.0.1',
                              description='ZMQ v1 사용 시 Isaac sim_server 호스트. rosbridge_v2 에서는 사용하지 않음.'),
        DeclareLaunchArgument('isaac_transport', default_value='xlerobot_ros',
                              description='Robot 연결 방식: xlerobot_ros(/xlerobot ROS topics) | rosbridge_v2(legacy alias) | zmq_v1(legacy Isaac sim_server).'),
        DeclareLaunchArgument('isaac_robot_id', default_value='0',
                              description='Isaac fleet 안에서 우리 ROS 스택이 바인딩할 robot index.'),
        DeclareLaunchArgument('isaac_camera_link', default_value='head_tilt',
                              description='ZMQ v1 tf.links.<id> 안에서 카메라가 mount된 링크 이름.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use ROS /clock. XLeRobot rosbridge topics use wall time by default.'),
        DeclareLaunchArgument('ros_localhost_only', default_value='1',
                              description='Set ROS_LOCALHOST_ONLY. Default hardware path uses rosbridge, so 1 is fine. Use 0 only for DDS robot I/O.'),
        DeclareLaunchArgument('use_foxglove', default_value='true', description='Foxglove 브리지'),
        DeclareLaunchArgument('foxglove_profile', default_value='full',
                              description='Foxglove topic profile: full|map'),
        DeclareLaunchArgument('use_da3', default_value='false', description='DA3 RGB depth wrapper'),
        DeclareLaunchArgument('use_nvblox', default_value='false', description='nvblox 3D mapping 노드'),
        DeclareLaunchArgument('use_vggt_slam', default_value='false', description='VGGT-SLAM 브리지'),
        DeclareLaunchArgument('use_semantic_vlm', default_value='false', description='RGB-D 기반 semantic VLM 노드'),
        DeclareLaunchArgument('use_semantic_ocr', default_value='true', description='RGB 기반 semantic OCR 노드'),
        DeclareLaunchArgument('use_explore', default_value='false', description='Legacy frontier exploration 자동 시작'),
        DeclareLaunchArgument('vlm_task_mode', default_value='scene_description',
                              description='semantic VLM mode: scene_description|text_objects'),
        DeclareLaunchArgument('vlm_model', default_value='Qwen/Qwen2.5-VL-3B-Instruct',
                              description='VLM model id'),
        DeclareLaunchArgument('vlm_device', default_value='auto',
                              description='VLM device: auto|cuda|cpu'),
        DeclareLaunchArgument('vlm_frame_interval', default_value='20',
                              description='Run one VLM inference every N RGB frames'),
        DeclareLaunchArgument('vlm_max_new_tokens', default_value='256',
                              description='Maximum VLM output tokens'),
        DeclareLaunchArgument('ocr_backend', default_value='paddle',
                              description='OCR backend: paddle|tesseract'),
        DeclareLaunchArgument('ocr_use_gpu', default_value='false',
                              description='Use PaddleOCR GPU runtime when available'),
        DeclareLaunchArgument('ocr_frame_interval', default_value='5',
                              description='Run one OCR inference every N RGB frames'),
        DeclareLaunchArgument('ocr_max_queue_size', default_value='32',
                              description='Maximum queued OCR RGB samples'),
        DeclareLaunchArgument('ocr_max_side', default_value='1280',
                              description='Resize RGB OCR input so the longest side is at most this value'),
        DeclareLaunchArgument('ocr_scales', default_value='1.0,2.0',
                              description='Comma-separated multi-scale OCR factors'),
        DeclareLaunchArgument('ocr_min_confidence', default_value='0.6',
                              description='Discard OCR detections with confidence <= this threshold'),
        DeclareLaunchArgument('ocr_floor_hint', default_value='',
                              description='Optional floor prior, e.g. 4F|13F|B3F'),
        DeclareLaunchArgument('ocr_floor_prior_mode', default_value='reject',
                              description='Floor prior mode: reject|complete'),
        DeclareLaunchArgument('ocr_track_max_gap_frames', default_value='60',
                              description='Maximum RGB frame gap for same-sign OCR track association'),
        DeclareLaunchArgument('ocr_track_max_depth_diff_m', default_value='0.0',
                              description='Reject same-sign associations above this depth gap; 0 disables'),
        DeclareLaunchArgument('use_slam_toolbox', default_value='true',
                              description='2D LiDAR slam_toolbox 활성화. use_rtabmap=true 면 false 로 둘 것.'),
        DeclareLaunchArgument('use_rtabmap', default_value='false',
                              description='RTAB-Map RGB-D SLAM. slam_toolbox 와 배타.'),
        DeclareLaunchArgument('rtabmap_odom_source', default_value='fusion',
                              description='RTAB-Map odometry source: fusion(RGB-D visual odom + LiDAR scan grid) | rgbd(depth sensor RGB-D+IMU) | icp(LiDAR scan ICP).'),
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='Bridge depth sensor IMU to /imu/data and let RGB-D SLAM consume it when enabled.'),
        DeclareLaunchArgument('use_lidar_odom', default_value='false',
                              description='Use scan-to-scan LiDAR odometry as /odom instead of /xlerobot/odom.'),
        DeclareLaunchArgument('enable_base_odom_bridge', default_value='true',
                              description='Bridge /xlerobot/odom to /odom. Disable for hardware runs that forbid base/wheel odom.'),
        DeclareLaunchArgument('use_nav_goal_bridge', default_value='true',
                              description='Enable /nav/destination and /nav/goal_pose2d to /goal_pose bridge.'),
        DeclareLaunchArgument('nav_destinations_file', default_value='',
                              description='YAML/JSON named destination file. Empty uses gz_nav_sim/config/nav_destinations.yaml.'),
        DeclareLaunchArgument('nav_destination_topic', default_value='/nav/destination',
                              description='std_msgs/String named destination input topic.'),
        DeclareLaunchArgument('nav_goal_pose2d_topic', default_value='/nav/goal_pose2d',
                              description='geometry_msgs/Pose2D direct goal input topic.'),
        DeclareLaunchArgument('nav_goal_from_clicked_point', default_value='false',
                              description='If true, convert /clicked_point to /goal_pose. False keeps Foxglove clicks non-driving.'),
        DeclareLaunchArgument('nav_clicked_point_yaw', default_value='0.0',
                              description='Yaw applied when nav_goal_from_clicked_point=true.'),
        DeclareLaunchArgument('use_nav_scan_filter', default_value='false',
                              description='Publish filtered /scan for Nav2 obstacle avoidance; raw input moves to /scan_raw.'),
        DeclareLaunchArgument('nav_scan_filter_min_range', default_value='0.20',
                              description='Drop Nav2 scan returns nearer than this many meters.'),
        DeclareLaunchArgument('nav_scan_filter_max_range', default_value='0.0',
                              description='Drop Nav2 scan returns farther than this many meters; 0 uses scan range_max.'),
        DeclareLaunchArgument('nav_scan_filter_remove_isolated_clusters', default_value='false',
                              description='Remove tiny near clusters from Nav2 scan. Keep false so people remain obstacles.'),
        DeclareLaunchArgument('nav_scan_filter_min_cluster_points', default_value='2',
                              description='Minimum contiguous points kept as a Nav2 obstacle cluster when cluster filtering is enabled.'),
        DeclareLaunchArgument('nav_scan_filter_cluster_jump_m', default_value='0.30',
                              description='Range jump threshold for Nav2 scan cluster splitting.'),
        DeclareLaunchArgument('nav_scan_filter_cluster_max_range', default_value='2.5',
                              description='Only remove tiny Nav2 clusters nearer than this range.'),
        DeclareLaunchArgument('use_slam_scan_filter', default_value='false',
                              description='Publish /scan_slam with close/dynamic-ish returns removed for SLAM input.'),
        DeclareLaunchArgument('slam_scan_filter_min_range', default_value='0.20',
                              description='Drop SLAM scan returns nearer than this many meters.'),
        DeclareLaunchArgument('slam_scan_filter_max_range', default_value='0.0',
                              description='Drop SLAM scan returns farther than this many meters; 0 uses scan range_max.'),
        DeclareLaunchArgument('slam_scan_filter_remove_isolated_clusters', default_value='true',
                              description='Remove tiny near clusters from the SLAM scan. Raw /scan still feeds Nav2.'),
        DeclareLaunchArgument('slam_scan_filter_min_cluster_points', default_value='3',
                              description='Minimum contiguous scan points kept as a stable SLAM cluster.'),
        DeclareLaunchArgument('slam_scan_filter_cluster_jump_m', default_value='0.30',
                              description='Range jump threshold for SLAM scan cluster splitting.'),
        DeclareLaunchArgument('slam_scan_filter_cluster_max_range', default_value='2.5',
                              description='Only remove tiny clusters nearer than this range.'),
        DeclareLaunchArgument('xlerobot_scan_frame', default_value='laser',
                              description='Frame id for /xlerobot/scan after topic normalization.'),
        DeclareLaunchArgument('robot_lidar_x', default_value='0.200',
                              description='Static TF base_link -> xlerobot_scan_frame x offset.'),
        DeclareLaunchArgument('robot_lidar_y', default_value='0.0',
                              description='Static TF base_link -> xlerobot_scan_frame y offset.'),
        DeclareLaunchArgument('robot_lidar_z', default_value='0.730',
                              description='Static TF base_link -> xlerobot_scan_frame z offset.'),
        DeclareLaunchArgument('robot_lidar_roll', default_value='0.0',
                              description='Static TF base_link -> xlerobot_scan_frame roll.'),
        DeclareLaunchArgument('robot_lidar_pitch', default_value='0.0',
                              description='Static TF base_link -> xlerobot_scan_frame pitch.'),
        DeclareLaunchArgument('robot_lidar_yaw', default_value='0.0',
                              description='Static TF base_link -> xlerobot_scan_frame yaw.'),
        DeclareLaunchArgument('robot_camera_x', default_value='0.0',
                              description='Calibration trim from XLeRobot head_camera_link -> camera_frame x.'),
        DeclareLaunchArgument('robot_camera_y', default_value='0.020',
                              description='Calibration trim from XLeRobot head_camera_link -> camera_frame y.'),
        DeclareLaunchArgument('robot_camera_z', default_value='0.0',
                              description='Calibration trim from XLeRobot head_camera_link -> camera_frame z.'),
        DeclareLaunchArgument('robot_camera_roll', default_value='0.0',
                              description='Calibration trim from XLeRobot head_camera_link -> camera_frame roll.'),
        DeclareLaunchArgument('robot_camera_pitch', default_value='0.0',
                              description='Calibration trim from XLeRobot head_camera_link -> camera_frame pitch.'),
        DeclareLaunchArgument('robot_camera_yaw', default_value='0.0',
                              description='Calibration trim from XLeRobot head_camera_link -> camera_frame yaw.'),
        DeclareLaunchArgument('cmd_max_linear_x', default_value='0.30',
                              description='Final /xlerobot/cmd_vel linear.x safety clamp in m/s.'),
        DeclareLaunchArgument('cmd_max_linear_y', default_value='0.30',
                              description='Final /xlerobot/cmd_vel linear.y safety clamp in m/s.'),
        DeclareLaunchArgument('cmd_max_angular_z', default_value='1.00',
                              description='Final /xlerobot/cmd_vel angular.z safety clamp in rad/s.'),
        DeclareLaunchArgument('lidar_odom_max_range', default_value='8.0',
                              description='Maximum range used by scan-to-scan LiDAR odometry.'),
        DeclareLaunchArgument('lidar_odom_max_points', default_value='240',
                              description='Maximum scan points used by Python ICP odometry.'),
        DeclareLaunchArgument('lidar_odom_icp_iterations', default_value='8',
                              description='ICP iterations for scan-to-scan LiDAR odometry.'),
        DeclareLaunchArgument('lidar_odom_max_correspondence_distance', default_value='0.35',
                              description='ICP correspondence gate in meters.'),
        DeclareLaunchArgument('lidar_odom_min_pairs', default_value='35',
                              description='Minimum valid ICP correspondences.'),
        DeclareLaunchArgument('lidar_odom_max_translation_per_scan', default_value='0.35',
                              description='Reject larger per-scan LiDAR odom translations.'),
        DeclareLaunchArgument('lidar_odom_max_rotation_per_scan', default_value='0.60',
                              description='Reject larger per-scan LiDAR odom rotations.'),
        DeclareLaunchArgument('lidar_odom_invert_delta', default_value='false',
                              description='Invert estimated scan-matching delta if the LiDAR odom direction is reversed.'),
        DeclareLaunchArgument('rtabmap_localization', default_value='false',
                              description='True면 Mem/IncrementalMemory=false.'),
        DeclareLaunchArgument('rtabmap_db', default_value='',
                              description='RTAB-Map .db 파일 경로 override.'),
        DeclareLaunchArgument('direct_depth', default_value='true',
                              description='True면 depth sensor native depth를 /depth/*로 publish/use.'),
        DeclareLaunchArgument('use_binary_rgbd_bridge', default_value='false',
                              description='Receive depth sensor RGB-D over a Pi TCP binary side channel instead of rosbridge JSON image topics.'),
        DeclareLaunchArgument('use_rtsp_camera_bridge', default_value='false',
                              description='Republish RTSP camera previews as ROS 2 CompressedImage topics.'),
        DeclareLaunchArgument('rtsp_camera_names', default_value='base,wrist_left,wrist_right',
                              description='Comma-separated RTSP camera preview names.'),
        DeclareLaunchArgument('rtsp_camera_publish_rate_hz', default_value='15.0',
                              description='Foxglove preview publish rate for RTSP cameras.'),
        DeclareLaunchArgument('rtsp_camera_jpeg_quality', default_value='80',
                              description='JPEG quality for RTSP CompressedImage previews.'),
        DeclareLaunchArgument('rtsp_base_camera_url', default_value='rtsp://127.0.0.1:8554/xlerobot_base',
                              description='Base camera RTSP source.'),
        DeclareLaunchArgument('rtsp_wrist_left_camera_url', default_value='rtsp://127.0.0.1:8554/xlerobot_wrist_left',
                              description='Left wrist camera RTSP source.'),
        DeclareLaunchArgument('rtsp_wrist_right_camera_url', default_value='rtsp://127.0.0.1:8554/xlerobot_wrist_right',
                              description='Right wrist camera RTSP source.'),
        DeclareLaunchArgument('binary_rgbd_host', default_value='0.0.0.0',
                              description='Listen host for Pi binary RGB-D transport.'),
        DeclareLaunchArgument('binary_rgbd_port', default_value='9102',
                              description='Listen TCP port for Pi binary RGB-D transport.'),
        DeclareLaunchArgument('use_depth_scan_fallback', default_value='false',
                              description='Convert depth sensor depth to /scan_raw when LiDAR/RTAB are unavailable.'),
        DeclareLaunchArgument('depth_scan_publish_rate_hz', default_value='15.0',
                              description='Maximum depth sensor depth-to-LaserScan publish rate.'),
        DeclareLaunchArgument('depth_sensor_pointcloud_stride', default_value='4',
                              description='Downsample stride for /depth/image_raw -> /depth/points.'),
        DeclareLaunchArgument('depth_sensor_pointcloud_min_depth', default_value='0.20',
                              description='Minimum depth sensor depth in meters used for /depth/points.'),
        DeclareLaunchArgument('depth_sensor_pointcloud_max_depth', default_value='4.50',
                              description='Maximum depth sensor depth in meters used for /depth/points.'),
        DeclareLaunchArgument('depth_sensor_pointcloud_max_points', default_value='12000',
                              description='Maximum PointCloud2 points published per depth sensor depth frame.'),
        DeclareLaunchArgument('use_hardware_lidar', default_value='false',
                              description='Use USB RPLIDAR C-series as /scan instead of /xlerobot/scan.'),
        DeclareLaunchArgument('hardware_lidar_serial', default_value='/dev/ttyUSB0',
                              description='RPLIDAR serial device.'),
        DeclareLaunchArgument('hardware_lidar_baud', default_value='460800',
                              description='RPLIDAR serial baud rate.'),
        DeclareLaunchArgument('hardware_lidar_frame', default_value='laser',
                              description='LaserScan frame_id for hardware lidar.'),
        DeclareLaunchArgument('hardware_lidar_samples', default_value='720',
                              description='Number of LaserScan bins per rotation.'),
        DeclareLaunchArgument('hardware_lidar_angle_offset_deg', default_value='0.0',
                              description='Hardware lidar yaw offset in degrees.'),
        DeclareLaunchArgument('hardware_lidar_invert', default_value='false',
                              description='Invert hardware lidar scan angle direction.'),
        DeclareLaunchArgument('hardware_lidar_range_min', default_value='0.12',
                              description='Hardware lidar minimum valid range in meters.'),
        DeclareLaunchArgument('hardware_lidar_range_max', default_value='12.0',
                              description='Hardware lidar maximum valid range in meters.'),
        DeclareLaunchArgument('hardware_lidar_min_quality', default_value='0',
                              description='Drop RPLIDAR samples below this quality.'),
        DeclareLaunchArgument('da3_model_id', default_value='',
                              description='Optional DA3 model override; empty uses YAML'),
        DeclareLaunchArgument('da3_process_res', default_value='',
                              description='Optional DA3 processing resolution override'),
        DeclareLaunchArgument('da3_process_res_method', default_value='',
                              description='Optional DA3 resize method override'),
        DeclareLaunchArgument('da3_inference_rate_hz', default_value='',
                              description='Optional DA3 inference rate override'),
        DeclareLaunchArgument('da3_input_views', default_value='',
                              description='Optional DA3 input view count override'),
        DeclareLaunchArgument('da3_point_cloud_stride', default_value='',
                              description='Optional point cloud downsample stride override'),
        DeclareLaunchArgument('da3_point_cloud_frame', default_value='',
                              description='Optional target TF frame override for published point cloud'),
        DeclareLaunchArgument('vggt_slam_server_python', default_value='',
                              description='Python 3.11 interpreter for VGGT-SLAM server'),
        DeclareLaunchArgument('vggt_slam_server_script', default_value='',
                              description='Path to vggt_slam_server.py'),
        DeclareLaunchArgument('vggt_slam_submap_size', default_value='',
                              description='Frames per submap override'),
        DeclareLaunchArgument('vggt_slam_min_disparity', default_value='',
                              description='Keyframe disparity threshold override'),
        DeclareLaunchArgument('vggt_slam_pointcloud_stride', default_value='',
                              description='Published pointcloud downsample stride'),
        DeclareLaunchArgument('vggt_slam_image_topic', default_value='',
                              description='Compressed image topic for VGGT-SLAM'),
        OpaqueFunction(function=_launch),
    ])
