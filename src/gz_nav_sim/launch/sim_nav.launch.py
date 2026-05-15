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
    isaac_transport = LaunchConfiguration('isaac_transport').perform(context).strip().lower() or 'rosbridge_v2'
    if isaac_transport not in ('rosbridge_v2', 'zmq_v1'):
        raise RuntimeError(
            "isaac_transport must be 'rosbridge_v2' or 'zmq_v1', "
            f"got: {isaac_transport}")

    foxglove = LaunchConfiguration('use_foxglove').perform(context).lower() == 'true'
    use_da3 = LaunchConfiguration('use_da3').perform(context).lower() == 'true'
    use_nvblox = LaunchConfiguration('use_nvblox').perform(context).lower() == 'true'
    use_vggt_slam = LaunchConfiguration('use_vggt_slam').perform(context).lower() == 'true'
    use_semantic_vlm = LaunchConfiguration('use_semantic_vlm').perform(context).lower() == 'true'
    use_semantic_ocr = LaunchConfiguration('use_semantic_ocr').perform(context).lower() == 'true'
    use_slam_toolbox = LaunchConfiguration('use_slam_toolbox').perform(context).lower() == 'true'
    use_rtabmap = LaunchConfiguration('use_rtabmap').perform(context).lower() == 'true'
    rtabmap_localization = LaunchConfiguration('rtabmap_localization').perform(context).lower() == 'true'
    rtabmap_db = LaunchConfiguration('rtabmap_db').perform(context).strip()
    use_explore = LaunchConfiguration('use_explore').perform(context).lower() == 'true'
    direct_depth = LaunchConfiguration('direct_depth').perform(context).lower() == 'true'

    nvblox_available = _package_available('nvblox_ros')
    foxglove_available = _package_available('foxglove_bridge')
    explore_available = _package_available('explore_lite')
    if use_nvblox and not nvblox_available:
        use_nvblox = False
    if foxglove and not foxglove_available:
        foxglove = False
    if use_explore and not explore_available:
        use_explore = False
    if use_rtabmap and use_slam_toolbox:
        raise RuntimeError(
            'use_rtabmap=true와 use_slam_toolbox=true는 동시 사용 불가 — '
            '한 백엔드만 map→odom TF를 발행해야 함.')

    workspace_root = os.path.abspath(os.path.join(pkg, '..', '..', '..', '..'))
    da3_repo = os.path.join(workspace_root, 'src', 'Depth-Anything-3')
    vggt_slam_repo = os.path.join(workspace_root, 'src', 'VGGT-SLAM')

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

    front_camera_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_tf',
        arguments=[
            '--x', '0.16', '--y', '0', '--z', '0.80',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'camera_frame',
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

    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')
    da3_params = os.path.join(pkg, 'config', 'da3_params.yaml')
    if direct_depth:
        vlm_depth_topic = '/d456/depth/image_raw'
        vlm_camera_info_topic = '/d456/depth/camera_info'
    else:
        vlm_depth_topic = '/camera/depth/image_raw'
        vlm_camera_info_topic = '/camera/camera_info'

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params, {
            'use_sim_time': True,
            'use_lifecycle_manager': False,
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

    nav2_params = os.path.join(pkg, 'config', 'nav2_params_d456.yaml')
    if not os.path.exists(nav2_params):
        nav2_params = os.path.join(nav2_pkg, 'params', 'nav2_params.yaml')

    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': nav2_params,
            'use_composition': 'False',
            'container_name': 'nav2_container',
        }.items(),
    )

    da3_overrides = {
        'use_sim_time': True,
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
            'use_sim_time': True,
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
            'use_sim_time': True,
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
            'input_topic': '/camera/points',
            'output_topic': '/camera/points_visual',
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
            'use_sim_time': True,
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
        'use_sim_time': True,
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
        'use_sim_time': True,
        'database_path': rtabmap_db_path,
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_rgbd': False,
        'subscribe_scan': True,
        'approx_sync': True,
        'approx_sync_max_interval': 3.0,
        'queue_size': 100,
        'sync_queue_size': 100,
        'topic_queue_size': 100,
        'wait_for_transform': 2.0,
        'qos_image': 2,
        'qos_camera_info': 1,
        'qos_scan': 1,
        'qos_odom': 1,
        'Mem/IncrementalMemory': 'false' if rtabmap_localization else 'true',
        'Mem/InitWMWithAllNodes': 'true',
        'Rtabmap/StartNewMapOnLoopClosure': 'false',
        'Rtabmap/DetectionRate': '1.0',
        'Rtabmap/CreateIntermediateNodes': 'true',
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Icp/PointToPlane': 'false',
        'Icp/MaxCorrespondenceDistance': '0.1',
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
        'Kp/MaxFeatures': '0',
        'Mem/MemoryThr': '5000',
        'Mem/ReduceGraph': 'true',
        'Rtabmap/MaxRetrieved': '2',
        'Grid/Sensor': '0',
        'Grid/RangeMax': '10.0',
        'Grid/CellSize': '0.05',
        'Grid/3D': 'false',
        'Grid/RayTracing': 'true',
        'Grid/FootprintHeight': '0.4',
        'Grid/MaxObstacleHeight': '1.5',
        'Grid/NormalsSegmentation': 'false',
    }

    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': False,
            'subscribe_scan': True,
            'subscribe_scan_cloud': False,
            'approx_sync': True,
            'queue_size': 30,
            'sync_queue_size': 30,
            'topic_queue_size': 30,
            'wait_for_transform': 0.2,
            'qos_scan': 1,
            'Reg/Force3DoF': 'true',
            'Reg/Strategy': '1',
            'Icp/PointToPlane': 'false',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/Iterations': '10',
            'Icp/Epsilon': '0.001',
            'Icp/VoxelSize': '0.05',
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/rtabmap/icp_odom'),
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
            ('scan', '/scan'),
            ('odom', '/rtabmap/icp_odom'),
            ('map', '/map'),
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/d456/depth/image_raw'),
        ],
        arguments=['--ros-args', '--log-level', 'rtabmap:=info'],
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'twist_mux.yaml')],
        remappings=[
            ('cmd_vel_out', '/cmd_vel_mux'),
        ],
    )

    if direct_depth:
        nvblox_depth_topic = '/d456/depth/image_raw'
        nvblox_depth_info_topic = '/d456/depth/camera_info'
    else:
        nvblox_depth_topic = '/camera/depth/image_raw'
        nvblox_depth_info_topic = '/camera/depth/camera_info'

    nvblox_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'nvblox.launch.py')),
        launch_arguments={
            'depth_topic': nvblox_depth_topic,
            'depth_info_topic': nvblox_depth_info_topic,
            'color_topic': '/camera/image_raw',
            'color_info_topic': '/camera/camera_info',
        }.items(),
    )

    nvblox_gltf_node = Node(
        package='gz_nav_sim',
        executable='nvblox_mesh_to_gltf.py',
        name='nvblox_mesh_to_gltf',
        output='screen',
        parameters=[{
            'use_sim_time': True,
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
            'cmd_rate_hz': 20.0,
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
            'cmd_rate_hz': 20.0,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'camera_optical_frame': 'camera_optical_frame',
            'scan_frame': 'base_link',
            'cmd_vel_in_topic': '/cmd_vel_mux',
            'cmd_vel_out_topic': '/xlerobot/cmd_vel',
            'odom_in_topic': '/xlerobot/odom',
            'odom_out_topic': '/odom',
            'rgb_image_in_topic': '/xlerobot/head/d456/color/image_raw',
            'rgb_compressed_image_in_topic': '/xlerobot/head/d456/color/image',
            'rgb_info_in_topic': '/xlerobot/head/d456/color/camera_info',
            'rgb_image_out_topic': '/camera/image_raw',
            'rgb_compressed_out_topic': '/camera/image_raw/compressed',
            'rgb_info_out_topic': '/camera/camera_info',
            'depth_image_in_topic': '/xlerobot/head/d456/depth/image_rect_raw',
            'depth_compressed_image_in_topic': '/xlerobot/head/d456/depth/image',
            'depth_info_in_topic': '/xlerobot/head/d456/depth/camera_info',
            'depth_image_out_topic': '/d456/depth/image_raw',
            'depth_info_out_topic': '/d456/depth/camera_info',
            'scan_in_topic': '/xlerobot/scan',
            'scan_out_topic': '/scan',
        }],
    )

    launch_actions = [
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
    ]
    if isaac_transport == 'zmq_v1':
        launch_actions.append(LogInfo(
            msg=f'[sim_nav] Isaac backend: ZMQ v1 bridge to tcp://{isaac_host}:5555/5556/5557'))
        launch_actions.append(isaac_zmq_bridge_node)
    else:
        launch_actions.append(LogInfo(
            msg='[sim_nav] Isaac backend: XLeRobot v2 ROS topics under /xlerobot'))
        launch_actions.append(xlerobot_v2_bridge_node)

    launch_actions.extend([
        base_footprint_tf,
        front_camera_frame_tf,
        front_camera_optical_tf,
        pointcloud_visualizer_node,
        trajectory_path_node,
        twist_mux_node,
    ])
    if use_slam_toolbox:
        launch_actions.extend([slam, slam_configure, slam_activate])
    if use_rtabmap:
        launch_actions.append(TimerAction(period=4.0, actions=[icp_odometry_node]))
        launch_actions.append(TimerAction(period=6.0, actions=[rtabmap_node]))
    launch_actions.extend([nav2_container, navigation])
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
        launch_actions.append(Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'use_sim_time': True,
                'max_qos_depth': 5,
                'send_buffer_limit': 100_000_000,
                'use_compression': True,
                'topic_whitelist': [
                    '/odom', '/scan', '/map', '/map_metadata', '/pose',
                    '/tf', '/tf_static', '/clock',
                    '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_teleop',
                    '/camera/image_raw/compressed', '/camera/camera_info',
                    '/rtabmap/cloud_map', '/rtabmap/cloud_ground', '/rtabmap/cloud_obstacles',
                    '/local_costmap/costmap', '/global_costmap/costmap',
                    '/plan', '/plan_smoothed', '/local_plan',
                    '/semantic_vlm/detections', '/semantic_vlm/markers',
                    '/semantic_vlm/image_annotations',
                    '/semantic_ocr/detections', '/semantic_ocr/markers',
                    '/semantic_ocr/image_annotations',
                ],
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
                'exec ros2 launch explore_lite explore.launch.py use_sim_time:=true; '
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
        DeclareLaunchArgument('isaac_transport', default_value='rosbridge_v2',
                              description='Isaac 연결 방식: rosbridge_v2(/xlerobot ROS topics) | zmq_v1(legacy sim_server).'),
        DeclareLaunchArgument('isaac_robot_id', default_value='0',
                              description='Isaac fleet 안에서 우리 ROS 스택이 바인딩할 robot index.'),
        DeclareLaunchArgument('isaac_camera_link', default_value='head_tilt',
                              description='ZMQ v1 tf.links.<id> 안에서 카메라가 mount된 링크 이름.'),
        DeclareLaunchArgument('use_foxglove', default_value='true', description='Foxglove 브리지'),
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
        DeclareLaunchArgument('rtabmap_localization', default_value='false',
                              description='True면 Mem/IncrementalMemory=false.'),
        DeclareLaunchArgument('rtabmap_db', default_value='',
                              description='RTAB-Map .db 파일 경로 override.'),
        DeclareLaunchArgument('direct_depth', default_value='true',
                              description='True면 D456 native depth(/d456/depth/*)를 사용.'),
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
