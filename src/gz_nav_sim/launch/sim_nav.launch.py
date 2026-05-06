"""Gazebo Classic 11 + Nav2 + SLAM Toolbox simulation (ROS 2 Humble).

  ros2 launch gz_nav_sim sim_nav.launch.py
  ros2 launch gz_nav_sim sim_nav.launch.py headless:=true use_foxglove:=true

SLAM이 자동으로 map→odom TF와 /map 토픽을 제공합니다. 초기 포즈 별도 설정 불필요.
월드: combined.world — office (origin) + hospital (+150m on X) 머지 맵.
로봇 스폰 위치: office 내부 (-3, 0), 엘리베이터 방향 바라봄.
엘리베이터 이동: /elevator/call 에 std_msgs/Empty publish (현재 존에 따라 반대 빌딩으로 텔레포트).
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def _join_paths(*groups: str) -> str:
    paths: list[str] = []
    for group in groups:
        for p in (group or '').split(os.pathsep):
            if p and p not in paths:
                paths.append(p)
    return os.pathsep.join(paths)


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
    pkg      = get_package_share_directory('gz_nav_sim')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    foxglove = LaunchConfiguration('use_foxglove').perform(context).lower() == 'true'
    use_da3  = LaunchConfiguration('use_da3').perform(context).lower() == 'true'
    use_nvblox = LaunchConfiguration('use_nvblox').perform(context).lower() == 'true'
    use_vggt_slam = LaunchConfiguration('use_vggt_slam').perform(context).lower() == 'true'
    use_semantic_vlm = LaunchConfiguration('use_semantic_vlm').perform(context).lower() == 'true'
    use_slam_toolbox = LaunchConfiguration('use_slam_toolbox').perform(context).lower() == 'true'
    use_rtabmap = LaunchConfiguration('use_rtabmap').perform(context).lower() == 'true'
    rtabmap_localization = LaunchConfiguration('rtabmap_localization').perform(context).lower() == 'true'
    rtabmap_db = LaunchConfiguration('rtabmap_db').perform(context).strip()
    use_elevator = LaunchConfiguration('use_elevator').perform(context).lower() == 'true'
    use_explore = LaunchConfiguration('use_explore').perform(context).lower() == 'true'
    robot_model = LaunchConfiguration('robot_model').perform(context).strip() or 'robot'
    direct_depth = LaunchConfiguration('direct_depth').perform(context).lower() == 'true'

    nvblox_available = _package_available('nvblox_ros')
    foxglove_available = _package_available('foxglove_bridge')
    image_transport_available = _package_available('image_transport')
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

    # 카메라 z height (TF용) — robot_model에 따라 달라짐.
    #   'robot'      → 0.50m (mono RGB)
    #   'robot_d456' → 0.80m (Realsense D456 depth+RGB on a mast)
    if robot_model == 'robot_d456':
        camera_z = '0.80'
    else:
        camera_z = '0.5'

    # World file: combined.world 의 model://robot 를 robot_model 로 substitute.
    # 35K-line world를 통째 복사하지 않고 launch 시 한 번 텍스트 치환.
    world_template = os.path.join(pkg, 'worlds', 'combined.world')
    if robot_model == 'robot':
        world = world_template
    else:
        with open(world_template, 'r') as f:
            world_content = f.read()
        world_content = world_content.replace(
            'model://robot</uri>', f'model://{robot_model}</uri>')
        world = f'/tmp/sim_nav_world_{robot_model}.world'
        with open(world, 'w') as f:
            f.write(world_content)

    workspace_root = os.path.abspath(os.path.join(pkg, '..', '..', '..', '..'))
    da3_repo = os.path.join(workspace_root, 'src', 'Depth-Anything-3')
    vggt_slam_repo = os.path.join(workspace_root, 'src', 'VGGT-SLAM')

    # ── Gazebo model / resource paths ────────────────────────────────────────
    # Include Gazebo Classic 11 system paths explicitly — without them the
    # shader libs under /usr/share/gazebo-11 can't be found and camera
    # sensors fail to render ("Unable to create CameraSensor").  Normally
    # `source /usr/share/gazebo/setup.bash` does this; we inline it here so
    # the launch works regardless of the user's shell setup.
    gazebo_system_paths = [
        '/usr/share/gazebo-11',
        '/usr/share/gazebo',
    ]
    model_paths = _join_paths(
        os.path.join(pkg, 'models'),
        os.path.join(pkg, 'models', 'office'),
        os.path.join(pkg, 'models', 'hospital'),
        '/usr/share/gazebo-11/models',
        '/usr/share/gazebo/models',
        os.environ.get('GAZEBO_MODEL_PATH', ''),
    )
    resource_paths = _join_paths(
        pkg,
        os.path.join(pkg, 'models', 'office'),
        *gazebo_system_paths,
        os.environ.get('GAZEBO_RESOURCE_PATH', ''),
    )
    plugin_paths = _join_paths(
        '/opt/ros/humble/lib',
        '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',
        os.environ.get('GAZEBO_PLUGIN_PATH', ''),
    )
    media_paths = _join_paths(
        '/usr/share/gazebo-11/media',
        os.environ.get('GAZEBO_MEDIA_PATH', ''),
    )

    # ── Gazebo Classic 11 (gzserver + gzclient) ──────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world,
            'verbose': 'true',
            'pause': 'false',
        }.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')),
        launch_arguments={'verbose': 'true'}.items(),
    )

    # ── Static TFs for camera frames ─────────────────────────────────────────
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
            '--x', '0.16', '--y', '0', '--z', camera_z,
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

    # ── SLAM Toolbox (online async — fresh map every run) ───────────────────
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

    # ── Nav2 (navigation only — map comes from SLAM) ────────────────────────
    if robot_model == 'robot_d456':
        nav2_params = os.path.join(pkg, 'config', 'nav2_params_d456.yaml')
    else:
        nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
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
            # use_composition=True + LoadComposableNodes의 RewrittenYaml param이
            # component에 전달되지 않는 Humble 버그 → costmap이 hardcoded default 플러그인
            # (static_layer 포함) 로드 → slam의 transient_local map과 race → SIGSEGV.
            # False면 각 서버가 별도 프로세스로 뜨고 --params-file 직접 로드 → yaml 적용 확실.
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

    # ── VGGT-SLAM bridge (Python 3.10 side) ─────────────────────────────────
    # Heavy SLAM solver runs in a separate Python 3.11 venv process,
    # spawned by the bridge via subprocess. See vggt_slam_server.py.
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

    elevator_node = Node(
        package='gz_nav_sim',
        executable='elevator_teleport.py',
        name='elevator_teleport',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_model': robot_model,
            # rtabmap 사용 시 .db swap, 아니면 slam_toolbox 라이프사이클 fresh restart.
            'slam_backend': 'rtabmap' if use_rtabmap else 'slam_toolbox',
        }],
    )

    # ── RTAB-Map (RGB-D SLAM, multi-session via .db) ────────────────────────
    # D456 model: RGB는 /camera/image_raw, depth는 /d456/depth/image_raw 로
    # 분리되어 있으므로 양쪽 다 remap. /scan 은 보조 proximity 입력.
    # `direct_depth` 와 `robot_model` 가 제대로 세팅된 d456 프리셋 가정.
    rtabmap_params = os.path.join(pkg, 'config', 'rtabmap_params.yaml')
    # 빈 문자열이면 RTAB-Map 이 in-memory 모드(저장 안 됨) — 기본 경로로 폴백.
    rtabmap_db_path = rtabmap_db or os.path.expanduser('~/.ros/rtabmap.db')
    rtabmap_overrides = {
        'use_sim_time': True,
        'database_path': rtabmap_db_path,
        'Mem/IncrementalMemory': 'false' if rtabmap_localization else 'true',
    }
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        output='screen',
        parameters=[rtabmap_params, rtabmap_overrides],
        remappings=[
            ('rgb/image',         '/camera/image_raw'),
            ('rgb/camera_info',   '/camera/camera_info'),
            ('depth/image',       '/d456/depth/image_raw'),
            ('scan',              '/scan'),
            ('odom',              '/odom'),
            # Nav2 가 구독하는 /map 으로 grid_map 노출 (rtabmap_ros 기본:
            # /rtabmap/grid_map). Nav2 의 nav2_params.yaml 이 /map 을 기대.
            ('grid_map',          '/map'),
        ],
        # rtabmap 은 시작 시 DB 가 있으면 자동 append, 없으면 새로 생성.
        # `--delete_db_on_start` 인자 미지정 → 멀티세션 자연스럽게 동작.
        arguments=['--ros-args', '--log-level', 'rtabmap:=info'],
    )

    # nvblox depth 입력 분기:
    #   direct_depth=true (D456 native depth → nvblox 직접): /d456/depth/*
    #   direct_depth=false (DA3/VGGT 출력 사용): /camera/depth/*
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

    # nvblox mesh → glTF(Draco) republisher — Foxglove SceneUpdate로 30~50배 압축
    nvblox_gltf_node = Node(
        package='gz_nav_sim',
        executable='nvblox_mesh_to_gltf.py',
        name='nvblox_mesh_to_gltf',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/nvblox_node/mesh',
            'output_topic': '/nvblox_node/scene',
            # nvblox는 메모리 위해 map_clearing_radius_m 안쪽만 유지하지만
            # Foxglove에선 누적된 mesh 영구 보관 (지나간 곳도 계속 보임)
            'accumulate_only': True,
        }],
    )

    launch_actions = [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_paths),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', resource_paths),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_paths),
        SetEnvironmentVariable('GAZEBO_MEDIA_PATH', media_paths),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        # DISPLAY는 xvfb-run이 자식에 inject한 값 그대로 사용 (보통 :100, :101...).
        # 이전에 ':99'로 override 했었는데 xvfb-run -a 자유 display 선택 결과와
        # 충돌 → gzserver "xcb_connection_has_error" → CameraSensor render 불가.
        gzserver,
    ]
    if not headless:
        launch_actions.append(gzclient)
    launch_actions.extend([
        base_footprint_tf,
        front_camera_frame_tf,
        front_camera_optical_tf,
        pointcloud_visualizer_node,
        trajectory_path_node,
    ])
    if use_slam_toolbox:
        launch_actions.extend([slam, slam_configure, slam_activate])
    if use_rtabmap:
        # Gazebo 카메라/라이다가 첫 프레임 publish 한 뒤 띄워야 sync timeout 회피.
        launch_actions.append(TimerAction(period=4.0, actions=[rtabmap_node]))
    launch_actions.extend([
        nav2_container,
        navigation,
    ])
    if use_da3:
        launch_actions.append(da3_node)
    if use_semantic_vlm:
        # Camera/depth publishers need a short warm-up before VLM samples frames.
        launch_actions.append(TimerAction(period=12.0, actions=[semantic_vlm_node]))
    elif LaunchConfiguration('use_semantic_vlm').perform(context).lower() == 'true':
        launch_actions.append(LogInfo(msg='[sim_nav] semantic VLM disabled'))
    if use_nvblox:
        # depth가 뜬 뒤에 nvblox를 띄워야 첫 프레임 누락이 없음
        launch_actions.append(TimerAction(period=8.0, actions=[nvblox_include]))
        # mesh→gltf republisher는 nvblox /mesh 토픽 뜬 뒤
        launch_actions.append(TimerAction(period=10.0, actions=[nvblox_gltf_node]))
    elif LaunchConfiguration('use_nvblox').perform(context).lower() == 'true':
        launch_actions.append(LogInfo(msg='[sim_nav] nvblox_ros not found; continuing without nvblox'))
    if use_vggt_slam:
        # VGGT-SLAM은 서버 프로세스를 spawn해야 해서 가제보 sensor가 뜬 후에 시작
        launch_actions.append(TimerAction(period=5.0, actions=[vggt_slam_node]))
    if use_elevator:
        launch_actions.append(elevator_node)
    if foxglove:
        launch_actions.append(Node(
            package='foxglove_bridge', executable='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'use_sim_time': True,
                'max_qos_depth': 5,
                'send_buffer_limit': 10_000_000,
                'use_compression': True,
                # raw 카메라/depth 는 12 MB/s × 채널 → loopback 폭주.
                # 화이트리스트로 가벼운 토픽만 advertise. raw 보고 싶으면
                # /camera/image_raw 직접 추가하거나 use_compressed_topic 활용.
                'topic_whitelist': [
                    '/odom', '/scan', '/map', '/tf', '/tf_static', '/clock',
                    '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_teleop',
                    '/camera/image_raw/compressed', '/camera/camera_info',
                    '/rtabmap/cloud_map', '/rtabmap/grid_map',
                    '/rtabmap/info', '/rtabmap/mapData',
                    '/local_costmap/costmap', '/global_costmap/costmap',
                    '/plan', '/plan_smoothed', '/local_plan',
                    '/gazebo/model_states', '/gazebo/link_states',
                ],
            }],
            output='screen',
        ))
        if image_transport_available and (use_vggt_slam or robot_model == 'robot'):
            # D456 Gazebo camera already publishes /camera/image_raw/compressed.
            # Keep the extra republisher only for the mono-RGB robot or when
            # VGGT-SLAM explicitly depends on this topic path.
            launch_actions.append(Node(
                package='image_transport', executable='republish',
                name='image_compressor',
                arguments=['raw', 'compressed'],
                remappings=[
                    ('in', '/camera/image_raw'),
                    ('out/compressed', '/camera/image_raw/compressed'),
                ],
                output='log',
            ))
        else:
            launch_actions.append(LogInfo(
                msg='[sim_nav] skipping compressed image republisher '
                    '(D456 already publishes compressed image or image_transport unavailable)'
            ))
    elif LaunchConfiguration('use_foxglove').perform(context).lower() == 'true':
        launch_actions.append(LogInfo(msg='[sim_nav] foxglove_bridge not found; continuing without Foxglove'))
    if use_explore:
        # Legacy frontier exploration stack: wait until Nav2 is active, then start
        # explore_lite against the SLAM /map topic.
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
        DeclareLaunchArgument('headless',    default_value='false',  description='GUI 없이 실행'),
        DeclareLaunchArgument('use_foxglove',default_value='true',  description='Foxglove 브리지'),
        DeclareLaunchArgument('use_da3', default_value='false', description='DA3 RGB depth wrapper'),
        DeclareLaunchArgument('use_nvblox', default_value='false', description='nvblox 3D mapping 노드 (isaac_ros_nvblox 필요)'),
        DeclareLaunchArgument('use_vggt_slam', default_value='false', description='VGGT-SLAM 브리지 (Python 3.11 venv 서버 spawn)'),
        DeclareLaunchArgument('use_semantic_vlm', default_value='true', description='RGB-D 기반 semantic VLM 노드'),
        DeclareLaunchArgument('use_explore', default_value='false', description='Legacy frontier exploration (explore_lite) 자동 시작'),
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
        DeclareLaunchArgument('use_slam_toolbox', default_value='true',
                              description='2D LiDAR slam_toolbox 활성화. use_rtabmap=true 면 false 로 둘 것.'),
        DeclareLaunchArgument('use_rtabmap', default_value='false',
                              description='RTAB-Map RGB-D SLAM (멀티세션, .db 영속). slam_toolbox 와 배타.'),
        DeclareLaunchArgument('rtabmap_localization', default_value='false',
                              description='True면 Mem/IncrementalMemory=false (기존 .db 위 로컬라이제이션 전용 모드).'),
        DeclareLaunchArgument('rtabmap_db', default_value='',
                              description='RTAB-Map .db 파일 경로 override. 비어 있으면 ~/.ros/rtabmap.db 자동.'),
        DeclareLaunchArgument('use_elevator', default_value='true', description='엘리베이터 텔레포트 노드'),
        DeclareLaunchArgument('robot_model', default_value='robot_d456',
                              description='Robot 모델 디렉토리. "robot"(mono RGB 0.5m) | "robot_d456"(D456 depth+RGB 0.8m)'),
        DeclareLaunchArgument('direct_depth', default_value='true',
                              description='True면 D456 native depth(/d456/depth/*)를 nvblox 입력으로 직접 사용. DA3/VGGT 우회.'),
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
                              description='Python 3.11 interpreter for VGGT-SLAM server (empty uses YAML)'),
        DeclareLaunchArgument('vggt_slam_server_script', default_value='',
                              description='Path to vggt_slam_server.py (empty uses YAML)'),
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
