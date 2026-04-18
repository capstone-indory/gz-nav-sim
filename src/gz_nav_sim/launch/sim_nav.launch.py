"""Gazebo Classic 11 + Nav2 + SLAM Toolbox simulation (ROS 2 Humble).

  ros2 launch gz_nav_sim sim_nav.launch.py
  ros2 launch gz_nav_sim sim_nav.launch.py headless:=true use_foxglove:=true

SLAM이 자동으로 map→odom TF와 /map 토픽을 제공합니다. 초기 포즈 별도 설정 불필요.
월드: combined.world — office (origin) + hospital (+150m on X) 머지 맵.
로봇 스폰 위치: office 내부 (-3, 0), 엘리베이터 방향 바라봄.
엘리베이터 이동: /elevator/call 에 std_msgs/Empty publish (현재 존에 따라 반대 빌딩으로 텔레포트).
"""

import os

from ament_index_python.packages import get_package_share_directory
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


def _launch(context, *_args, **_kwargs):
    pkg      = get_package_share_directory('gz_nav_sim')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    foxglove = LaunchConfiguration('use_foxglove').perform(context).lower() == 'true'
    use_da3  = LaunchConfiguration('use_da3').perform(context).lower() == 'true'
    use_nvblox = LaunchConfiguration('use_nvblox').perform(context).lower() == 'true'
    use_vggt_slam = LaunchConfiguration('use_vggt_slam').perform(context).lower() == 'true'
    use_elevator = LaunchConfiguration('use_elevator').perform(context).lower() == 'true'
    world    = os.path.join(pkg, 'worlds', 'combined.world')

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
            '--x', '0.16', '--y', '0', '--z', '0.5',
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
            'use_composition': 'True',
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
        parameters=[{'use_sim_time': True, 'robot_model': 'robot'}],
    )

    nvblox_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'nvblox.launch.py')),
        launch_arguments={
            'depth_topic': '/camera/depth/image_raw',
            'depth_info_topic': '/camera/depth/camera_info',
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
        SetEnvironmentVariable('DISPLAY', ':99'),
        gzserver,
    ]
    if not headless:
        launch_actions.append(gzclient)
    launch_actions.extend([
        base_footprint_tf,
        front_camera_frame_tf,
        front_camera_optical_tf,
        slam,
        slam_configure,
        slam_activate,
        nav2_container,
        navigation,
    ])
    if use_da3:
        launch_actions.append(da3_node)
    if use_nvblox:
        # DA3 depth가 뜬 뒤에 nvblox를 띄워야 첫 프레임 누락이 없음
        launch_actions.append(TimerAction(period=8.0, actions=[nvblox_include]))
        # mesh→gltf republisher는 nvblox /mesh 토픽 뜬 뒤
        launch_actions.append(TimerAction(period=10.0, actions=[nvblox_gltf_node]))
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
            }],
            output='screen',
        ))
    return launch_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless',    default_value='false',  description='GUI 없이 실행'),
        DeclareLaunchArgument('use_foxglove',default_value='true',  description='Foxglove 브리지'),
        DeclareLaunchArgument('use_da3', default_value='true', description='DA3 RGB depth wrapper'),
        DeclareLaunchArgument('use_nvblox', default_value='false', description='nvblox 3D mapping 노드 (isaac_ros_nvblox 필요)'),
        DeclareLaunchArgument('use_vggt_slam', default_value='false', description='VGGT-SLAM 브리지 (Python 3.11 venv 서버 spawn)'),
        DeclareLaunchArgument('use_elevator', default_value='true', description='엘리베이터 텔레포트 노드'),
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
