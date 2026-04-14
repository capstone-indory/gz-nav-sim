"""Gazebo + Nav2 + SLAM Toolbox standalone simulation.

  ros2 launch gz_nav_sim sim_nav.launch.py
  ros2 launch gz_nav_sim sim_nav.launch.py headless:=true use_foxglove:=true

SLAM이 자동으로 map→odom TF와 /map 토픽을 제공합니다. 초기 포즈 별도 설정 불필요.
로봇 스폰 위치: 복도 서단 (-8.5, 0.0)
"""

import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


# ── macOS: conda Gazebo만 쓰도록 환경 정리 ───────────────────────────────────
def _clean_env() -> dict:
    env = os.environ.copy()
    if platform.system() != 'Darwin':
        return env

    path = [entry for entry in env.get('PATH', '').split(os.pathsep)
            if entry and 'rviz_ogre_vendor' not in entry]
    env['PATH'] = os.pathsep.join(path)

    # Gazebo가 자체 런타임을 쓰도록 Qt / DYLD override는 비운다.
    for key in ['DYLD_LIBRARY_PATH', 'DYLD_FALLBACK_LIBRARY_PATH', 'DYLD_FRAMEWORK_PATH',
                'QT_PLUGIN_PATH', 'QT_QPA_PLATFORM_PLUGIN_PATH']:
        env.pop(key, None)
    return env


def _gz_bin() -> str:
    return os.path.join(os.environ['CONDA_PREFIX'], 'bin', 'gz')


def _join_resource_paths(*path_groups: str) -> str:
    paths = []
    for group in path_groups:
        for path in group.split(os.pathsep):
            if path and path not in paths:
                paths.append(path)
    return os.pathsep.join(paths)


def _optional_launch_arg(context, name: str, cast=None):
    value = LaunchConfiguration(name).perform(context).strip()
    if value == '':
        return None
    return cast(value) if cast is not None else value


# ── Launch ────────────��─────────────────────────��──────────────────────────────
def _launch(context, *_args, **_kwargs):
    pkg      = get_package_share_directory('gz_nav_sim')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    gz       = _gz_bin()
    is_macos = platform.system() == 'Darwin'
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    foxglove = LaunchConfiguration('use_foxglove').perform(context).lower() == 'true'
    use_da3  = LaunchConfiguration('use_da3').perform(context).lower() == 'true'
    world    = os.path.join(pkg, 'worlds', 'sim.sdf')
    env      = _clean_env()
    partition = f'gz_nav_sim_{os.getpid()}'
    workspace_root = os.path.abspath(os.path.join(pkg, '..', '..', '..', '..'))
    da3_repo = os.path.join(workspace_root, 'src', 'Depth-Anything-3')
    resource_paths = os.pathsep.join([
        os.path.join(pkg, 'worlds'),
        os.path.join(pkg, 'models'),
    ])
    env['GZ_PARTITION'] = partition
    env['IGN_PARTITION'] = partition
    env['GZ_SIM_RESOURCE_PATH'] = _join_resource_paths(
        resource_paths,
        env.get('GZ_SIM_RESOURCE_PATH', ''),
    )
    env['IGN_GAZEBO_RESOURCE_PATH'] = _join_resource_paths(
        resource_paths,
        env.get('IGN_GAZEBO_RESOURCE_PATH', ''),
    )

    # ── Gazebo ─────────────��──────────────────────────────────────────────────
    if is_macos and not headless:
        # macOS: server(OpenGL) + GUI(Metal) 분리
        gz_procs = [
            ExecuteProcess(cmd=[gz, 'sim', '-s', '-r', '-v', '4',
                                '--render-engine-server', 'ogre2',
                                '--render-engine-server-api-backend', 'opengl',
                                world], env=env, output='screen'),
            ExecuteProcess(cmd=[gz, 'sim', '-g', '-v', '4',
                                '--render-engine-gui', 'ogre2',
                                '--render-engine-gui-api-backend', 'metal'],
                           env=env, output='screen'),
        ]
    elif headless:
        extra = ['-s', '--render-engine-server', 'ogre2',
                 '--render-engine-server-api-backend', 'opengl'] if is_macos \
                else ['-s', '--headless-rendering']
        gz_procs = [ExecuteProcess(cmd=[gz, 'sim', '-r', '-v', '4', *extra, world],
                                   env=env, output='screen')]
    else:
        gz_procs = [ExecuteProcess(cmd=[gz, 'sim', '-r', '-v', '4', world],
                                   env=env, output='screen')]

    # ── Bridge ─────────��──────────────────────────────────────────────────────
    # diff_drive가 /odom, /tf 발행 → bridge로 ROS에 전달
    # /clock은 Gazebo에서 직접 브리지
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/front_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
    )

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
        name='front_camera_frame_tf',
        arguments=[
            '--x', '0.16', '--y', '0', '--z', '0.08',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'front_camera_frame',
        ],
        output='screen',
    )

    front_camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_camera_optical_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.57079632679', '--pitch', '0', '--yaw', '-1.57079632679',
            '--frame-id', 'front_camera_frame', '--child-frame-id', 'front_camera_optical_frame',
        ],
        output='screen',
    )

    # ── SLAM Toolbox (online async — fresh map every run) ─────────────────────
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
        period=2.0,
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

    # ── Nav2 (navigation only — map comes from SLAM) ──────────────────────────
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
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'autostart': 'true',
                          'params_file': nav2_params,
                          'use_composition': 'True',
                          'container_name': 'nav2_container'}.items(),
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

    return [
        SetEnvironmentVariable('GZ_PARTITION', partition),
        SetEnvironmentVariable('IGN_PARTITION', partition),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        *gz_procs,
        bridge,
        base_footprint_tf,
        front_camera_frame_tf,
        front_camera_optical_tf,
        slam,
        slam_configure,
        slam_activate,
        nav2_container,
        navigation,
        *([] if not use_da3 else [da3_node]),
        *([] if not foxglove else [Node(
            package='foxglove_bridge', executable='foxglove_bridge',
            parameters=[{'port': 8765, 'use_sim_time': True}], output='screen',
        )]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless',    default_value='false',  description='GUI 없이 실행'),
        DeclareLaunchArgument('use_foxglove',default_value='true',  description='Foxglove 브리지'),
        DeclareLaunchArgument('use_da3', default_value='false', description='DA3 RGB depth wrapper'),
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
        OpaqueFunction(function=_launch),
    ])
