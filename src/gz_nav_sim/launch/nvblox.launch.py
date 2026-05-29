"""nvblox 3D mapping node — RGB-D depth + XLeRobot RGB input.

분리된 include용 런치. sim_nav.launch.py에서 use_nvblox:=true로 포함.

입력 토픽:
  /depth/image_raw       sensor_msgs/Image (depth sensor native depth)
  /depth/camera_info     sensor_msgs/CameraInfo
  /camera/image_raw           sensor_msgs/Image (XLeRobot RGB)
  /camera/camera_info         sensor_msgs/CameraInfo

출력 (nvblox_node 자체 토픽):
  ~/mesh                      visualization_msgs/Marker (3D mesh)
  ~/esdf_pointcloud           sensor_msgs/PointCloud2
  ~/static_occupancy_grid     nav_msgs/OccupancyGrid (옵션)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('gz_nav_sim')
    nvblox_params = os.path.join(pkg, 'config', 'nvblox_params.yaml')

    depth_topic = LaunchConfiguration('depth_topic')
    depth_info_topic = LaunchConfiguration('depth_info_topic')
    color_topic = LaunchConfiguration('color_topic')
    color_info_topic = LaunchConfiguration('color_info_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # GPU 1 (RTX 3090, 24GB) 단독 점유.
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[nvblox_params, {
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        additional_env={'CUDA_VISIBLE_DEVICES': '1'},
        remappings=[
            ('depth/image', depth_topic),
            ('depth/camera_info', depth_info_topic),
            ('color/image', color_topic),
            ('color/camera_info', color_info_topic),
            # /pose schema 충돌 회피: slam_toolbox가 PoseWithCovarianceStamped로
            # publish vs nvblox는 PoseStamped로 구독 → Foxglove 경고. nvblox는
            # TF로 pose 받으니 unused 이름으로 remap해 매칭 끊음.
            ('pose', 'nvblox_pose_unused'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('depth_topic', default_value='/depth/image_raw'),
        DeclareLaunchArgument('depth_info_topic', default_value='/depth/camera_info'),
        DeclareLaunchArgument('color_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('color_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        nvblox_node,
    ])
