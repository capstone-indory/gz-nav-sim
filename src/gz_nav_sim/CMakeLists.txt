cmake_minimum_required(VERSION 3.8)
project(gz_nav_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/binary_rgbd_bridge_node.py
  scripts/da3_depth_node.py
  scripts/depth_to_laserscan_node.py
  scripts/depth_to_pointcloud_node.py
  scripts/isaac_bridge.py
  scripts/laser_scan_odometry_node.py
  scripts/low_latency_cmd_bridge.py
  scripts/nav_destination_node.py
  scripts/nvblox_mesh_to_gltf.py
  scripts/pointcloud_visualizer_node.py
  scripts/rplidar_c1_scan_node.py
  scripts/rtsp_compressed_bridge_node.py
  scripts/scan_slam_filter_node.py
  scripts/semantic_ocr_node.py
  scripts/semantic_vlm_node.py
  scripts/trajectory_path_node.py
  scripts/vggt_slam_bridge.py
  scripts/vggt_slam_server.py
  scripts/xlerobot_v2_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
