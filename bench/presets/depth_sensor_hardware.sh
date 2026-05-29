# Preset: real XLeRobot hardware via a remote robot I/O computer.
# The robot computer runs ./run_xlerobot_rosbridge_io.sh and owns all hardware:
#   /xlerobot/scan, depth sensor compressed RGB/depth, and depth sensor IMU publish
#   /xlerobot/cmd_vel subscribe
# This compute PC only runs SLAM/Nav2/Foxglove and bridges those /xlerobot
# topics into the local navigation interface. In the default hardware path,
# /xlerobot/* comes in through local rosbridge_server, not DDS discovery.
# Default hardware path uses RTAB-Map with LiDAR-only ICP odometry and scan
# occupancy/refinement. It does not consume robot/base wheel odom. Use
# RTABMAP_ODOM_SOURCE=fusion for RGB-D visual odom plus LiDAR refinement, or
# RTABMAP_ODOM_SOURCE=rgbd for camera-only odom.
PRESET_NAME="depth_sensor_hardware"
PRESET_DESC="Remote XLeRobot rosbridge I/O + LiDAR-only RTAB-Map/Nav2/Foxglove"

USE_RTABMAP_DEFAULT="${USE_RTABMAP:-true}"
USE_SLAM_TOOLBOX_DEFAULT="${USE_SLAM_TOOLBOX:-false}"
if [[ "${USE_RTABMAP_DEFAULT,,}" == "true" && -z "${USE_SLAM_TOOLBOX:-}" ]]; then
  USE_SLAM_TOOLBOX_DEFAULT="false"
fi

LAUNCH_ARGS=(
  isaac_transport:=${ISAAC_TRANSPORT:-xlerobot_ros}
  isaac_host:=${ISAAC_HOST:-127.0.0.1}
  isaac_robot_id:=${ISAAC_ROBOT_ID:-1}
  ros_localhost_only:=${ROS_LOCALHOST_ONLY:-1}
  use_sim_time:=false
  use_hardware_lidar:=false
  use_foxglove:=true
  foxglove_profile:=map
  xlerobot_scan_frame:=${XLEROBOT_SCAN_FRAME:-laser}
  robot_lidar_x:=${ROBOT_LIDAR_X:-0.200}
  robot_lidar_y:=${ROBOT_LIDAR_Y:-0.0}
  robot_lidar_z:=${ROBOT_LIDAR_Z:-0.730}
  robot_lidar_roll:=${ROBOT_LIDAR_ROLL:-0.0}
  robot_lidar_pitch:=${ROBOT_LIDAR_PITCH:-0.0}
  robot_lidar_yaw:=${ROBOT_LIDAR_YAW:-0.0}
  robot_camera_x:=${ROBOT_CAMERA_X:-0.0}
  robot_camera_y:=${ROBOT_CAMERA_Y:-0.020}
  robot_camera_z:=${ROBOT_CAMERA_Z:-0.0}
  robot_camera_roll:=${ROBOT_CAMERA_ROLL:-0.0}
  robot_camera_pitch:=${ROBOT_CAMERA_PITCH:-0.0}
  robot_camera_yaw:=${ROBOT_CAMERA_YAW:-0.0}
  use_slam_toolbox:=${USE_SLAM_TOOLBOX_DEFAULT}
  use_lidar_odom:=${USE_LIDAR_ODOM:-false}
  use_depth_scan_fallback:=${USE_DEPTH_SCAN_FALLBACK:-true}
  depth_scan_publish_rate_hz:=${DEPTH_SCAN_PUBLISH_RATE_HZ:-15.0}
  enable_base_odom_bridge:=false
  rtabmap_odom_source:=${RTABMAP_ODOM_SOURCE:-icp}
  rtabmap_db:=${RTABMAP_DB:-}
  use_imu:=${USE_IMU:-false}
  use_binary_rgbd_bridge:=${USE_BINARY_RGBD_BRIDGE:-true}
  binary_rgbd_host:=${BINARY_RGBD_HOST:-0.0.0.0}
  binary_rgbd_port:=${BINARY_RGBD_PORT:-9102}
  use_rtsp_camera_bridge:=${USE_RTSP_CAMERA_BRIDGE:-true}
  rtsp_camera_names:=${RTSP_CAMERA_NAMES:-base,wrist_left,wrist_right}
  rtsp_camera_publish_rate_hz:=${RTSP_CAMERA_PUBLISH_RATE_HZ:-15.0}
  rtsp_camera_jpeg_quality:=${RTSP_CAMERA_JPEG_QUALITY:-80}
  rtsp_base_camera_url:=${RTSP_BASE_CAMERA_URL:-rtsp://127.0.0.1:${MEDIAMTX_RTSP_PORT:-8554}/${ROBOT_BASE_CAMERA_PATH:-xlerobot_base}}
  rtsp_wrist_left_camera_url:=${RTSP_WRIST_LEFT_CAMERA_URL:-rtsp://127.0.0.1:${MEDIAMTX_RTSP_PORT:-8554}/${ROBOT_WRIST_LEFT_CAMERA_PATH:-xlerobot_wrist_left}}
  rtsp_wrist_right_camera_url:=${RTSP_WRIST_RIGHT_CAMERA_URL:-rtsp://127.0.0.1:${MEDIAMTX_RTSP_PORT:-8554}/${ROBOT_WRIST_RIGHT_CAMERA_PATH:-xlerobot_wrist_right}}
  use_nav_goal_bridge:=${USE_NAV_GOAL_BRIDGE:-true}
  nav_destinations_file:=${NAV_DESTINATIONS_FILE:-${REPO_ROOT:-$PWD}/src/gz_nav_sim/config/nav_destinations.yaml}
  nav_goal_from_clicked_point:=${NAV_GOAL_FROM_CLICKED_POINT:-false}
  use_nav_scan_filter:=${USE_NAV_SCAN_FILTER:-true}
  nav_scan_filter_min_range:=${NAV_SCAN_FILTER_MIN_RANGE:-0.20}
  nav_scan_filter_max_range:=${NAV_SCAN_FILTER_MAX_RANGE:-0.0}
  nav_scan_filter_remove_isolated_clusters:=${NAV_SCAN_FILTER_REMOVE_ISOLATED_CLUSTERS:-false}
  nav_scan_filter_min_cluster_points:=${NAV_SCAN_FILTER_MIN_CLUSTER_POINTS:-2}
  nav_scan_filter_cluster_jump_m:=${NAV_SCAN_FILTER_CLUSTER_JUMP_M:-0.30}
  nav_scan_filter_cluster_max_range:=${NAV_SCAN_FILTER_CLUSTER_MAX_RANGE:-2.5}
  use_slam_scan_filter:=${USE_SLAM_SCAN_FILTER:-true}
  slam_scan_filter_min_range:=${SLAM_SCAN_FILTER_MIN_RANGE:-0.20}
  slam_scan_filter_max_range:=${SLAM_SCAN_FILTER_MAX_RANGE:-0.0}
  slam_scan_filter_remove_isolated_clusters:=${SLAM_SCAN_FILTER_REMOVE_ISOLATED_CLUSTERS:-true}
  slam_scan_filter_min_cluster_points:=${SLAM_SCAN_FILTER_MIN_CLUSTER_POINTS:-3}
  slam_scan_filter_cluster_jump_m:=${SLAM_SCAN_FILTER_CLUSTER_JUMP_M:-0.30}
  slam_scan_filter_cluster_max_range:=${SLAM_SCAN_FILTER_CLUSTER_MAX_RANGE:-2.5}
  cmd_max_linear_x:=${CMD_MAX_LINEAR_X:-0.30}
  cmd_max_linear_y:=${CMD_MAX_LINEAR_Y:-0.30}
  cmd_max_angular_z:=${CMD_MAX_ANGULAR_Z:-1.00}
  lidar_odom_max_range:=${LIDAR_ODOM_MAX_RANGE:-8.0}
  lidar_odom_max_points:=${LIDAR_ODOM_MAX_POINTS:-240}
  lidar_odom_icp_iterations:=${LIDAR_ODOM_ICP_ITERATIONS:-8}
  lidar_odom_max_correspondence_distance:=${LIDAR_ODOM_MAX_CORRESPONDENCE_DISTANCE:-0.35}
  lidar_odom_min_pairs:=${LIDAR_ODOM_MIN_PAIRS:-35}
  lidar_odom_max_translation_per_scan:=${LIDAR_ODOM_MAX_TRANSLATION_PER_SCAN:-0.35}
  lidar_odom_max_rotation_per_scan:=${LIDAR_ODOM_MAX_ROTATION_PER_SCAN:-0.60}
  lidar_odom_invert_delta:=${LIDAR_ODOM_INVERT_DELTA:-false}
  use_rtabmap:=${USE_RTABMAP_DEFAULT}
  use_da3:=false
  use_nvblox:=false
  use_vggt_slam:=false
  use_semantic_vlm:=false
  use_semantic_ocr:=false
  use_explore:=false
  direct_depth:=true
)

RECORD_TOPICS=(
  /scan_raw
  /scan
  /scan_slam
  /odom
  /tf
  /tf_static
  /map
  /map_metadata
  /rtabmap/info
  /rtabmap/cloud_map
  /local_costmap/costmap
  /global_costmap/costmap
  /plan
  /goal_pose
  /nav/destination
  /nav/goal_pose2d
  /camera/image_raw/compressed
  /camera/camera_info
  /depth/image_raw
  /depth/camera_info
  /depth/points
  /imu/data
  /cmd_vel
  /cmd_vel_mux
  /xlerobot/cmd_vel
  /xlerobot/scan
  /xlerobot/base_camera/image/compressed
  /xlerobot/base_camera/camera_info
  /xlerobot/wrist_left_camera/image/compressed
  /xlerobot/wrist_left_camera/camera_info
  /xlerobot/wrist_right_camera/image/compressed
  /xlerobot/wrist_right_camera/camera_info
  /xlerobot/head_camera/color/image
  /xlerobot/head_camera/color/camera_info
  /xlerobot/head_camera/depth/image
  /xlerobot/head_camera/depth/camera_info
  /xlerobot/head_camera/imu
  /xlerobot/imu/data
)
