# Preset: depth sensor + LiDAR-authoritative RTAB-Map (Isaac Sim v2 backend)
# XLeRobot Hospital Isaac 앱이
# rosbridge_server 를 통해 /xlerobot 네임스페이스로 센서/오도를 publish 한다.
#
# ROS PC에서 rosbridge_server 를 먼저 띄우고, Isaac 앱은 그 rosbridge 에 붙인다.
# 이 ROS 스택은 같은 ROS graph 안의 /xlerobot 토픽을 구독/발행한다.
#
# Legacy ZMQ 서버가 필요하면 launch arg 를 isaac_transport:=zmq_v1 로 바꾸고
# ISAAC_HOST / ISAAC_ROBOT_ID 를 사용한다.
PRESET_NAME="depth_sensor_isaac"
PRESET_DESC="depth sensor + RGB-D/LiDAR fusion RTAB-Map stack (Isaac v2, /xlerobot ROS topics)"

USE_RTABMAP_DEFAULT="${USE_RTABMAP:-true}"
USE_SLAM_TOOLBOX_DEFAULT="${USE_SLAM_TOOLBOX:-false}"
if [[ "${USE_RTABMAP_DEFAULT,,}" == "false" && -z "${USE_SLAM_TOOLBOX:-}" ]]; then
  USE_SLAM_TOOLBOX_DEFAULT="true"
fi

LAUNCH_ARGS=(
  isaac_transport:=${ISAAC_TRANSPORT:-xlerobot_ros}
  isaac_host:=${ISAAC_HOST:-127.0.0.1}
  isaac_robot_id:=${ISAAC_ROBOT_ID:-1}
  robot_camera_x:=${ROBOT_CAMERA_X:-0.0}
  robot_camera_y:=${ROBOT_CAMERA_Y:-0.020}
  robot_camera_z:=${ROBOT_CAMERA_Z:-0.0}
  robot_camera_roll:=${ROBOT_CAMERA_ROLL:-0.0}
  robot_camera_pitch:=${ROBOT_CAMERA_PITCH:-0.0}
  robot_camera_yaw:=${ROBOT_CAMERA_YAW:-0.0}
  use_da3:=${USE_DA3:-true}
  use_nvblox:=${USE_NVBLOX:-true}
  use_vggt_slam:=false
  use_semantic_vlm:=false
  use_semantic_ocr:=${USE_SEMANTIC_OCR:-true}
  use_rtabmap:=${USE_RTABMAP_DEFAULT}
  rtabmap_odom_source:=${RTABMAP_ODOM_SOURCE:-fusion}
  use_imu:=${USE_IMU:-true}
  rtabmap_localization:=false
  use_slam_toolbox:=${USE_SLAM_TOOLBOX_DEFAULT}
  use_depth_scan_fallback:=${USE_DEPTH_SCAN_FALLBACK:-false}
  depth_scan_publish_rate_hz:=${DEPTH_SCAN_PUBLISH_RATE_HZ:-15.0}
  use_nav_scan_filter:=${USE_NAV_SCAN_FILTER:-true}
  use_slam_scan_filter:=${USE_SLAM_SCAN_FILTER:-true}
  use_foxglove:=true
  foxglove_profile:=full
  use_explore:=false
  direct_depth:=${DIRECT_DEPTH:-true}
)

RECORD_TOPICS=(
  /camera/image_raw
  /camera/image_raw/compressed
  /camera/camera_info
  /depth/image_raw
  /depth/camera_info
  /imu/data
  /depth/points
  /depth/points_visual
  /scan
  /odom
  /tf
  /tf_static
  /clock
  /map
  /map_metadata
  /rtabmap/cloud_map
  /rtabmap/info
  /semantic_ocr/detections
  /semantic_ocr/markers
  /semantic_ocr/image_annotations
  /pose
  /local_costmap/costmap
  /global_costmap/costmap
)
