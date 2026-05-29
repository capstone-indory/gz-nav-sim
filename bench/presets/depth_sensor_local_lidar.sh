# Preset: local USB RPLIDAR bench mode.
# Use this only when the lidar is physically plugged into this compute PC.
# It still consumes /xlerobot/odom and publishes /xlerobot/cmd_vel, but /scan
# comes from this PC's serial RPLIDAR node instead of /xlerobot/scan.
PRESET_NAME="depth_sensor_local_lidar"
PRESET_DESC="Local USB RPLIDAR bench + XLeRobot odom/cmd + SLAM Toolbox/Nav2"

LAUNCH_ARGS=(
  isaac_transport:=${ISAAC_TRANSPORT:-xlerobot_ros}
  isaac_host:=${ISAAC_HOST:-127.0.0.1}
  isaac_robot_id:=${ISAAC_ROBOT_ID:-1}
  ros_localhost_only:=${ROS_LOCALHOST_ONLY:-0}
  use_sim_time:=false
  use_hardware_lidar:=true
  hardware_lidar_serial:=${HARDWARE_LIDAR_SERIAL:-/dev/ttyUSB0}
  hardware_lidar_baud:=${HARDWARE_LIDAR_BAUD:-460800}
  hardware_lidar_frame:=${HARDWARE_LIDAR_FRAME:-laser}
  hardware_lidar_samples:=${HARDWARE_LIDAR_SAMPLES:-720}
  hardware_lidar_angle_offset_deg:=${HARDWARE_LIDAR_ANGLE_OFFSET_DEG:-0.0}
  hardware_lidar_invert:=${HARDWARE_LIDAR_INVERT:-false}
  hardware_lidar_range_min:=${HARDWARE_LIDAR_RANGE_MIN:-0.12}
  hardware_lidar_range_max:=${HARDWARE_LIDAR_RANGE_MAX:-12.0}
  hardware_lidar_min_quality:=${HARDWARE_LIDAR_MIN_QUALITY:-0}
  xlerobot_scan_frame:=${XLEROBOT_SCAN_FRAME:-laser}
  robot_lidar_x:=${ROBOT_LIDAR_X:-0.200}
  robot_lidar_y:=${ROBOT_LIDAR_Y:-0.0}
  robot_lidar_z:=${ROBOT_LIDAR_Z:-0.730}
  use_foxglove:=true
  foxglove_profile:=map
  use_slam_toolbox:=true
  use_rtabmap:=false
  use_da3:=false
  use_nvblox:=false
  use_vggt_slam:=false
  use_semantic_vlm:=false
  use_semantic_ocr:=false
  use_explore:=false
  direct_depth:=true
)

RECORD_TOPICS=(
  /scan
  /odom
  /tf
  /tf_static
  /map
  /map_metadata
  /local_costmap/costmap
  /global_costmap/costmap
  /plan
  /cmd_vel
  /cmd_vel_mux
)
