# Preset: D456 RGB 입력 + VGGT-SLAM (depth + pose). nvblox 사용 X (VGGT 자체 출력).
# 카메라 0.8m. RGB만 사용. VGGT venv (Python 3.11) 필요.
PRESET_NAME="d456_vggt"
PRESET_DESC="D456 RGB → VGGT-SLAM (D456 환경, RGB-only e2e SLAM)"

LAUNCH_ARGS=(
  use_da3:=false
  use_nvblox:=false
  use_vggt_slam:=true
  use_elevator:=true
  use_foxglove:=true
  headless:=false
  robot_model:=robot_d456
  direct_depth:=false
)

RECORD_TOPICS=(
  /camera/image_raw/compressed
  /camera/camera_info
  /scan
  /odom
  /tf
  /tf_static
  /clock
  # VGGT-SLAM 출력
  /vggt_slam/pose
  /vggt_slam/path
  /vggt_slam/pointcloud
  # SLAM / Nav2
  /map
  /local_costmap/costmap
  /global_costmap/costmap
)
