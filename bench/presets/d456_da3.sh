# Preset: D456 RGB 입력 + DA3 mono depth 추정 → nvblox.
# 카메라 0.8m. D456 native depth는 사용 안 함 (RGB만).
PRESET_NAME="d456_da3"
PRESET_DESC="D456 RGB → DA3 mono depth → nvblox (D456 환경에서 DA3 정확도 비교)"

LAUNCH_ARGS=(
  use_da3:=true
  use_nvblox:=true
  use_vggt_slam:=false
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
  # DA3 출력 (delta는 작음, world_points는 작음, depth는 raw float32라 무거우니 제외)
  /camera/depth/world_points
  /camera/depth/global_map_delta
  # SLAM / Nav2
  /map
  /local_costmap/costmap
  /global_costmap/costmap
  /nvblox_node/scene
)
