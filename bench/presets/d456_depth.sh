# Preset: Realsense D456 native depth → nvblox 직접.
# DA3/VGGT 모델 사용 X. 카메라 0.8m 마스트 끝.
PRESET_NAME="d456_depth"
PRESET_DESC="D456 native depth → nvblox (모노큐러 모델 우회, 가장 빠름/정확)"

LAUNCH_ARGS=(
  use_da3:=false
  use_nvblox:=true
  use_vggt_slam:=false
  use_elevator:=true
  use_foxglove:=true
  headless:=false
  robot_model:=robot_d456
  direct_depth:=true
)

# bag record — 시각화 / 분석용. raw depth는 제외 (3GB+ 낭비, nvblox voxel
# 출력으로 이미 충분). nvblox 재처리가 필요하면 그때만 raw 추가.
RECORD_TOPICS=(
  /camera/image_raw/compressed
  /camera/camera_info
  /scan
  /odom
  /tf
  /tf_static
  /clock
  # D456 metadata (raw image는 제외 — 너무 큼)
  /d456/depth/camera_info
  # SLAM / Nav2 출력
  /map
  /local_costmap/costmap
  /global_costmap/costmap
  # nvblox 출력 — voxel quantized 맵
  /nvblox_node/scene
  /nvblox_node/static_occupancy
  /nvblox_node/static_esdf_pointcloud
)
