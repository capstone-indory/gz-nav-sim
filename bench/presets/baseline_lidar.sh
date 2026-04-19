# Preset: depth 매핑 끄고 SLAM Toolbox + Nav2만 (2D 라이다 baseline).
# nvblox/DA3/VGGT 모두 OFF. 매핑 비교의 lower bound.
PRESET_NAME="baseline_lidar"
PRESET_DESC="2D 라이다 + SLAM Toolbox + Nav2만 (depth 매핑 OFF, baseline)"

LAUNCH_ARGS=(
  use_da3:=false
  use_nvblox:=false
  use_vggt_slam:=false
  use_elevator:=true
  use_foxglove:=true
  headless:=false
)

RECORD_TOPICS=(
  /camera/image_raw/compressed
  /camera/camera_info
  /scan
  /odom
  /tf
  /tf_static
  /clock
  /map
)
