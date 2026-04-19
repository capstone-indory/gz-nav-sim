# Preset: VGGT-SLAM 단독 (nvblox/DA3 없이).
# VGGT가 자체 pose + depth + pointcloud + loop closure.
PRESET_NAME="vggt_only"
PRESET_DESC="VGGT-SLAM 단독 (자체 매핑, 시각화는 viser/global_pointcloud)"

LAUNCH_ARGS=(
  use_da3:=false
  use_nvblox:=false
  use_vggt_slam:=true
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
)
