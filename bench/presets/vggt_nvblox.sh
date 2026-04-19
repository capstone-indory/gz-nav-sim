# Preset: VGGT-SLAM + nvblox 결합.
# 현재 launch는 두 mapper 동시 실행 가능 (서로 다른 토픽).
# VGGT는 자체 pointcloud, nvblox는 DA3 depth 필요 → DA3도 같이 켬.
# (VGGT pose를 nvblox로 흘리는 직접 결합은 별도 작업)
PRESET_NAME="vggt_nvblox"
PRESET_DESC="VGGT-SLAM + DA3 + nvblox 동시 (토픽 분리, 비교 관찰용)"

LAUNCH_ARGS=(
  use_da3:=true
  use_nvblox:=true
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
