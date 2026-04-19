# Preset: DA3 (mono depth) + nvblox (TSDF/ESDF/mesh).
# 현재 default 조합. 라이다 2-DOF affine 보정 + EMA smoothing.
PRESET_NAME="da3_nvblox"
PRESET_DESC="DA3 mono depth → nvblox TSDF/ESDF/mesh"

LAUNCH_ARGS=(
  use_da3:=true
  use_nvblox:=true
  use_vggt_slam:=false
  use_elevator:=true
  use_foxglove:=true
  headless:=false
)

# bag record 대상 (--record 옵션 사용 시)
RECORD_TOPICS=(
  /camera/image_raw/compressed
  /camera/camera_info
  /scan
  /odom
  /tf
  /tf_static
  /clock
)
