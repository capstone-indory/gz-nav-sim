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
  # headless:=true로 두면 Gazebo Classic의 CameraSensor 초기화 실패 →
  # "Rendering is disabled" 에러로 카메라 토픽 publish 0건 → DA3 idle.
  # gzclient 30~50% CPU 비용 있더라도 camera sensor 동작 조건이라 false 유지.
  headless:=false
)

# bag record 대상 (--record 옵션 사용 시)
# Foxglove 오프라인 재생 시 DA3/nvblox 결과까지 다 시각화 가능하도록 포함.
# 단점: bag 크기 커짐 (high-rate sensor + dense pointcloud).
RECORD_TOPICS=(
  # Sensor / state
  /camera/image_raw/compressed
  /camera/camera_info
  /scan
  /odom
  /tf
  /tf_static
  /clock
  # DA3 출력
  /camera/depth/image_raw
  /camera/depth/world_points
  /camera/depth/global_map_delta
  # SLAM / Nav2
  /map
  /local_costmap/costmap
  /global_costmap/costmap
  # nvblox mesh visualization (foxglove SceneUpdate)
  /nvblox_node/scene
)
