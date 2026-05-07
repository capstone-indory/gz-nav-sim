# Preset: D456 RGB-D + SLAM Toolbox (idle on boot — 명령 기반 액티브)
# 부팅 시 Gazebo + Nav2 + foxglove 만 띄우고 SLAM/탐사는 안 함.
# 웹의 "매핑·탐사 시작" 버튼 → adapter 가 slam_toolbox + explore_lite spawn.
PRESET_NAME="d456_slam_toolbox"
PRESET_DESC="D456 + SLAM Toolbox (idle, 명령 시작)"

LAUNCH_ARGS=(
  use_da3:=false
  use_nvblox:=false
  use_vggt_slam:=false
  use_rtabmap:=false
  # SLAM 노드는 부팅 시 안 띄움 — 웹에서 시작 명령 받으면 adapter 가 spawn.
  use_slam_toolbox:=false
  use_elevator:=true
  use_foxglove:=true
  # 자율 탐사도 부팅 시 안 띄움 — 같은 흐름.
  use_explore:=false
  # gzclient 헤드리스 (loopback 폭주 방지). 시각화는 Foxglove(ws://8765).
  headless:=true
  robot_model:=robot_d456
  direct_depth:=true
)

RECORD_TOPICS=(
  /camera/image_raw/compressed
  /camera/camera_info
  /d456/depth/camera_info
  /scan
  /odom
  /tf
  /tf_static
  /clock
  # slam_toolbox 출력
  /map
  /map_metadata
  /pose
  # Nav2
  /local_costmap/costmap
  /global_costmap/costmap
)
