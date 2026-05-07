# Preset: D456 RGB-D + RTAB-Map (multi-session SLAM)
# slam_toolbox / VGGT-SLAM / nvblox 비활성. RTAB-Map 단독.
PRESET_NAME="d456_rtabmap"
PRESET_DESC="D456 RGB-D + RTAB-Map (DB 영속, 멀티세션)"

LAUNCH_ARGS=(
  use_da3:=false
  use_nvblox:=false
  use_vggt_slam:=false
  use_rtabmap:=true
  use_slam_toolbox:=false
  use_elevator:=true
  use_foxglove:=true
  # frontier 자율 탐사 자동 시작 (Nav2 active 후 explore_lite spawn).
  # 웹 'Explore' 버튼이 같은 일을 별도 subprocess 로 시도하므로 둘 중 하나만.
  use_explore:=true
  # gzclient 가 헤드리스 환경에서도 sensor/scene 데이터를 gzserver 로부터 수십 개
  # TCP 로 받아 loopback 177 MB/s 폭주. 시각화는 Foxglove(ws://8765)로 충분.
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
  # RTAB-Map 출력
  /map
  /rtabmap/grid_map
  /rtabmap/cloud_map
  /rtabmap/info
  /rtabmap/mapData
  # Nav2
  /local_costmap/costmap
  /global_costmap/costmap
)
