# Preset: D456 RGB-D + SLAM Toolbox (Isaac Sim 백엔드)
# d456_slam_toolbox 와 동일한 ROS 스택 (slam_toolbox idle, Nav2, foxglove,
# nvblox, elevator off) 인데 Gazebo 가 아니라 외부 indoory_isaac_sim 이
# 센서/오도 publish 한다. ZMQ 채널 5555/5556/5557 로 연결.
#
# Isaac sim_server 가 어디서 도는지는 ISAAC_HOST 환경변수로 주입.
#   ISAAC_HOST=192.168.1.42 ./run_multisession_slam.sh isaac
# 비우면 127.0.0.1 (로컬에서 sim_server 같이 띄우는 경우).
#
# Multi-robot fleet: sim_server --num-robots N 으로 부팅된 fleet 중 어느
# robot 을 우리 ROS 스택이 운전할지는 ISAAC_ROBOT_ID 로 선택.
#   ISAAC_ROBOT_ID=1 ./run_multisession_slam.sh isaac     # robot 1 운전
PRESET_NAME="d456_isaac"
PRESET_DESC="D456 + SLAM Toolbox (Isaac Sim, ZMQ bridge)"

LAUNCH_ARGS=(
  sim_backend:=isaac
  isaac_host:=${ISAAC_HOST:-127.0.0.1}
  isaac_robot_id:=${ISAAC_ROBOT_ID:-1}
  use_da3:=false
  use_nvblox:=true
  use_vggt_slam:=false
  use_semantic_vlm:=false
  world:=office
  use_rtabmap:=false
  # 부팅 시 slam_toolbox 자동 시작 — passive SLAM (라이다 기반 라이브 mapping +
  # localization). 사용자 텔레옵에 따라 맵이 그려짐. explore_lite (자율 탐사) 는
  # 별도 — 웹의 '자율 탐사' 버튼으로만 시작.
  use_slam_toolbox:=true
  # Isaac 은 단일 씬: 빌딩 텔레포트 비활성.
  use_elevator:=false
  use_foxglove:=true
  use_explore:=false
  # Gazebo client 가 없으니 의미 없지만 일관성을 위해 남겨둠.
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
  /map
  /map_metadata
  /pose
  /local_costmap/costmap
  /global_costmap/costmap
)
