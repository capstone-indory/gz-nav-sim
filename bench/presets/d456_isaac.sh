# Preset: D456 + SLAM Toolbox + nvblox (Isaac Sim 백엔드)
# 라이다 기반 2D SLAM (slam_toolbox) 가 map→odom TF 와 /map 을 발행.
# nvblox 는 그 TF 위에 depth 를 누적해 3D mesh 시각화.
#
# Isaac sim_server 호스트는 ISAAC_HOST 환경변수.
#   ISAAC_HOST=100.80.87.68 ./run_multisession_slam.sh isaac
# Multi-robot fleet 중 어느 robot 을 운전할지는 ISAAC_ROBOT_ID.
#   ISAAC_ROBOT_ID=1 ./run_multisession_slam.sh isaac
PRESET_NAME="d456_isaac"
PRESET_DESC="D456 + SLAM Toolbox (Isaac Sim, ZMQ bridge)"

LAUNCH_ARGS=(
  sim_backend:=isaac
  isaac_host:=${ISAAC_HOST:-127.0.0.1}
  isaac_robot_id:=${ISAAC_ROBOT_ID:-1}
  use_da3:=false
  # nvblox: SLAM 이 아니라 3D 볼륨 매핑 시각화 — slam_toolbox 의 TF 위에 mesh 만 그림.
  use_nvblox:=true
  use_vggt_slam:=false
  use_semantic_vlm:=false
  world:=office
  # 라이다 우선 SLAM. RTAB-Map 은 사용 안 함.
  use_rtabmap:=false
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
