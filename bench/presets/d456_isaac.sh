# Preset: D456 RGB-D + SLAM Toolbox (Isaac Sim v2 백엔드)
# XLeRobot Hospital Isaac 앱이
# rosbridge_server 를 통해 /xlerobot 네임스페이스로 센서/오도를 publish 한다.
#
# ROS PC에서 rosbridge_server 를 먼저 띄우고, Isaac 앱은 그 rosbridge 에 붙인다.
# 이 ROS 스택은 같은 ROS graph 안의 /xlerobot 토픽을 구독/발행한다.
#
# Legacy ZMQ 서버가 필요하면 launch arg 를 isaac_transport:=zmq_v1 로 바꾸고
# ISAAC_HOST / ISAAC_ROBOT_ID 를 사용한다.
PRESET_NAME="d456_isaac"
PRESET_DESC="D456 + SLAM Toolbox (XLeRobot Hospital Isaac v2, /xlerobot ROS topics)"

LAUNCH_ARGS=(
  isaac_transport:=${ISAAC_TRANSPORT:-rosbridge_v2}
  isaac_host:=${ISAAC_HOST:-127.0.0.1}
  isaac_robot_id:=${ISAAC_ROBOT_ID:-1}
  use_da3:=false
  # nvblox 비활성 — 3D 시각화는 RTAB-Map cloud_map (RGB-D 누적 colored cloud,
  # /var/indoory/maps/{id}.db 에 같이 영속) 으로 통일. mesh 가 다시 필요해지면 켜면 됨.
  use_nvblox:=false
  use_vggt_slam:=false
  use_semantic_vlm:=false
  # RTAB-Map scan-only 모드: 라이다만 사용해서 slam_toolbox 와 동일한 시각 품질 +
  # 멀티세션 .db 영속화. depth/RGB 는 sync timing 이슈로 사용 안 함 (Isaac sim 의
  # RGB-depth 700ms gap, sub-13% base actuator 등 회피).
  use_rtabmap:=true
  rtabmap_localization:=false
  use_slam_toolbox:=false
  use_foxglove:=true
  use_explore:=false
  # OCR detection rate 부스트 (default: backend=paddle, interval=5, conf=0.6, scales=1.0,2.0).
  # 작은 표지판도 catch 위해 confidence 낮추고, 멀리 있는 표지판 위해 zoom scale
  # 추가, 처리 빈도 늘림. paddle 모듈 없으면 코드가 자동 tesseract fallback.
  ocr_frame_interval:=2
  ocr_min_confidence:=0.4
  ocr_scales:=1.0,1.5,2.0,3.0
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
