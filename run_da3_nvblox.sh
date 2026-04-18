#!/bin/bash
# DA3 (monocular depth) + nvblox (3D mapping) 통합 실행.
#
# 동작:
#   Gazebo → 카메라(/camera/image_raw)
#       → DA3 노드: depth + camera_info publish
#           → nvblox: TSDF 통합, mesh / ESDF / occupancy publish
#       + SLAM Toolbox: map→odom TF
#       + Nav2: 2D 내비
#
# 출력 토픽:
#   /nvblox_node/mesh                3D mesh (MarkerArray)
#   /nvblox_node/esdf_pointcloud     ESDF slice
#   /nvblox_node/static_occupancy_grid  2D occupancy

set -e

cd /root/gz-nav-sim
source /opt/ros/humble/setup.bash
source install/setup.bash

# nvblox는 CUDA 11.8로 빌드됨 — alternative가 11.8을 가리키도록 보장
if [ -L /usr/local/cuda ] && [ "$(readlink -f /usr/local/cuda)" != "/usr/local/cuda-11.8" ]; then
    echo "[warn] /usr/local/cuda → $(readlink -f /usr/local/cuda) (nvblox는 11.8 기대)"
    echo "[warn] sudo update-alternatives --set cuda /usr/local/cuda-11.8"
fi

# Stale X 소켓 정리 후 Xvfb 새로 기동 — 기존 Xvfb를 재사용하면 GL 컨텍스트가 깨짐
pkill -f 'Xvfb.*:99' 2>/dev/null || true
rm -f /tmp/.X11-unix/X99 /tmp/.X99-lock 2>/dev/null || true
sleep 1

export DISPLAY=:99
echo "[1/2] Xvfb 시작..."
Xvfb :99 -screen 0 1280x1024x24 &
XVFB_PID=$!
sleep 2
if ! ps -p $XVFB_PID > /dev/null; then
    echo "[err] Xvfb 시작 실패"; exit 1
fi
trap "kill $XVFB_PID 2>/dev/null || true; pkill -f 'Xvfb.*:99' 2>/dev/null || true" EXIT

echo "[2/2] vglrun으로 ROS2 launch (GPU EGL 경로)"
echo "[info] DA3 + nvblox 실행 — Foxglove ws://localhost:8765"
echo ""

vglrun -d egl0 ros2 launch gz_nav_sim sim_nav.launch.py \
  headless:=false \
  use_foxglove:=true \
  use_da3:=true \
  use_nvblox:=true \
  use_vggt_slam:=false \
  use_elevator:=true \
  da3_inference_rate_hz:=5.0
