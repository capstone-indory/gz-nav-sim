#!/bin/bash
# VGGT-SLAM 단독 실행 (DA3, nvblox 비활성).
#
# VGGT-SLAM은 자체적으로 pose + depth + pointcloud 추정. DA3 depth 불필요.
#
# 동작:
#   Gazebo → /camera/image_raw/compressed
#       → vggt_slam_bridge (py3.10): JPEG → ZeroMQ
#           → vggt_slam_server (py3.11 venv): VGGT-1B 추론
#               → /vggt_slam/{pose,trajectory,pointcloud}
#       + SLAM Toolbox: map→odom TF (Nav2용 기존 라이다 SLAM)
#
# 첫 실행 시 VGGT-1B (~3GB) HuggingFace 다운로드로 1~2분 응답 없음 — 정상.
# Foxglove에서 frame=map 로 trajectory 보면 됨.

set -e

cd /root/gz-nav-sim
source /opt/ros/humble/setup.bash
source install/setup.bash

# venv 확인
VENV_PY=/root/gz-nav-sim/venv_vggt/bin/python
if [ ! -x "$VENV_PY" ]; then
    echo "[err] VGGT-SLAM venv가 없습니다: $VENV_PY"
    exit 1
fi

# Stale X 소켓 정리 후 Xvfb 새로 기동
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
echo "[info] VGGT-SLAM 실행 — 첫 실행은 VGGT-1B 다운로드 1~2분 응답없음 (정상)"
echo "[info] Foxglove: ws://localhost:8765"
echo ""

vglrun -d egl0 ros2 launch gz_nav_sim sim_nav.launch.py \
  headless:=false \
  use_foxglove:=true \
  use_da3:=false \
  use_nvblox:=false \
  use_vggt_slam:=true \
  use_elevator:=true
