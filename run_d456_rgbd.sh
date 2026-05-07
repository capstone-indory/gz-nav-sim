#!/bin/bash
# D456 RGB-D + nvblox (3D mapping) 통합 실행.
#
# 동작:
#   Gazebo → 카메라(/camera/image_raw)
#       + D456 native depth(/d456/depth/*)
#           → nvblox: TSDF 통합, mesh / ESDF / occupancy publish
#       + SLAM Toolbox: map→odom TF
#       + Nav2: 2D 내비
#
# 출력 토픽:
#   /nvblox_node/mesh                3D mesh (MarkerArray)
#   /nvblox_node/esdf_pointcloud     ESDF slice
#   /nvblox_node/static_occupancy_grid  2D occupancy

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
source /opt/ros/humble/setup.bash
if [ -f /home/fnhid/lingbot-real-orc/install/explore_lite_msgs/share/explore_lite_msgs/package.bash ]; then
    source /home/fnhid/lingbot-real-orc/install/explore_lite_msgs/share/explore_lite_msgs/package.bash
fi
if [ -f /home/fnhid/lingbot-real-orc/install/explore_lite/share/explore_lite/package.bash ]; then
    source /home/fnhid/lingbot-real-orc/install/explore_lite/share/explore_lite/package.bash
fi
source install/setup.bash

cleanup_existing_sim() {
    echo "[prep] 이전 Gazebo/launch 프로세스 정리..."
    pkill -f '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch gz_nav_sim sim_nav.launch.py' 2>/dev/null || true
    pkill -f 'gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so' 2>/dev/null || true
    pkill -f 'gzserver .*sim_nav_world_' 2>/dev/null || true
    pkill -f 'Xvfb.*:99' 2>/dev/null || true
    sleep 2
}

# nvblox는 CUDA 11.8로 빌드됨 — alternative가 11.8을 가리키도록 보장
if [ -L /usr/local/cuda ] && [ "$(readlink -f /usr/local/cuda)" != "/usr/local/cuda-11.8" ]; then
    echo "[warn] /usr/local/cuda → $(readlink -f /usr/local/cuda) (nvblox는 11.8 기대)"
    echo "[warn] sudo update-alternatives --set cuda /usr/local/cuda-11.8"
fi

cleanup_existing_sim

# Stale X 소켓 정리 후 Xvfb 새로 기동 — 기존 Xvfb를 재사용하면 GL 컨텍스트가 깨짐
rm -f /tmp/.X11-unix/X99 /tmp/.X99-lock 2>/dev/null || true
sleep 1

export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
echo "[1/2] Xvfb 시작..."
Xvfb :99 -screen 0 1280x1024x24 &
XVFB_PID=$!
sleep 2
if ! ps -p $XVFB_PID > /dev/null; then
    echo "[err] Xvfb 시작 실패"; exit 1
fi
trap "kill $XVFB_PID 2>/dev/null || true; pkill -f 'Xvfb.*:99' 2>/dev/null || true; pkill -f 'gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so' 2>/dev/null || true; pkill -f 'gzserver .*sim_nav_world_' 2>/dev/null || true" EXIT

echo "[2/2] vglrun으로 ROS2 launch (GPU EGL 경로)"
echo "[info] D456 RGB-D + nvblox + semantic OCR 실행 — Foxglove ws://localhost:8765"
echo ""

LAUNCH_CMD=(ros2 launch gz_nav_sim sim_nav.launch.py \
  headless:=true \
  use_foxglove:=true \
  robot_model:=robot_d456 \
  direct_depth:=true \
  use_da3:=false \
  use_nvblox:=true \
  use_vggt_slam:=false \
  use_explore:="${USE_EXPLORE:-true}" \
  use_semantic_ocr:="${USE_SEMANTIC_OCR:-true}" \
  ocr_frame_interval:="${OCR_FRAME_INTERVAL:-5}" \
  ocr_min_confidence:="${OCR_MIN_CONFIDENCE:-0.6}" \
  ocr_scales:="${OCR_SCALES:-1.0,2.0}" \
  ocr_floor_hint:="${OCR_FLOOR_HINT:-}" \
  ocr_floor_prior_mode:="${OCR_FLOOR_PRIOR_MODE:-reject}" \
  use_semantic_vlm:="${USE_SEMANTIC_VLM:-false}" \
  vlm_frame_interval:="${VLM_FRAME_INTERVAL:-40}" \
  vlm_model:="${VLM_MODEL:-Qwen/Qwen2.5-VL-3B-Instruct}" \
  vlm_device:="${VLM_DEVICE:-auto}" \
  use_elevator:=true)

if command -v vglrun >/dev/null 2>&1; then
  vglrun -d egl0 "${LAUNCH_CMD[@]}"
else
  echo "[warn] vglrun not found; falling back to plain ros2 launch"
  "${LAUNCH_CMD[@]}"
fi
