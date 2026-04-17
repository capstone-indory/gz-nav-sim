#!/bin/bash
set -e

cd /root/gz-nav-sim
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start Xvfb
export DISPLAY=:99
Xvfb :99 -screen 0 1280x1024x24 -ac 2>/dev/null &
XVFB_PID=$!
trap "kill $XVFB_PID 2>/dev/null || true" EXIT
sleep 2

echo "[1/4] Starting simulation with camera..."
ros2 launch src/gz_nav_sim/launch/sim_nav.launch.py \
  headless:=false \
  use_foxglove:=true \
  use_da3:=false \
  use_elevator:=false > /tmp/sim.log 2>&1 &
SIM_PID=$!

echo "[2/4] Waiting for simulation to start (20s)..."
sleep 20

echo "[3/4] Checking camera topics..."
if ros2 topic list 2>/dev/null | grep -q camera; then
  echo "✓ Camera topics found!"
  ros2 topic list 2>/dev/null | grep camera

  echo ""
  echo "[4/4] Checking /front_camera/image_raw frequency (5s)..."
  timeout 5 ros2 topic hz /front_camera/image_raw 2>&1 | head -10 || {
    echo "Topic exists but not publishing"
    ros2 topic echo /front_camera/image_raw --once 2>&1 | head -5 || echo "Cannot read topic"
  }
else
  echo "✗ No camera topics found!"
  echo "Available topics:"
  ros2 topic list 2>/dev/null | head -20

  echo ""
  echo "Checking simulation log for errors:"
  grep -i "camera\|error\|failed" /tmp/sim.log | tail -10 || echo "No camera-related messages in log"
fi

echo ""
echo "Cleaning up..."
kill $SIM_PID 2>/dev/null || true
