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

echo "[1/5] Starting simulation with DA3 depth inference..."
ros2 launch src/gz_nav_sim/launch/sim_nav.launch.py \
  headless:=false \
  use_foxglove:=true \
  use_da3:=true \
  use_elevator:=false > /tmp/sim_da3.log 2>&1 &
SIM_PID=$!

echo "[2/5] Waiting for simulation and DA3 to start (30s)..."
sleep 30

echo "[3/5] Checking camera topics..."
ros2 topic list 2>/dev/null | grep -E "camera|image" | sort

echo ""
echo "[4/5] Checking depth topics from DA3..."
if ros2 topic list 2>/dev/null | grep -q "depth"; then
  echo "✓ Depth topics found:"
  ros2 topic list 2>/dev/null | grep depth | sort
  echo ""
  echo "Checking /front_camera/depth/points frequency (5s)..."
  timeout 5 ros2 topic hz /front_camera/depth/points 2>&1 | head -5 || echo "Topic not publishing yet"
else
  echo "✗ No depth topics found"
fi

echo ""
echo "[5/5] Checking DA3 node status..."
ps aux | grep da3_depth_node | grep -v grep || echo "DA3 node not running"

echo ""
echo "Simulation log tail (last 15 lines with 'da3' or 'error'):"
grep -i "da3\|error\|model" /tmp/sim_da3.log | tail -15 || echo "No matching log entries"

echo ""
echo "Cleaning up in 5 seconds..."
sleep 5
kill $SIM_PID 2>/dev/null || true
