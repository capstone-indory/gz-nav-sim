#!/bin/bash
set -e

# Start Xvfb and export DISPLAY
export DISPLAY=:99
Xvfb :99 -screen 0 1280x1024x24 &
XVFB_PID=$!
sleep 2

echo "=== Xvfb started with PID $XVFB_PID on DISPLAY=$DISPLAY ==="
echo "=== Environment check ==="
echo "DISPLAY=$DISPLAY"
echo "GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"
echo "GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH"
echo "GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH"
echo ""

echo "=== Checking Gazebo 11 files ==="
ls -la /usr/share/gazebo-11/media/ | head -5
echo ""

echo "=== Starting ROS 2 launch ==="
cd /root/gz-nav-sim
ros2 launch src/gz_nav_sim/launch/sim_nav.launch.py headless:=false use_foxglove:=true use_da3:=true use_elevator:=false 2>&1 | tee /tmp/launch.log &
LAUNCH_PID=$!

# Wait for simulation to start
sleep 15

echo ""
echo "=== Checking ROS topics ==="
ros2 topic list | grep -E "(camera|image|scan|cloud)" || echo "No camera/image topics found"
ros2 topic hz /front_camera/image_raw 2>&1 | head -20 &
sleep 3

echo ""
echo "=== Checking gazebo topics ==="
ros2 topic list | grep gazebo || echo "No gazebo topics"

echo ""
echo "=== Checking gzserver errors in launch log ==="
grep -i "error\|failed\|unable" /tmp/launch.log | head -10 || echo "No obvious errors in log"

echo ""
echo "=== Waiting for interrupt (Ctrl+C) ==="
wait $LAUNCH_PID

kill $XVFB_PID 2>/dev/null || true
