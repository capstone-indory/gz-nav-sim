#!/bin/bash
set -e

cd /root/gz-nav-sim
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "════════════════════════════════════════════════════════════"
echo "Starting Gazebo Sim + Nav2 + SLAM + DA3 Depth Inference"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "Topics available once simulation starts:"
echo "  Camera:  /front_camera/image_raw"
echo "  Depth:   /front_camera/depth/points"
echo "  LiDAR:   /scan"
echo "  Odometry:/odom"
echo ""
echo "Press Ctrl+C to stop"
echo "════════════════════════════════════════════════════════════"
echo ""

# Clean up any stale X sockets
rm -f /tmp/.X11-unix/99 /tmp/.X99-lock 2>/dev/null || true

# Start Xvfb as daemon (standard Docker pattern)
export DISPLAY=:99
echo "[1/2] Starting Xvfb virtual display..."
Xvfb :99 -screen 0 1280x1024x24 &
XVFB_PID=$!

# Wait for Xvfb to initialize
sleep 2

# Verify Xvfb is running
if ! ps -p $XVFB_PID > /dev/null; then
    echo "[✗] Xvfb failed to start"
    exit 1
fi
echo "[✓] Xvfb running (PID: $XVFB_PID)"

# Trap to cleanup on exit
trap "kill $XVFB_PID 2>/dev/null || true; pkill -f 'Xvfb.*:99' 2>/dev/null || true" EXIT

echo "[2/2] Starting ROS 2 simulation with GPU acceleration (VirtualGL)..."

# Run ROS 2 launch with VirtualGL (routes OpenGL calls to GPU via EGL)
vglrun -d egl0 ros2 launch src/gz_nav_sim/launch/sim_nav.launch.py \
  headless:=false \
  use_foxglove:=true \
  use_da3:=true \
  use_elevator:=false
