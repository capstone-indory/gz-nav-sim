#!/bin/bash
set -e

cd /root/gz-nav-sim
source install/setup.bash

echo "Starting Xvfb virtual display..."
export DISPLAY=:99
Xvfb :99 -screen 0 1280x1024x24 -ac 2>/dev/null &
XVFB_PID=$!
sleep 2

echo "Display ready at DISPLAY=$DISPLAY (PID: $XVFB_PID)"
echo "Starting simulation..."
echo ""

trap "kill $XVFB_PID 2>/dev/null || true" EXIT

# Run the launch with proper display
ros2 launch src/gz_nav_sim/launch/sim_nav.launch.py \
  headless:=false \
  use_foxglove:=true \
  use_da3:=true \
  use_elevator:=false

