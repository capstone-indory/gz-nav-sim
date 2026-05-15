# XLeRobot Hospital Isaac v2 Quick Start

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- `rosbridge_server` on the ROS PC
- Isaac Sim app running from `/home/indory/isaacsim/user_projects/xlerobot_hospital`

## ROS PC

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090
```

## Isaac Machine

```bash
cd /home/indory/isaacsim
./python.sh /home/indory/isaacsim/user_projects/xlerobot_hospital/scripts/launch_streaming.py \
  --public-endpoint 100.73.67.97 \
  --stream-port 49100 \
  --rosbridge-host ROS_PC_IP \
  --rosbridge-port 9090 \
  --no-keyboard
```

## Navigation Stack

```bash
cd ~/gz-nav-sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --paths src/gz_nav_sim
source install/setup.bash
./run_multisession_slam.sh
```

The stack consumes `/xlerobot/*` topics and republishes the local navigation
interface: `/odom`, `/scan`, `/camera/image_raw`, `/camera/camera_info`, and
`/d456/depth/*`.
