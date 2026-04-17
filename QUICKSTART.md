# GZ-Nav-Sim Quick Start Guide

## Overview
This is a ROS 2 Humble + Gazebo Classic 11 simulation featuring:
- **Gazebo Classic 11** physics engine with accurate sensor simulation
- **Nav2** for autonomous navigation and path planning
- **SLAM Toolbox** for simultaneous localization and mapping
- **Depth-Anything-3 (DA3)** for monocular depth estimation
- **Multi-building world**: Office + Hospital (merged via translation/rotation)

## Prerequisites

### System Requirements
- Ubuntu 22.04 (Jammy)
- ROS 2 Humble
- Gazebo Classic 11
- Python 3.10+
- NVIDIA GPU (recommended for DA3 depth inference)

### Install Dependencies
```bash
# Install ROS 2 Humble and Gazebo Classic 11
sudo apt-get update
sudo apt-get install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-gazebo-msgs \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-tf2-ros \
  xvfb

# Install virtual display support (required for headless camera rendering)
sudo apt-get install -y xvfb xauth mesa-utils

# Install Python 3 build tools
pip3 install --upgrade pip setuptools wheel
```

### Build the Workspace
```bash
cd ~/gz-nav-sim
colcon build --symlink-install
```

## Running the Simulation

### Option 1: With Virtual Display (Recommended for Headless Servers)
```bash
# Set up virtual display and run simulation
cd ~/gz-nav-sim
bash run_stable.sh
```

**Topics Available:**
- `/front_camera/image_raw` - RGB camera feed (640x480 @ 10Hz)
- `/front_camera/camera_info` - Camera calibration info
- `/front_camera/depth/image_raw` - Depth map from DA3 (320x240 @ inferred)
- `/front_camera/depth/points` - Point cloud from depth
- `/scan` - 2D LiDAR scan (500 samples, 12m range, 10Hz)
- `/odom` - Robot odometry
- `/map` - SLAM-generated map
- `/tf` - Transform frames

### Option 2: With X11 Display (For Local Desktop)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

export DISPLAY=:0  # Or your actual X display
ros2 launch src/gz_nav_sim/launch/sim_nav.launch.py \
  headless:=false \
  use_da3:=true \
  use_elevator:=false
```

### Option 3: Headless Mode (CLI Only)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

export DISPLAY=:99
Xvfb :99 -screen 0 1280x1024x24 -ac &
ros2 launch src/gz_nav_sim/launch/sim_nav.launch.py \
  headless:=true \
  use_da3:=true \
  use_elevator:=false \
  use_foxglove:=false
```

## Key Configuration

### Simulation Parameters
```bash
# Launch arguments:
headless:=false              # GUI on/off (requires DISPLAY)
use_da3:=true                # Enable DA3 depth inference
use_elevator:=false          # Enable elevator teleportation
use_foxglove:=false          # Enable Foxglove visualization bridge
```

### Robot Specs
- **Body**: 0.36m diameter cylinder, 0.12m height
- **Wheels**: Differential drive, 0.38m apart, 0.16m diameter
- **LiDAR**: RPLIDAR C1 equivalent (500 samples, 0-12m range)
- **Camera**: Front RGB camera (640x480, 74° FOV)
- **Casters**: Passive front and rear casters (0.08m diameter)

### World Layout
```
Office (origin rotated +90° CCW)
  ├─ Walls & furniture (from ServiceSim)
  ├─ Visual elevator at (0, 0, 0)
  └─ Robot spawns at (-3, 0)

Hospital (+150m on X-axis)
  ├─ Walls & medical equipment
  ├─ Physical elevator cabin
  └─ Elevator landing at (148.5, 19.3)
```

## Troubleshooting

### Camera Not Publishing
1. Verify Xvfb is running: `ps aux | grep Xvfb`
2. Check DISPLAY variable: `echo $DISPLAY`
3. Verify plugin: `find /opt/ros/humble -name "*camera*" -type f`

### DA3 Not Inferencing
1. Check GPU: `nvidia-smi` (if GPU available)
2. Check logs: `ros2 topic echo /front_camera/depth/points --once`
3. Verify model path: `ls -la ~/gz-nav-sim/src/Depth-Anything-3`

### Low FPS / Simulation Lag
1. Disable visualization in headless mode
2. Reduce camera resolution in robot.sdf
3. Disable secondary sensors (LiDAR, depth) if not needed

### Gazebo/ROS 2 Connection Issues
1. Rebuild: `colcon build --symlink-install`
2. Clear build cache: `rm -rf build install log`
3. Re-source environment: `source install/setup.bash`

## Navigation with Nav2

Once simulation is running, you can use Nav2 for autonomous navigation:

```bash
# In another terminal:
source install/setup.bash

# Send a navigation goal via CLI (x, y, theta)
ros2 action send_goal nav_to_pose nav2_msgs/action/NavigateToPose \
  '{ pose: { header: { frame_id: "map" }, pose: { position: { x: 5.0, y: 5.0, z: 0 }, orientation: { w: 1.0 } } } }'

# Or use RViz for interactive goal selection:
ros2 run rviz2 rviz2 -c src/gz_nav_sim/config/nav2_rviz.yaml
```

## File Structure
```
gz-nav-sim/
├── src/gz_nav_sim/
│   ├── launch/
│   │   └── sim_nav.launch.py          # Main launch file
│   ├── models/
│   │   ├── robot/
│   │   │   ├── robot.sdf              # Robot definition
│   │   │   └── model.config
│   │   ├── office/                    # Office building models
│   │   └── hospital/                  # Hospital building models
│   ├── worlds/
│   │   ├── office.world               # Office world
│   │   ├── hospital.world             # Hospital world
│   │   └── combined.world             # Merged world (generated)
│   ├── config/
│   │   ├── nav2_params.yaml
│   │   ├── slam_params.yaml
│   │   ├── da3_params.yaml
│   │   └── building_manifest.json     # Building layout metadata
│   ├── scripts/
│   │   ├── elevator_teleport.py       # Elevator teleportation node
│   │   ├── merge_worlds.py            # World merge script
│   │   └── da3_depth_node.py          # DA3 depth inference wrapper
│   └── package.xml
├── run_stable.sh                      # Stable startup script
├── run_sim.sh                         # Full-featured startup script
├── test_camera.sh                     # Camera test script
├── test_da3.sh                        # DA3 test script
└── QUICKSTART.md                      # This file
```

## Advanced: Running World Generation

If you modify office.world or hospital.world, regenerate the merged world:

```bash
cd ~/gz-nav-sim
python3 src/gz_nav_sim/scripts/merge_worlds.py
```

This creates:
- `src/gz_nav_sim/worlds/combined.world` (merged SDF)
- `src/gz_nav_sim/config/building_manifest.json` (model layout metadata)

## Contact
Maintainer: Indoory (dev@indoory.io)
License: Apache-2.0
