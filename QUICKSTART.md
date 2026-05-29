# XLeRobot Hospital Isaac v2 Quick Start

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- `rosbridge_server` on the ROS PC
- Isaac Sim app running from `/home/indory/isaacsim/user_projects/xlerobot_hospital`

## ROS PC

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090 bson_only_mode:=false
```

Transport contract:

- Use JSON rosbridge only for control/state/lightweight topics.
- Isaac and hardware publish the same narrow `/xlerobot/*` contract into the
  compute PC. The local bridge republishes `/scan`; RTAB creates the local
  motion estimate from RGB-D instead of using robot/base odometry.
- Hardware RGB-D for SLAM uses the TCP binary side channel, then the compute PC
  republishes `/camera/image_raw`, `/camera/camera_info`, `/depth/image_raw`,
  `/depth/camera_info`, `/imu/data`, and `/depth/points` for RTAB/nvblox
  consumers.
- The 관제 web UI receives video through RTSP/H.264 plus WebRTC, not rosbridge
  image JSON.

## Isaac Machine

```bash
cd /home/indory/isaacsim
./python.sh /home/indory/isaacsim/user_projects/xlerobot_hospital/scripts/launch_streaming.py \
  --public-endpoint 100.73.67.97 \
  --stream-port 49100 \
  --rosbridge-host ROS_PC_IP \
  --rosbridge-port 9090 \
  --rosbridge-wire-format json \
  --no-keyboard
```

Isaac publishes front RGB and depth as compressed ROS image data. The browser
must not connect to Isaac directly. The compute bridge restores local algorithm
topics when full-feature nodes need them, and the web adapter converts the front
RGB compressed stream to browser video for the 관제 UI.

Future ROS/web H.264 camera encoding on Ubuntu needs GStreamer plugins. Install once:

```bash
scripts/install_h264_video_plugins.sh
gst-inspect-1.0 x264enc h264parse mpegtsmux hlssink
```

These plugins are for the ROS/web adapter encoder path, not for an Isaac HTTP
side-channel.

## Navigation Stack

```bash
cd ~/gz-nav-sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --paths src/gz_nav_sim
source install/setup.bash
./run_multisession_slam.sh
```

The stack consumes the narrow `/xlerobot/*` hardware contract and republishes
the local navigation interface: `/scan`, `/camera/image_raw/compressed`,
`/camera/image_raw`, `/depth/image_raw`, `/depth/camera_info`, and the RTAB
RGB-D motion estimate when the corresponding features are enabled.

The default Isaac run requests OCR, depth, 3D hooks, and RTAB-Map. RTAB-Map is a
hard requirement when requested; nvblox remains optional and is skipped with a
launch log message until installed.

For the real robot split setup, run the robot I/O script on the robot computer
and the hardware mode on this compute PC. Hardware mode starts SLAM/Nav/Foxglove
and the web stack by default:

```bash
# compute PC
./run_multisession_slam.sh hardware
```

Hardware mode autostarts the robot computer over SSH by default. The built-in
default target is the `RasberryPi` alias and it falls back to `pi@lekiwi`; if
`~/.ssh/indory_RasberryPi_ed25519` exists, the launcher passes it to ssh. For
manual Pi startup, use
`ROBOT_SSH_AUTOSTART=0 ./run_multisession_slam.sh hardware`.

For a lightweight navigation-only compute run, use:

```bash
./run_multisession_slam.sh hardware --no-web
```

For a local USB RPLIDAR bench test where the lidar is plugged into this compute
PC, use `local-lidar` instead:

```bash
sudo apt install python3-serial
HARDWARE_LIDAR_SERIAL=/dev/ttyUSB0 ./run_multisession_slam.sh local-lidar
```

## Real Robot, Lightweight I/O

For the real XLeRobot, keep the Raspberry Pi/onboard computer light. Run only
base, lidar, and optional compressed camera I/O there. Run ROS 2,
rosbridge_server, SLAM, Nav2, Foxglove, and the web stack on the compute PC.

Do not run the robot I/O script on the compute PC unless the motor controller,
lidar, and camera are physically plugged into that same machine. In the intended
split setup, the Pi uses its `~/indoory_ros` runtime and connects back to this
computer's rosbridge websocket.

On the robot computer:

```bash
cd ~/indoory_ros
# edit robot/xlerobot_robot_io.env for lidar/camera/base device settings
COMPUTE_PC_HOST=<compute-pc-ip> PUBLISH_TF=0 scripts/start_pi_bridge_stack.sh
```

On the compute PC:

```bash
cd ~/gz-nav-sim
scripts/setup_compute_pc_hardware_conda.sh
micromamba run -n gz-nav-humble ./run_multisession_slam.sh hardware
```

The compute launcher starts `rosbridge_server`, detects the compute PC IP that
the Pi should use, then SSH-starts `~/indoory_ros/scripts/start_pi_bridge_stack.sh`
on the Pi. It also starts the web stack by default; add `--no-web` or set
`HARDWARE_WITH_WEB=0` for the older SLAM/Nav/Foxglove-only behavior.

The Pi does not need ROS_DOMAIN_ID because it does not use DDS. It connects to
`ws://<compute-pc-ip>:9090` for commands, scan, status, and optional IMU, while
RGB-D frames go through the TCP binary side channel. The compute PC sends
`/xlerobot/cmd_vel` back. Hardware SLAM uses RTAB RGB-D motion estimation
instead of Pi wheel/base odometry.

For depth sensor on the robot computer, use:

```bash
ENABLE_DEPTH_SENSOR=true
DEPTH_SENSOR_DEPTH_TOPIC=/xlerobot/head_camera/depth/image
DEPTH_SENSOR_DEPTH_CAMERA_INFO_TOPIC=/xlerobot/head_camera/depth/camera_info
DEPTH_SENSOR_IMU_TOPIC=/xlerobot/head_camera/imu
DEPTH_SENSOR_ENABLE_COLOR=true
```

The compute PC normalizes those streams to `/depth/image_raw`,
`/depth/camera_info`, `/depth/points`, and `/imu/data`. Color preview is
still exposed as `/camera/image_raw/compressed` for the web adapter.

Hardware SLAM uses RGB-D motion estimation by default. `/xlerobot/scan` becomes
`/scan_raw`, then filtered `/scan` feeds Nav2 and filtered `/scan_slam` feeds
RTAB grid mapping. Returns under 20 cm are dropped in both scan paths.

Nav goals can be sent three ways:

```bash
# Foxglove: publish PoseStamped to /goal_pose
./bench/drive.sh nav2 --x 1.0 --y 0.0 --yaw 0.0
./bench/drive.sh dest --name home
```

Named destinations live in `src/gz_nav_sim/config/nav_destinations.yaml` and
arrive on `/nav/destination`. Direct compact goals can also be published to
`/nav/goal_pose2d`.

The compute PC command above is the normal web-connected hardware path. The
same remote `/xlerobot/*` robot I/O contract is used whether the web stack is
enabled or disabled.
