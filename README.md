# XLeRobot Hospital Isaac v2 Navigation Stack

ROS 2 Humble navigation stack for the XLeRobot Hospital Isaac Sim v2 app.
The Isaac app publishes `/xlerobot/*` through `rosbridge_server`; this workspace
bridges those topics into the local Nav2/SLAM interface.

## Network And Video Contract

- `rosbridge_server` is JSON and carries only control/state/lightweight topics.
- Robot control/state topics stay narrow: `/xlerobot/cmd_vel`,
  `/xlerobot/scan`, and lightweight status topics. Hardware RTAB mode does not
  bridge wheel/base odometry into `/odom`.
- Browser video uses the video side channel: Pi/Isaac H.264 RTSP into MediaMTX,
  then WebRTC to the web UI.
- RGB-D for SLAM uses the binary side channel in hardware mode and is normalized
  on the compute PC as `/camera/image_raw`, `/camera/image_raw/compressed`,
  `/camera/camera_info`, `/depth/image_raw`, and `/depth/camera_info`.
- Camera source names:
  - `/xlerobot/base_camera/*` is the LeKiwi base USB camera.
  - `/xlerobot/head_camera/*` is the head RGB-D camera.
- Head camera source contract:
  - `/xlerobot/head_camera/color/image`
    (`sensor_msgs/CompressedImage`, head RGB).
  - `/xlerobot/head_camera/depth/image`
    (`sensor_msgs/CompressedImage`, head depth PNG).
  - `/xlerobot/head_camera/imu`
    (`sensor_msgs/Imu`, head IMU).
- Depth point clouds and maps use `/depth/points` and related `/depth/*`
  topics. Human-facing video stays out of rosbridge.
- RTAB-Map is required when an RTAB preset is selected; the launcher fails fast
  if `rtabmap_odom`, `rtabmap_slam`, or `rtabmap_msgs` is missing. nvblox remains
  optional and is skipped with a launch log when unavailable.

This keeps heavy camera payloads off rosbridge and off the browser websocket
path while keeping the camera available for SLAM and the control UI.

## Setup

```bash
cd ~/gz-nav-sim
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get update
sudo apt-get install -y python3-pip
python3 -m pip install --upgrade pip setuptools wheel
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install \
  numpy opencv-python pillow \
  paddleocr pytesseract \
  huggingface_hub transformers safetensors accelerate
colcon build --symlink-install --paths src/gz_nav_sim
source install/setup.bash
```

The default Isaac preset is the full feature path: front RGB, depth, OCR,
depth/point-cloud generation, 3D visualization hooks, and RTAB-Map requested by
default. RTAB-Map packages must be installed or built for that preset; nvblox is
optional and is skipped with a launch log when unavailable.

OCR samples `/camera/image_raw`, runs PaddleOCR independently from the VLM path,
and publishes detections under `/semantic_ocr/*`.

PaddleOCR is the primary OCR backend because it returns recognized text,
confidence, and quadrilateral boxes in one pass, supports angle classification
for tilted hallway text, and works with multi-scale RGB frames. Tesseract is a
fallback if PaddleOCR is unavailable.

## Run

```bash
ros2 launch gz_nav_sim sim_nav.launch.py \
  isaac_transport:=rosbridge_v2 \
  use_foxglove:=true \
  direct_depth:=true \
  use_da3:=true \
  use_nvblox:=true \
  use_rtabmap:=true \
  use_slam_toolbox:=false \
  use_semantic_ocr:=true \
  ocr_frame_interval:=5 \
  ocr_min_confidence:=0.6 \
  use_semantic_vlm:=false
```

The OCR publishes strict JSON detections and candidate/confirmed annotations:

- `/semantic_ocr/detections`
- `/semantic_ocr/markers`
- `/semantic_ocr/image_annotations`

The VLM remains separate and can be enabled with `use_semantic_vlm:=true`; its
outputs are not unioned with OCR:

- `/semantic_vlm/detections`
- `/semantic_vlm/markers`
- `/semantic_vlm/image_annotations`

Foxglove connects to `ws://localhost:8765`.

The v2 bridge maps:

- `/xlerobot/cmd_vel` from `/cmd_vel_mux`
- `/xlerobot/scan` to `/scan`
- front compressed RGB to `/camera/image_raw/compressed`
- raw RGB for algorithms to `/camera/image_raw` when enabled
- depth sensor depth to `/depth/image_raw` and `/depth/points`
- depth sensor IMU to `/imu/data`

Isaac must not expose a separate browser-facing camera endpoint. Isaac sends
compressed sensor data into ROS; the ROS/web adapter turns the front camera into
browser video for the 관제 UI.

```bash
./run_multisession_slam.sh
```

Remote hardware mode expects the robot computer to connect to this compute PC's
rosbridge websocket and exchange `/xlerobot/*` messages. The compute PC runs
SLAM/Nav2/Foxglove plus the web stack, and publishes `/xlerobot/cmd_vel`:

```bash
./run_multisession_slam.sh hardware
```

Use `./run_multisession_slam.sh hardware --no-web` or `HARDWARE_WITH_WEB=0`
when you only want the lightweight SLAM/Nav/Foxglove stack.

By default, hardware mode now starts the Pi-side I/O agent over SSH after the
compute PC rosbridge is ready. The default target mirrors:

```sshconfig
Host RasberryPi
  HostName lekiwi
  User pi
  IdentityFile ~/.ssh/indory_RasberryPi_ed25519
```

The launcher uses `ROBOT_SSH_TARGET=RasberryPi` by default, falls back to
`pi@lekiwi` if that alias is not installed locally, auto-detects the compute PC
address reachable from the Pi, and starts
`~/indoory_ros/scripts/start_pi_bridge_stack.sh`. It passes the websocket as
`ROSBRIDGE_URL=ws://<compute-pc-ip>:9090` and leaves `/tf` ownership on the
compute PC bridge. It will try
`ROBOT_SSH_IDENTITY=~/.ssh/indory_RasberryPi_ed25519` when that key exists.
Useful overrides:

- `COMPUTE_ROSBRIDGE_HOST=<ip>` if auto-detection chooses the wrong interface.
- `ROBOT_SSH_AUTOSTART=0` to run the Pi command manually.
- `ROBOT_STOP_PI_LOCAL_WEB=0` if you intentionally want to keep the Pi-local
  teleoperation web process running, even though it may occupy lidar/camera.
- `ROBOT_REMOTE_REPO=~/gz-nav-sim ROBOT_IO_REMOTE_COMMAND=run_xlerobot_rosbridge_io.sh`
  to use the older robot-side runtime from this repo.
- `ROBOT_SSH_REQUIRED=1` if SSH autostart failure should abort the whole launch.

Local USB RPLIDAR bench mode is still available when the lidar is plugged into
this compute PC:

```bash
sudo apt install python3-serial
HARDWARE_LIDAR_SERIAL=/dev/ttyUSB0 ./run_multisession_slam.sh local-lidar
```

Useful hardware tuning environment variables:

- `HARDWARE_LIDAR_BAUD`, default `460800`
- `HARDWARE_LIDAR_ANGLE_OFFSET_DEG`, default `0.0`
- `HARDWARE_LIDAR_INVERT`, default `false`

## Real Robot Split Runtime

Use this when the Raspberry Pi should only move the base and read sensors while
the compute PC runs SLAM/Nav2/Foxglove and, by default, the web control stack.
The Pi still avoids Postgres, Spring, frontend, ROS 2, VLM, and other heavy
services.

The hardware I/O command is for the robot computer, not for the compute PC. The
compute PC should run the navigation stack and rosbridge_server; the Pi connects
to that websocket with a small Python client.

Compute PC:

```bash
cd ~/gz-nav-sim
scripts/setup_compute_pc_hardware_conda.sh
micromamba run -n gz-nav-humble ./run_multisession_slam.sh hardware
```

For compute-side navigation without the web UI:

```bash
micromamba run -n gz-nav-humble ./run_multisession_slam.sh hardware --no-web
```

Robot computer:

```bash
cd ~/indoory_ros
# edit robot/xlerobot_robot_io.env for lidar/depth sensor/base device settings
COMPUTE_PC_HOST=<compute-pc-ip> PUBLISH_TF=0 scripts/start_pi_bridge_stack.sh
```

That manual robot-computer command is still useful for debugging, but the normal
hardware launcher starts it automatically over SSH and overrides the rosbridge
URL at runtime. Device settings such as lidar serial, base serial, and depth sensor
options can stay in `~/indoory_ros/robot/xlerobot_robot_io.env` on the Pi.

The robot-side topics are intentionally narrow:

- Subscribe: `/xlerobot/cmd_vel`
- Publish: `/xlerobot/scan`, `/xlerobot/head_camera/depth/image`,
  `/xlerobot/head_camera/depth/camera_info`, and `/xlerobot/head_camera/imu`
- Optional color preview for ROS consumers: `/xlerobot/head_camera/color/image`
  and `/xlerobot/head_camera/color/camera_info`; the browser video path is
  RTSP/H.264 plus WebRTC, not rosbridge image JSON
- Compute-side normalized topics: `/depth/image_raw`,
  `/depth/camera_info`, `/depth/points`, and `/imu/data`

depth sensor env on the robot computer:

```bash
ENABLE_DEPTH_SENSOR=true
DEPTH_SENSOR_DEPTH_TOPIC=/xlerobot/head_camera/depth/image
DEPTH_SENSOR_DEPTH_CAMERA_INFO_TOPIC=/xlerobot/head_camera/depth/camera_info
DEPTH_SENSOR_IMU_TOPIC=/xlerobot/head_camera/imu
DEPTH_SENSOR_ENABLE_COLOR=true
DEPTH_SENSOR_COLOR_TOPIC=/xlerobot/head_camera/color/image
DEPTH_SENSOR_COLOR_CAMERA_INFO_TOPIC=/xlerobot/head_camera/color/camera_info
```

In hardware mode the default SLAM path uses RTAB-Map RGB-D motion estimation
instead of Pi wheel/base odometry. `/xlerobot/scan` is exposed as `/scan_raw`,
then filtered into `/scan` for Nav2 obstacle avoidance and `/scan_slam` for the
RTAB grid. Depth sensor frames arrive over the binary RGB-D side channel and are
decoded on the compute PC into `/depth/image_raw` and `/depth/points`; IMU is
bridged to `/imu/data` only when the mounted depth sensor exposes it.

Nav2 goals are available through Foxglove `/goal_pose`, direct coordinates, or
named destinations:

```bash
./bench/drive.sh nav2 --x 1.0 --y 0.0 --yaw 0.0
./bench/drive.sh dest --name home
ros2 topic pub --once /nav/goal_pose2d geometry_msgs/msg/Pose2D "{x: 1.0, y: 0.0, theta: 0.0}"
```

Edit `src/gz_nav_sim/config/nav_destinations.yaml` for map-specific names, or
launch with `NAV_DESTINATIONS_FILE=/path/to/goals.yaml ./run_multisession_slam.sh hardware`.

`run_multisession_slam.sh hardware` now uses this same remote robot I/O
contract. The old local USB lidar behavior is available as
`run_multisession_slam.sh local-lidar`.
