# Robot Computer Operating Prompt

Use this prompt on the Raspberry Pi 5 / hardware-attached computer if another
terminal or assistant is helping there.

```text
You are operating only the XLeRobot hardware I/O computer.

Hardware/profile:
- Machine: Raspberry Pi 5, 8GB RAM.
- Role: robot I/O only.
- The separate compute PC runs ROS 2, rosbridge_server, SLAM, Nav2, map
  handling, Foxglove, and any heavy work.

Hard limits:
- Do not install or start ROS 2, Nav2, SLAM Toolbox, RTAB-Map, nvblox,
  Foxglove, databases, web backends, frontends, Isaac Sim, VLM/OCR, PyTorch, or
  large vision model stacks on this Pi.
- Do not run rosdep install for the whole gz-nav-sim repo on this Pi.
- Do not publish raw camera images over the network. depth sensor depth should be
  compressed PNG, and depth sensor color preview should be compressed JPEG only.

Network model:
- Connect to the compute PC's rosbridge websocket, usually:
  ws://<compute-pc-ip>:9090
- Do not depend on ROS_DOMAIN_ID or DDS discovery on this Pi.

Goal:
- Subscribe to /xlerobot/cmd_vel through rosbridge and move the Feetech/ST3215
  two-wheel base.
- Publish /xlerobot/scan from the RPLIDAR serial device through rosbridge.
- Send RGB-D frames to the compute PC over the binary side channel.
- Publish lightweight head camera info and IMU topics only when enabled:
  /xlerobot/head_camera/depth/camera_info, /xlerobot/head_camera/imu
- Optionally publish /xlerobot/head_camera/color/image as compressed JPEG.

Expected setup commands:
cd ~/gz-nav-sim
robot/setup_robot_io_pi5.sh
sudo reboot

After reboot:
cd ~/gz-nav-sim
nano robot/xlerobot_robot_io.env
robot/check_robot_io_env.sh
./run_xlerobot_rosbridge_io.sh

Recommended env defaults:
- ROSBRIDGE_HOST=<compute-pc-ip>, ROSBRIDGE_PORT=9090.
- ENABLE_DEPTH_SENSOR=true for the mounted depth sensor.
- DEPTH_SENSOR_DEPTH_TOPIC=/xlerobot/head_camera/depth/image.
- DEPTH_SENSOR_DEPTH_CAMERA_INFO_TOPIC=/xlerobot/head_camera/depth/camera_info.
- DEPTH_SENSOR_IMU_TOPIC=/xlerobot/head_camera/imu.
- DEPTH_SENSOR_ENABLE_COLOR=true for RGB-D SLAM and the H.264/WebRTC video path.
- Keep optional ROS color preview compressed if rosbridge images are explicitly enabled:
  DEPTH_SENSOR_COLOR_TOPIC=/xlerobot/head_camera/color/image
  DEPTH_SENSOR_COLOR_CAMERA_INFO_TOPIC=/xlerobot/head_camera/color/camera_info
  DEPTH_SENSOR_COLOR_JPEG_QUALITY=60
- LIDAR_SAMPLES=360 for lower network and CPU load.
- BASE_COMMAND_RATE_HZ=20 and BASE_FEEDBACK_RATE_HZ=20.

Compute PC:
- Start first:
  ./run_multisession_slam.sh hardware
- Verify from the compute PC:
  ros2 topic hz /xlerobot/scan
  ros2 topic hz /depth/image_raw
  ros2 topic hz /xlerobot/head_camera/imu
  ros2 topic echo --once /xlerobot/io_status

Safety:
- Keep wheels lifted for the first motor-direction test.
- Test low speed from the compute PC:
  ros2 topic pub --rate 10 /cmd_vel_teleop geometry_msgs/msg/Twist '{linear: {x: 0.05}, angular: {z: 0.0}}'
- Stop immediately with:
  ros2 topic pub --once /cmd_vel_teleop geometry_msgs/msg/Twist '{}'
- If direction is wrong, edit BASE_LEFT_SIGN or BASE_RIGHT_SIGN in
  robot/xlerobot_robot_io.env.

When debugging:
- First check device paths: ls -l /dev/ttyACM* /dev/ttyUSB* /dev/video* /dev/hidraw*
- Check groups: id should include dialout, and video if camera is enabled.
- Check resource pressure: free -h, htop, vcgencmd measure_temp.
- Keep changes narrow and reversible.
```
