# gz-nav-sim

Gazebo Classic 11 + ROS 2 Humble navigation simulation with lidar SLAM,
D456 RGB-D mapping, Foxglove visualization, and semantic VLM inspection.

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
  huggingface_hub transformers safetensors accelerate
colcon build --symlink-install
source install/setup.bash
```

The semantic VLM samples `/camera/image_raw` every 20 frames and uses
`/d456/depth/*` only to place repeated observations on the map. No separate OCR
engine is used.

## Run

```bash
ros2 launch gz_nav_sim sim_nav.launch.py \
  headless:=false \
  use_foxglove:=true \
  robot_model:=robot_d456 \
  direct_depth:=true \
  use_da3:=false \
  use_nvblox:=true \
  use_semantic_vlm:=true \
  vlm_model:=Qwen/Qwen2.5-VL-3B-Instruct \
  vlm_frame_interval:=20
```

The VLM publishes strict JSON detections and candidate/confirmed annotations:

- `/semantic_vlm/detections`
- `/semantic_vlm/markers`

Foxglove connects to `ws://localhost:8765`.

D456 publishes RGB on `/camera/image_raw`, depth on `/d456/depth/image_raw`,
and depth point cloud on `/d456/depth/points`.

## Convenience Script

```bash
./run_d456_rgbd.sh
```

The script starts Xvfb, launches D456 RGB-D + nvblox + Foxglove, and enables
the semantic VLM. Override the model or device with `VLM_MODEL` and
`VLM_DEVICE`.
