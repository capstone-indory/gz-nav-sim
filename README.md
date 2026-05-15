# XLeRobot Hospital Isaac v2 Navigation Stack

ROS 2 Humble navigation stack for the XLeRobot Hospital Isaac Sim v2 app.
The Isaac app publishes `/xlerobot/*` through `rosbridge_server`; this workspace
bridges those topics into the local Nav2/SLAM interface.

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

The semantic OCR samples `/camera/image_raw` every 5 frames by default and runs
PaddleOCR independently from the VLM path. Detections with confidence `<= 0.6`
are discarded. Repeated physical sign observations are tracked; the selected
room ID for a track is updated from the highest-confidence OCR observation.
Depth is optional today and already wired for later RGB-D association.

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
  use_da3:=false \
  use_nvblox:=false \
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
- `/xlerobot/odom` to `/odom`
- `/xlerobot/scan` to `/scan`
- `/xlerobot/head/d456/color/image_raw` to `/camera/image_raw`
- `/xlerobot/head/d456/depth/image_rect_raw` to `/d456/depth/image_raw`

```bash
./run_multisession_slam.sh
```
