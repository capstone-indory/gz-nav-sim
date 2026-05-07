# gz-nav-sim

Gazebo Classic 11 + ROS 2 Humble navigation simulation with lidar SLAM,
D456 RGB-D mapping, Foxglove visualization, semantic OCR, and optional
semantic VLM inspection.

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
colcon build --symlink-install
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
  headless:=false \
  use_foxglove:=true \
  robot_model:=robot_d456 \
  direct_depth:=true \
  use_da3:=false \
  use_nvblox:=true \
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

D456 publishes RGB on `/camera/image_raw`, depth on `/d456/depth/image_raw`,
and depth point cloud on `/d456/depth/points`.

## Convenience Script

```bash
./run_d456_rgbd.sh
```

The script starts Xvfb, launches D456 RGB-D + nvblox + Foxglove, and enables
semantic OCR. Override OCR with `OCR_FRAME_INTERVAL`, `OCR_MIN_CONFIDENCE`,
`OCR_SCALES`, `OCR_FLOOR_HINT`, and `OCR_FLOOR_PRIOR_MODE`. Enable VLM
separately with `USE_SEMANTIC_VLM=true`.
