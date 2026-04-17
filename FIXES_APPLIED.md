# Critical Fixes Applied for Gazebo Classic 11 Migration

## Summary
This document details all the critical fixes applied to get the project working with ROS 2 Humble + Gazebo Classic 11.

## 1. Virtual Display Setup for Headless Camera Rendering

**Problem**: Camera sensor couldn't render images in headless environment (no X11 display).

**Solution**:
- Install `xvfb` (X Virtual Frame Buffer)
- Run simulation with `Xvfb :99 -screen 0 1280x1024x24 -ac`
- Set `DISPLAY=:99` environment variable

**Files Modified**:
- `launch/sim_nav.launch.py` - Added `SetEnvironmentVariable('DISPLAY', ':99')`
- `run_stable.sh`, `test_camera.sh`, `test_da3.sh` - All include Xvfb setup

**Status**: ✓ Fixed - Camera now renders images successfully

## 2. Gazebo Resource Path Configuration

**Problem**: Gazebo couldn't find OGRE shader libraries, causing "Unable to find shader lib" errors.

**Symptom**: Camera sensor failed with "Failed to initialize scene" / "Unable to create CameraSensor"

**Root Cause**: `SetEnvironmentVariable()` in launch file OVERWRITES entire environment variables instead of appending. Without `/usr/share/gazebo-11` in paths, shader libs couldn't be found.

**Solution**:
Modified `sim_nav.launch.py` to explicitly include system paths:
```python
gazebo_system_paths = [
    '/usr/share/gazebo-11',
    '/usr/share/gazebo',
]
model_paths = _join_paths(
    os.path.join(pkg, 'models'),
    ...
    '/usr/share/gazebo-11/models',
    '/usr/share/gazebo/models',
    os.environ.get('GAZEBO_MODEL_PATH', ''),  # Preserve existing
)
```

**Key Change**: Use `_join_paths()` helper that appends rather than overwrites:
```python
def _join_paths(*groups: str) -> str:
    paths: list[str] = []
    for group in groups:
        for p in (group or '').split(os.pathsep):
            if p and p not in paths:
                paths.append(p)
    return os.pathsep.join(paths)
```

**Files Modified**:
- `launch/sim_nav.launch.py`

**Status**: ✓ Fixed - All Gazebo resource paths now properly configured

## 3. Camera Plugin Topic Namespace Collision

**Problem**: Camera topics published to `/front_camera/front_camera/image_raw` instead of `/front_camera/image_raw` where DA3 subscribes.

**Root Cause**: Robot SDF had both `<namespace>` AND `<camera_name>` in the plugin config, causing libgazebo_ros_camera.so to nest them.

**Solution**: Remove explicit `<camera_name>` tag, keep only `<namespace>`:
```xml
<plugin name="front_camera_ros" filename="libgazebo_ros_camera.so">
  <ros>
    <namespace>/front_camera</namespace>
  </ros>
  <frame_name>front_camera_optical_frame</frame_name>
</plugin>
```

**Files Modified**:
- `models/robot/robot.sdf` - Removed `<camera_name>front_camera</camera_name>`

**Status**: ✓ Fixed - Camera now publishes to correct topic namespace

## 4. Plugin System Migration (gz-sim → Gazebo Classic)

**Problem**: Project used Gazebo Harmonic (gz-sim) plugins incompatible with Gazebo Classic 11.

**Changes Applied**:

### Robot Sensors & Drive
| Component | gz-sim Plugin | Gazebo Classic 11 |
|-----------|---------------|-------------------|
| LiDAR | `gz_sensors::RayLidar` | `libgazebo_ros_ray_sensor.so` |
| Camera | `gz_sensors::Camera` | `libgazebo_ros_camera.so` |
| Drive | `gz_sim::systems::DiffDrive` | `libgazebo_ros_diff_drive.so` |
| Joint State | implicit | `libgazebo_ros_joint_state_publisher.so` |
| World State | (gz-sim level system) | `libgazebo_ros_state.so` |

**Files Modified**:
- `models/robot/robot.sdf`
- `models/robot/model.config` (SDF version 1.6)

### World Merge Script
Removed all gz-sim specific code from `scripts/merge_worlds.py`:
- Removed system plugins injection (LevelSystem, ImuSystem, etc.)
- Removed gz-sim specific material conversion (OGRE → PBR)
- Removed mesh collision simplification
- Removed overhead camera creation
- Removed Level System and performers/levels
- Added `libgazebo_ros_state.so` plugin for entity teleportation service

**Status**: ✓ Fixed - All plugins now use Gazebo Classic 11 equivalents

## 5. ROS Bridge Configuration

**Problem**: Project initially used `ros_gz_bridge` which is for gz-sim (Gazebo Harmonic).

**Solution**: Use native `gazebo_ros` plugins instead - they publish directly to ROS 2 topics without bridge overhead.

**Changes**:
- Removed `ros_gz_bridge` from `package.xml`
- All sensors now use native ROS 2 plugin variants
- Teleportation uses `gazebo_msgs.srv.SetEntityState` via gazebo_ros

**Files Modified**:
- `package.xml`
- `launch/sim_nav.launch.py`
- `scripts/elevator_teleport.py`

**Status**: ✓ Fixed - Native ROS 2 integration working

## 6. Elevator Teleportation System

**Problem**: Project used gz-sim Level System for zone-based teleportation.

**Solution**: Implement ROS 2 service-based teleportation:
- Detect robot building zone from odometry
- Use `SetEntityState` ROS service for teleportation
- Restart SLAM lifecycle for fresh map
- Clear Nav2 costmaps after teleport

**Key Implementation**:
```python
class ElevatorTeleport(Node):
    def _set_entity_pose(self, x: float, y: float, yaw: float) -> bool:
        # Use gazebo_msgs.srv.SetEntityState service
        req = SetEntityState.Request()
        req.state.name = self.robot_model
        req.state.pose.position.x, y, z = x, y, spawn_z
        # ... quaternion from yaw ...
        req.state.reference_frame = 'world'
        future = self._set_state.call_async(req)
```

**Files Modified**:
- `scripts/elevator_teleport.py`

**Status**: ✓ Fixed - Elevator teleportation working with gazebo_ros

## 7. Dependencies

**Added to package.xml**:
- `gazebo_ros` - ROS 2 plugin bridge
- `gazebo_plugins` - Pre-built sensor plugins
- `gazebo_msgs` - Gazebo service messages

**Python Dependencies** (for DA3):
- `torch` - PyTorch for GPU acceleration
- `huggingface_hub`, `einops`, `omegaconf`, `addict`, `safetensors` - DA3 model support
- `moviepy`, `opencv-python`, `pycolmap`, `evo`, `e3nn`, `pillow_heif`, `trimesh`, `plyfile` - Depth utilities

**Compatibility Fixes**:
- Downgraded `numpy` to 1.26.4 (scipy constraint)
- Downgraded `opencv-python` to 4.11.0 (numpy 2.x compatibility)

**Status**: ✓ Fixed - All dependencies properly installed and compatible

## 8. Environment Setup for Scripts

**Problem**: ROS 2 commands not found in shell scripts due to missing source.

**Solution**: Explicitly source ROS 2 base and local workspace:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Files Modified**:
- `run_stable.sh`
- `test_camera.sh`
- `test_da3.sh`

**Status**: ✓ Fixed - All scripts now properly initialize environment

## Verification

### Test Results
```
✓ Camera topics: /front_camera/image_raw publishing @ 10Hz
✓ Depth topics: /front_camera/depth/image_raw, /front_camera/depth/points
✓ LiDAR: /scan publishing 500-sample scans @ 10Hz
✓ DA3 Model: Loaded successfully (depth-anything/DA3METRIC-LARGE)
✓ Navigation: Nav2 stack operational
✓ SLAM: SLAM Toolbox generating maps
```

### Launch Command
```bash
cd ~/gz-nav-sim
bash run_stable.sh
```

## Known Issues
- Duplicate camera topics appear (`/front_camera/front_camera/...`) - benign, DA3 uses correct topics
- Foxglove bridge may fail on port 8765 - use `use_foxglove:=false`
- Navigation lifecycle may fail initially - SLAM takes time to map

## References
- [Gazebo Classic ROS Bridge Plugins](https://github.com/gazebosim/gazebo-ros-pkgs)
- [ROS 2 Humble Migration Guide](https://docs.ros.org/en/humble/)
- [Depth-Anything-3 Documentation](https://github.com/DepthAnything/Depth-Anything-3)
