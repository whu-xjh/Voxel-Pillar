# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is **fast_livo** (package name), a high-speed LiDAR-Inertial Odometry system. It supports multi-LiDAR fusion, visual-inertial odometry, and external IMU integration optimized for high-speed scenarios (>5 m/s). The system features 250Hz IMU propagation and efficient voxel-based mapping with LRU caching.

**Recent Updates** (2025-01 to 2026-03):
- Added pillar voxel ground detection system with configurable adjacency filtering and iterative plane fitting
- Function renaming: `PerformInitialGroundDetection` → `GroundDetection`, `PerformAdjacentGroundDetection` → `AdjacentCheck`, `RefineGroundDetectionByPlaneFitting` → `PlaneFitting`, `UpdatePointGroundFlags` → `UpdateFlags`, `PublishPillarGroundPoints` → `PublishPillarPoints`
- All pillar voxel functions are sequential (no parallelization)
- Ground/isolated points excluded from ICP optimization to improve mapping accuracy

## Build System

This is a ROS Catkin workspace. Build from the catkin workspace root:

```bash
cd /home/xjh/Doc/highspeed-lio/catkin_ws
catkin_make
source devel/setup.bash
```

**Note**: The workspace path is hardcoded in build commands. If your workspace is located elsewhere, adjust paths accordingly.

For development with debugging:
```bash
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

**Architecture-specific optimizations** (CMakeLists.txt:21-37):
- ARM (32/64-bit): `-O3 -mcpu=native -mtune=native` with NEON support for 32-bit
- x86-64: `-O2 -march=native` (conservative to avoid compiler crashes)
- Multi-threading: Auto-configured based on CPU core count (`MP_EN`, `MP_PROC_NUM`)
- Debug builds: `-O0 -g`

**Dependencies**: PCL (≥1.8), Eigen3 (≥3.3.4), OpenCV, Sophus, Boost, vikit_common/vikit_ros. Optional: mimalloc, OpenMP, LAStools.

## Core Architecture

### System Components

**Executables:**
- `fastlivo_mapping`: Main mapping node (src/main.cpp)
- `livox_multi_lidar`: Multi-LiDAR data merger (fuses 3 Livox LiDARs)

**Libraries:**
- `laser_mapping`: Core LiDAR-IMU fusion (src/LIVMapper.cpp)
- `imu_proc`: 250Hz IMU propagation with bias estimation (src/IMU_Processing.cpp)
- `imu_filter`: IMU data filtering and preprocessing (src/imu_filter.cpp)
- `lio`: Voxel octree map with LRU caching (src/voxel_map.cpp)
- `vio`: Visual-inertial odometry with feature tracking (src/vio.cpp, src/frame.cpp, src/visual_point.cpp)
- `pre`: Point cloud preprocessing and filtering (src/preprocess.cpp)

### Data Flow

1. **Multi-LiDAR Merger** (`livox_multi_lidar`) → `/livox/multi_lidar`
2. **Preprocessing** (`pre`) filters and voxelizes point clouds
3. **IMU Processing** (`imu_proc`) propagates at 250Hz, estimates biases, supports external IMU fusion
4. **LIO/VIO Fusion** (`laser_mapping`) performs state estimation
5. **Voxel Mapping** (`lio`) maintains octree map with LRU caching (configurable via `lio/capacity`)

**State Management**: The `StatesGroup` struct (common_lib.h:144-199) maintains 19-dimensional state including rotation, position, velocity, IMU biases, gravity, and exposure time with covariance.

### External IMU Integration

**Key Files**: include/LIVMapper.h:45-60, include/IMU_Processing.h:48-51, include/common_lib.h:132-142

**External IMU Data Structure** (`ExternalIMUData`):
- Position, linear velocity, orientation (Euler angles), velocity covariance
- Interpolated from odom messages via `findClosestExternalIMUs()` and `interpolateExternalIMU()`

**Configuration** (config/livox_multi_lidar.yaml:111-119):
- `enable`: Enable external IMU fusion
- `external_imu_init_frame`: Frames for external IMU initialization (default: 30)
- `external_imu_only`: Use only external IMU (default: false)
- `external_R`, `external_T`: Extrinsic calibration between IMUs
- `buffer_size`: External IMU buffer size

**Initialization**: External IMU initialized separately from internal IMU with `MAX_EXTERNAL_INI_COUNT=20` iterations vs `MAX_INI_COUNT=20` for internal (IMU_Processing.h:85).

### Voxel Map System

**Voxel Structure** (include/voxel_map.h):
- Octree-based with configurable layering (`max_layer`, `layer_init_num`)
- LRU caching for memory management (`capacity`: 0=disabled, default 100000)
- Intensity fusion support (`intensity_fusion_en`)
- Point-to-plane optimization with eigenvalue-based plane fitting
- Neighbor search with configurable types (8-neighbor vs 24-neighbor)

**Pillar Voxel System** (config/livox_multi_lidar.yaml:69-82):
- **Purpose**: Ground detection and isolation point identification using vertical pillar voxels
- **Key Functions** (all sequential, no parallelization):
  1. `BuildPillarMap()`: Organize point cloud into pillar voxels (voxel_map.cpp:1389-1436)
  2. `GroundDetection()`: Initial ground voxel classification (voxel_map.cpp:1438-1453)
  3. `AdjacentCheck()`: Adjacency-based filtering with `adjacent_check_en` toggle (voxel_map.cpp:1455-1565)
  4. `PlaneFitting()`: Iterative RANSAC-style plane refinement (voxel_map.cpp:1567-1668)
  5. `UpdateFlags()`: Update point cloud ground/isolated flags (voxel_map.cpp:1670-1697)
  6. `PublishPillarPoints()`: Publish ground and isolated points (voxel_map.cpp:1699-1783)

**Configuration Parameters**:
- `pillar_voxel_en`: Enable/disable entire system (default: false)
- `adjacent_check_en`: Toggle adjacency filtering (default: true)
- `plane_fitting_ground_en`: Enable iterative plane fitting (default: true)
- `plane_fitting_iterations`: Number of RANSAC iterations (default: 3)
- `plane_fitting_iteration_thresholds`: Distance thresholds per iteration [0.2, 0.1, 0.1]
- `min_adjacent_num`: Minimum adjacent ground voxels required (default: 3)
- `neighbor_search_type`: 0=8-neighbor, 1=24-neighbor (default: 0)

**Execution Flow** (LIVMapper.cpp:473-484):
```cpp
BuildPillarMap(feats_down_world);
GroundDetection(_state.pos_end);
AdjacentCheck();  // Returns seed points for plane fitting
PlaneFitting(seed_points);
UpdateFlags(_pv_list);
PublishPillarPoints(pubGroundCloud, pubIsolatedCloud);
ClearPillarVoxels();  // Memory cleanup
```

**Output Topics**:
- `/cloud_ground`: Ground point cloud
- `/cloud_isolated`: Isolated point cloud (single voxels without ground neighbors)

## Usage

### Multi-LiDAR Mapping with External IMU
```bash
# Terminal 1: Launch mapping with multi-LiDAR fusion
roslaunch fast_livo mapping_livox_multi_lidar.launch

# Terminal 2: Play rosbag data
rosbag play your_multi_lidar.bag
```

### Available Launch Files
- `mapping_livox_multi_lidar.launch`: Multi-LiDAR setup with external IMU support
- `mapping_avia.launch`: Livox Avia LiDAR configuration
- `mapping_avia_marslvig.launch`: Livox Avia with MARS LVIG visual-inertial dataset
- `mapping_mid360.launch`: Livox Mid360 configuration
- `mapping_hesaixt32_hilti22.launch`: Hesai XT32 + Hilti dataset setup
- `mapping_hap.launch`: HAP dataset configuration
- `mapping_kitti.launch`: KITTI dataset configuration
- `mapping_ouster_ntu.launch`: Ouster NTU dataset configuration
- `mapping_subt_mrs.launch`: SubT-MRS dataset (Velodyne VLP-16)

**RViz Configurations**: Pre-configured visualization files in `rviz_cfg/`:
- `livox_multi_lidar.rviz`: Multi-LiDAR setup visualization
- `fast_livo2.rviz`, `fast_livo2_follow_view.rviz`: Standard and follow-view configs
- `hilti.rviz`, `ntu_viral.rviz`, `M300.rviz`: Dataset-specific visualizations

### Debugging (launch/mapping_livox_multi_lidar.launch:29-30)
Uncomment and add to `<node>` tag:
- `launch-prefix="gdb -ex run --args"` for GDB debugging
- `launch-prefix="valgrind --leak-check=full"` for memory leak detection

### Data Topics (livox_multi_lidar.yaml:2-5)
- **LiDAR**: `/livox/multi_lidar` (merged multi-LiDAR data)
- **IMU**: `/livox/imu_192_168_1_159` (default internal IMU from LiDAR 159)
- **External IMU**: `/novatel/oem7/odom` (external IMU odometry)
- **Camera**: `/left_camera/image` (optional, VIO mode when `img_en: 1`)
- **Odometry Output**: `/aft_mapped_to_init`
- **Ground Points**: `/cloud_ground` (pillar voxel ground detection output)
- **Isolated Points**: `/cloud_isolated` (pillar voxel isolated point output)

## Configuration

### Key Configuration Files
- `livox_multi_lidar.yaml`: Multi-LiDAR with external IMU (primary config)
- `HILTI22.yaml`: Hesai XT32 + Hilti industrial dataset
- `NTU_VIRAL.yaml`: NTU viral dataset with visual-inertial data
- `MARS_LVIG.yaml`: MARS LVIG visual-inertial dataset
- `avia.yaml`, `mid360.yaml`, `kitti.yaml`, `hap.yaml`: Sensor-specific configs
- `camera_pinhole.yaml`: Pinhole camera intrinsics for VIO
- `camera_*.yaml`: Dataset-specific camera configurations (fisheye, MARS_LVIG, NTU_VIRAL, SubT_MRS)

### Important Parameters (livox_multi_lidar.yaml)

**Common** (lines 1-8):
- `img_en`: Enable VIO mode (0 = LIO only, 1 = VIO enabled)
- `lidar_en`: Enable LiDAR processing
- `imu_topic`: Default internal IMU from LiDAR 159 (`/livox/imu_192_168_1_159`)

**Preprocessing** (lines 23-28):
- `lidar_type`: LiDAR type (1 = Livox Avia)
- `scan_line`: Scan line count (Avia: 6, Mid360: 4)
- `blind`: Blind spot distance in meters (default: 0.8)
- `point_filter_num`: Point downsampling factor (default: 1)

**VIO** (lines 31-42):
- `capacity`: LRU cache for visual feature map (0 = disable)
- `exposure_estimate_en`: Enable exposure time estimation

**Publish** (lines 90-96):
- `dense_map_en`: Publish dense map
- `pub_effect_en`: Publish effective points for visualization
- `pub_scan_num`: Number of scans to publish

**Point Cloud Saving** (lines 102-108):
- `save_en`: Enable PCD file saving
- `filter_size_pcd`: Voxel filter size for saved PCD
- `interval`: Frames per PCD file (-1 = all in one file, may cause memory issues)

**Evaluation** (lines 98-99):
- `seq_name`: Sequence name for trajectory evaluation output

**External IMU** (lines 111-119):
- `external_imu/enable`: Enable external IMU fusion
- `external_imu/external_imu_init_frame`: Initialization frames (default: 30)
- `external_imu/external_imu_only`: Use only external IMU (default: false)
- `external_imu/external_R`, `external_T`: Extrinsic calibration

**Voxel Mapping** (lines 54-66):
- `lio/voxel_size`: Voxel resolution (default: 1.0 meter)
- `lio/capacity`: LRU cache capacity (default: 0 = disabled)
- `lio/intensity_fusion_en`: Enable intensity-based fusion

**Pillar Voxel System** (lines 69-82):
- `pillar_voxel/pillar_voxel_en`: Enable pillar voxel ground detection (default: false)
- `pillar_voxel/adjacent_check_en`: Enable adjacency-based filtering (default: true)
- `pillar_voxel/min_adjacent_num`: Minimum adjacent ground voxels (default: 3)
- `pillar_voxel/plane_fitting_ground_en`: Enable iterative plane fitting (default: true)
- `pillar_voxel/plane_fitting_iterations`: RANSAC iterations (default: 3)
- `pillar_voxel/plane_fitting_iteration_thresholds`: Per-iteration distance thresholds (default: [0.2, 0.1, 0.1])
- `pillar_voxel/plane_fitting_distance_threshold`: Final classification threshold (default: 0.5)

**Multi-LiDAR Merger** (launch file lines 9-11):
- Input topics: `/livox/lidar_192_168_1_159`, `_160`, `_161`
  - Topic naming: `159`, `160`, `161` refer to the last octet of LiDAR IP addresses (e.g., 192.168.1.159)
  - Modify these in launch file when using different LiDAR IP configurations
- Extrinsic calibration in `extrin_calib` section (lines 10-16)

**Camera Image Handling** (launch file line 27):
- `image_transport republish` node decompresses compressed camera images for VIO
- Converts `compressed in:=/left_camera/image` to `raw out:=/left_camera/image`
- Required when camera publishes compressed images only

## SubT-MRS Dataset Support

**Configuration Files:**
- `config/SubT_MRS.yaml`: Main configuration for SubT-MRS datasets (Velodyne VLP-16)
- `config/camera_pinhole_SubT_MRS.yaml`: Camera intrinsics (MEI fisheye parameters converted to Pinhole)
- `launch/mapping_subt_mrs.launch`: Launch file for SubT-MRS

**Key SubT-MRS Settings:**
- `lidar_type: 2` (Velodyne, uses `sensor_msgs/PointCloud2`)
- `scan_line: 16` (VLP-16 has 16 laser channels)
- Input topics: `/velodyne_points`, `/imu/data`
- No `image_transport` node required (raw images used, `img_en: 0` by default)

## Point Cloud Format Requirements

### Velodyne PointCloud2 Field Order

When working with Velodyne LiDAR (`lidar_type: 2`), PointCloud2 **must** have fields in this exact order:

| Field | Offset | Type | Description |
|-------|--------|------|-------------|
| x | 0 | FLOAT32 | |
| y | 4 | FLOAT32 | |
| z | 8 | FLOAT32 | |
| intensity | 12 | FLOAT32 | |
| **time** | 16 | FLOAT32 | Must come **before** ring |
| **ring** | 20 | UINT16 | Must come **after** time |

Point step: 22 bytes (4+4+4+4+4+2+2 padding)

This matches the `velodyne_ros::Point` structure defined in `include/preprocess.h:67-79`:

```cpp
namespace velodyne_ros {
struct Point {
    PCL_ADD_POINT4D;      // x, y, z
    float intensity;       // offset 12
    float time;            // offset 16
    std::uint16_t ring;    // offset 20
};
}
```

**Wrong field order causes**: `[ LIO ]: No point!!!` error or segfault

## Camera Configuration Requirements

**Critical**: Camera YAML files must use proper float notation:

```yaml
# WRONG - causes segfault during initialization
scale: 1
cam_fx: 758.315

# CORRECT
scale: 1.0
cam_fx: 758.3153257832925
```

The `vk::camera_loader::loadFromRosNs()` uses `getParam<double>()` which expects proper float format. Integer values like `scale: 1` can cause parsing failures.

**When VIO is disabled** (`img_en: 0`):
- Camera config is still loaded (required by initialization)
- Use `camera_pinhole.yaml` or dataset-specific camera config
- Camera parameters are read even if not actively used for odometry

## Common Issues and Debugging

### "No point!!!" Error

**Symptoms**: Repeated `[ LIO ]: No point!!!` messages after receiving LiDAR data

**Causes**:
1. `lidar_type` mismatch (config has wrong LiDAR type)
2. PointCloud2 field order incorrect (time/ring order swapped)
3. `blind` parameter filtering all points
4. Empty or malformed point cloud messages

**Debug Steps**:
1. Verify input topic: `rostopic list` and `rostopic info /velodyne_points`
2. Check point cloud content: `rostopic echo /velodyne_points -n 1`
3. Confirm `lidar_type: 2` for Velodyne PointCloud2 format
4. Reduce `blind` parameter temporarily to test

### Process Crash (Exit Code -11)

**Symptoms**: Node dies immediately after startup

**Common Causes**:
1. Camera config YAML syntax error (e.g., `scale: 1` instead of `scale: 1.0`)
2. Missing camera configuration file
3. ROS master not running when VIO initialization occurs

**Debug Steps**:
1. Check camera config YAML for proper float notation
2. Verify camera file exists at correct path
3. Run with gdb to get backtrace:
   ```xml
   <node ... launch-prefix="gdb -ex run --args">
   ```

### Rosbag Reading Error

**Symptoms**: `Error reading from file: wanted X bytes, read Y bytes`

**Cause**: Rosbag was not properly closed during conversion

**Fix**: Run `rosbag reindex your_file.bag`



**Data Processing:**
- `Log/plot.py`: Plot trajectory, IMU data, and state estimation results (requires `mat_pre.txt`, `mat_out.txt`, `imu.txt`)
- `scripts/extract_odom_from_bag.py`: Extract odometry data from rosbag to TUM format
  ```bash
  python3 scripts/extract_odom_from_bag.py <bag_file> [output_file] --topic /novatel/oem7/odom
  ```
- `scripts/mesh.py`: Generate mesh from point cloud data
- `scripts/colmap_output.sh`: COLMAP integration for visual reconstruction

**Evaluation:**
- `Log/result/ntu_viral/evaluate_viral.py`: Convert trajectories for NTU VIRAL dataset evaluation
  - Converts SLAM trajectories from IMU frame to PRISM coordinate system
  - Converts Leica ground truth to TUM format
  - Usage: Modify file paths in script, then run to convert trajectories

## Development Notes

**Key Data Structures:**
- `StatesGroup` (common_lib.h:144-199): 19-dimensional state with covariance
- `ExternalIMUData` (common_lib.h:132-142): External IMU message with position, velocity, orientation, covariance
- `VoxelPlane` (voxel_map.h:77+): Voxel-based plane representation
- `PointToPlane` (voxel_map.h:61-75): Point-to-plane correspondence for optimization
- `PillarVoxelConfig` (voxel_map.h:218-241): Pillar voxel system configuration
  - Controls ground detection pipeline behavior
  - Includes adjacency filtering, plane fitting, and threshold parameters
- `pointWithVar` (common_lib.h): Point with variance and ground/isolated flags
  - `is_ground`: Set by pillar voxel ground detection
  - `is_isolated`: Set for isolated points (single voxels without neighbors)

**State Machine** (common_lib.h:53-59):
- `WAIT`: Initial state waiting for data
- `VIO`: Visual-inertial odometry mode
- `LIO`: LiDAR-inertial odometry mode
- `LO`: LiDAR-only mode

**Logging** (common_lib.h:35):
- Debug files saved to `Log/` directory via `DEBUG_FILE_DIR(name)` macro

**Architecture & Performance:**
- C++17 standard with CPU architecture-specific optimizations (ARM NEON, x86 native)
- Automatic multi-threading configuration based on CPU core count (`MP_EN` and `MP_PROC_NUM` defines)
- OpenMP support for parallel processing when available
- Optional mimalloc integration for improved memory allocation

**Sensor Integration:**
- Covariance-based external IMU fusion with configurable initialization periods
- Time synchronization handles offsets between heterogeneous sensors (`imu_time_offset`, `img_time_offset`)
- Seamless switching between pure LIO and VIO modes (`img_en` flag)
- Multi-LiDAR calibration and real-time data fusion via extrinsic parameters

**Memory Management:**
- Efficient voxel octree with LRU caching
- VIO feature map LRU cache (`vio/capacity`: 0 = disable)
- Pillar voxel cleanup: `ClearPillarVoxels()` called after each frame (LIVMapper.cpp:482)
- LAStools integration for point cloud processing (optional)
- Architecture-specific compile flags for optimal performance

**Pillar Voxel Ground Detection Pipeline:**
The pillar voxel system operates independently of the main voxel map:
1. **Input**: Downsampled world point cloud (`feats_down_world`)
2. **Pillar Organization**: Points grouped by (x,y) coordinates, vertical voxels by z
3. **Initial Classification**: Bottom voxel marked as ground if:
   - Only one voxel in pillar → `is_isolated_voxel_ = true`
   - Height angle check passes (if enabled)
4. **Adjacency Filtering**: Ground voxels without enough adjacent ground neighbors removed
5. **Plane Fitting**: Iterative RANSAC-style refinement:
   - Compute SVD-based plane from seed points
   - Filter points by distance threshold
   - Repeat for configured iterations
6. **Point Flag Update**: All points marked with `is_ground` or `is_isolated`
7. **Output**: Separate point clouds published for ground and isolated points
8. **Cleanup**: All pillar voxels deleted after each frame (no persistence)

**Important Implementation Notes:**
- Pillar voxel functions are **sequential only** - no parallelization (do not add OpenMP)
- Ground/isolated points **excluded** from ICP optimization (BuildResidualListOMP:866-870)
- `pv.is_ground` and `pv.is_isolated` flags must be reset in `BuildPillarMap` each frame
