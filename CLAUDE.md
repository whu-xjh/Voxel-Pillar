# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is **voxel_pillar** (package name), a Voxel-Pillar system with pillar voxel redundant point detection. It's a high-speed LiDAR-Inertial Odometry system supporting multi-LiDAR fusion and external IMU integration, optimized for high-speed scenarios (>5 m/s). The system features 250Hz IMU propagation and efficient voxel-based mapping with LRU caching. (Visual-inertial odometry support has been removed; only the LIO path remains.)

## Build System

This is a ROS Catkin workspace. The project path is `/home/wsl-4080/code/Voxel-Pillar/catkin_ws/src/Voxel-Pillar` (WSL2, Ubuntu 20.04, ROS Noetic).

**IMPORTANT — CPU instability, always build on E-cores**: this machine's i9-14900KF has degraded/unstable P-cores (threads 0-15). Building on them causes random failures: g++ `internal compiler error: Segmentation fault` at varying system headers, linker `ld terminated with signal 11`, and even cmake parse errors that move between runs. Objects that compile "successfully" on P-cores may be silently corrupted. E-cores (threads 16-31) are stable. All builds — including the cmake configure step — must be wrapped in `taskset -c 16-31`:

```bash
taskset -c 16-31 bash -c 'source /opt/ros/noetic/setup.bash && catkin_make -C /home/wsl-4080/code/Voxel-Pillar/catkin_ws -j16'
source /home/wsl-4080/code/Voxel-Pillar/catkin_ws/devel/setup.bash
```

For development with debugging:
```bash
taskset -c 16-31 bash -c 'source /opt/ros/noetic/setup.bash && catkin_make -C /home/wsl-4080/code/Voxel-Pillar/catkin_ws -DCMAKE_BUILD_TYPE=Debug -j16'
```

**Architecture-specific optimizations** (CMakeLists.txt:21-37):
- ARM (32/64-bit): `-O3 -mcpu=native -mtune=native` with NEON support for 32-bit
- x86-64: `-O2` only (no `-march=native`; conservative to avoid compiler crashes)
- Multi-threading: Auto-configured based on CPU core count (`MP_EN`, `MP_PROC_NUM`)
- Debug builds: `-O0 -g`

**Dependencies**: PCL (≥1.8), Eigen3 (≥3.3.4), Boost. Optional: mimalloc, OpenMP, LAStools.

**Python tooling**: evaluation scripts (e.g. `Log/result/ntu_viral/evaluate_viral.py`) need the conda env `xjh` (Python 3.10, has `evo`): `source /root/miniconda3/etc/profile.d/conda.sh && conda activate xjh` before running them.

## Core Architecture

### System Components

**Executables:**
- `voxelpillar_mapping`: Main mapping node (src/main.cpp)
- `merge_lidar`: Multi-LiDAR data merger (fuses 3 Livox LiDARs)

**Libraries:**
- `laser_mapping`: Core LiDAR-IMU fusion (src/LIVMapper.cpp)
- `imu_proc`: 250Hz IMU propagation with bias estimation (src/IMU_Processing.cpp)
- `lio`: Voxel octree map with LRU caching (src/voxel_map.cpp)
- `pre`: Point cloud preprocessing and filtering (src/preprocess.cpp)

### Data Flow

1. **Multi-LiDAR Merger** (`merge_lidar`) → `/livox/multi_lidar`
2. **Preprocessing** (`pre`) filters and voxelizes point clouds
3. **IMU Processing** (`imu_proc`) propagates at 250Hz, estimates biases, supports external IMU fusion
4. **State Estimation** (`laser_mapping`) performs LiDAR-inertial odometry
5. **Voxel Mapping** (`lio`) maintains octree map with LRU caching (configurable via `lio/capacity`)

**State Management**: The `StatesGroup` struct (common_lib.h:125-214) maintains an 18-dimensional state including rotation, position, velocity, IMU biases, and gravity with covariance.

### External IMU Integration

**Key Files**: include/LIVMapper.h:25-40, include/IMU_Processing.h:38-41, include/common_lib.h:113-123

**External IMU Data Structure** (`ExternalIMUData`):
- Position, linear velocity, orientation (Euler angles), velocity covariance
- Interpolated from odom messages via `findClosestExternalIMUs()` and `interpolateExternalIMU()`

**Configuration** (config/merge_lidar.yaml:92-99):
- `enable`: Enable external IMU fusion
- `external_imu_init_frame`: Frames for external IMU initialization (code fallback: 3; shipped configs use 30–50)
- `external_imu_only`: Use only external IMU (default: false)
- `external_R`, `external_T`: Extrinsic calibration between IMUs
- `buffer_size`: External IMU buffer size

**Initialization**: External IMU initialized separately from internal IMU with `MAX_EXTERNAL_INI_COUNT=20` iterations vs `MAX_INI_COUNT=20` for internal (IMU_Processing.h:85).

### Voxel Map System

**Voxel Structure** (include/voxel_map.h):
- Octree-based with configurable layering (`max_layer`, `layer_init_num`)
- LRU caching for memory management (`capacity`: <=1 disables the cache; the
  cache holds at most `capacity` voxels, evicting least-recently-updated ones.
  Recency is driven by point insertion (`UpdateVoxelMap`/`BuildVoxelMap`) —
  voxel lookups during ICP do not refresh it. `enforceCapacity()` runs after
  both insert paths; evictions are logged with a cumulative counter
  (`evicted_voxel_count_`))
- Intensity fusion support (`intensity_fusion_en`): joint geometric+intensity
  score for plane selection, computed as the product of two normalized
  Gaussian likelihoods (geometry prob × intensity prob, both in the
  1/sqrt(sigma) * exp(-0.5 m^2) form); per-plane intensity stats are
  batch-initialized once then evolve via EMA (re-inits keep the history).
  The per-point measurement noise `sigma_meas` is auto-estimated during the
  init window (frame-to-frame NN differencing on static data, LIVMapper.cpp)
  and stored in `VoxelPlane::intensity_meas_var_`; it replaces the former
  hard std floor of 1e-3. The effective intensity variance
  `sigma_int_sq = std^2 + meas_var` is floored at 1e-3 so it is always
  strictly positive: every candidate plane is scored with the same 2D
  likelihood form (no geometry-only fallback while fusion is enabled).
  The EMA rate is configurable via `lio/intensity_ema_alpha` (default 0.5,
  clamped to (0,1])
- Intensity association gate (`intensity_gate_en`, `intensity_gate_k`):
  rejects associations whose intensity mismatch exceeds k sigma_int
- Point-to-plane optimization with eigenvalue-based plane fitting
- Neighbor search with configurable types (8-neighbor vs 24-neighbor)

**Pillar Voxel System** (config/merge_lidar.yaml `pillar_voxel` block):
- **Purpose**: Redundant point detection and isolated point identification using vertical pillar voxels
- **Key Functions** (all sequential, no parallelization):
  1. `BuildPillarMap()`: Organize point cloud into pillar voxels — flat `unordered_map<PillarLocation, vector<pair<z, PillarVoxel>>>`, each pillar's array sorted by z (voxel_map.cpp)
  2. `pillarDetection()`: Three sequential steps — initial per-pillar flags → horizontal adjacency check (with height consistency) → point label assignment; early-exits when Step 1 flags nothing
  3. `DefineSkipPoints()`: Apply skip filter to the main point cloud (newest-n-per-voxel retention via `applyVoxelRetention()`)
  4. `PublishPillarPoints()`: Publish redundant/isolated clouds (skips assembly and serialization when a topic has no subscribers)
  5. `ClearPillarVoxels()`: Whole structure cleared after each frame (no persistence)

**Configuration Parameters** (loaded by `loadPillarVoxelConfig`, voxel_map.cpp):
- `pillar_voxel_en`: Enable/disable entire system (default: false)
- `voxel_size`: Pillar voxel resolution (default: 1.0)
- `redundant_detection_method`: 0=none (isolated points only), 1=neighborhood redundant detection
- `adjacent_redundant_threshold`: Minimum adjacent occupied voxels (same z-layer, height-consistent) to confirm a redundant voxel (0 = disable redundant detection)
- `adjacent_isolated_threshold`: Minimum adjacent voxels to CANCEL isolation (caution: 0 skips the check entirely — opposite semantics to the redundant threshold)
- `redundant_neighbor_type` / `isolated_neighbor_type`: 0=4-neighbor, 1=8-neighbor (`neighbor_type` is a legacy alias of the former)
- `keep_num_per_voxel`: 0=skip all flagged points, n=keep the n newest points per flagged voxel (default: 0)
- `keep_redundant` / `keep_isolated`: apply retention (true) or skip all (false), per category
- `height_consistency_ratio`: adjacent voxels count as neighbors only if virtual-point heights differ by ≤ ratio × voxel_size (default: 0.25)

**Execution Flow** (LIVMapper.cpp, guarded by `if (pillar_config.pillar_voxel_en_)`):
```cpp
voxelmap_manager->pillar_map_.BuildPillarMap(feats_down_world);
voxelmap_manager->pillar_map_.pillarDetection();
voxelmap_manager->DefineSkipPoints(feats_down_world);
voxelmap_manager->pillar_map_.PublishPillarPoints(pubRedundantCloud, pubIsolatedCloud);
voxelmap_manager->pillar_map_.removeFlaggedPoints(feats_down_body, feats_down_world, voxelmap_manager->skip_list_);
voxelmap_manager->ClearPillarVoxels();
```

**Output Topics**:
- `/cloud_redundant`: Redundant point cloud
- `/cloud_isolated`: Isolated point cloud (single voxels without redundant neighbors)

## Usage

### Multi-LiDAR Mapping with External IMU
```bash
# Terminal 1: Launch mapping with multi-LiDAR fusion
roslaunch voxel_pillar mapping_merge_lidar.launch

# Terminal 2: Play rosbag data
rosbag play your_multi_lidar.bag
```

### Available Launch Files
- `mapping_merge_lidar.launch`: Multi-LiDAR setup with external IMU support
- `mapping_avia.launch`: Livox Avia LiDAR configuration
- `mapping_avia_marslvig.launch`: Livox Avia with MARS LVIG dataset
- `mapping_geode.launch`: GEODE Livox Avia dataset (Shield & Tunneling tunnels)
- `mapping_hesaixt32_hilti22.launch`: Hesai XT32 + Hilti dataset setup
- `mapping_ouster_ntu.launch`: Ouster NTU dataset configuration
- `mapping_subt_mrs.launch`: SubT-MRS dataset (Velodyne VLP-16)

**RViz Configurations**: Pre-configured visualization files in `rviz_cfg/`:
- `merge_lidar.rviz`: Multi-LiDAR setup visualization
- `voxel_pillar.rviz`: General voxel pillar visualization
- `hilti.rviz`, `ntu_viral.rviz`, `M300.rviz`: Dataset-specific visualizations

### Debugging (launch/mapping_merge_lidar.launch:25-28)
Uncomment and add to `<node>` tag:
- `launch-prefix="gdb -ex run --args"` for GDB debugging
- `launch-prefix="valgrind --leak-check=full"` for memory leak detection

### Frame and Topic Conventions
- World frame id: `world` (renamed from the legacy `camera_init`); body frame: `body`; odometry child frame: `aft_mapped`
- IMU propagation odometry topic: `/imu_propagate` (renamed from the legacy `/LIVO2/imu_propagate`)

### Data Topics (merge_lidar.yaml:1-5)
- **LiDAR**: `/livox/lidar` (default input topic)
- **IMU**: `/livox/imu_192_168_1_159` (default internal IMU from LiDAR 159)
- **External IMU**: `/novatel/oem7/odom` (external IMU odometry)
- **Odometry Output**: `/aft_mapped_to_init`
- **Redundant Points**: `/cloud_redundant` (pillar voxel redundant point detection output)
- **Isolated Points**: `/cloud_isolated` (pillar voxel isolated point output)

## Configuration

### Key Configuration Files
- `merge_lidar.yaml`: Multi-LiDAR with external IMU (primary config)
- `HILTI22.yaml`: Hesai XT32 + Hilti industrial dataset
- `NTU_VIRAL.yaml`: NTU viral dataset
- `MARS_LVIG.yaml`: MARS LVIG dataset
- `avia.yaml`: Livox Avia sensor-specific config
- `GEODE.yaml`: GEODE Livox Avia tunnel dataset

### Important Parameters (merge_lidar.yaml)

**Common** (lines 1-5):
- `lidar_en`: Enable LiDAR processing
- `imu_topic`: Default internal IMU from LiDAR 159 (`/livox/imu_192_168_1_159`)

**Preprocessing** (lines 15-20):
- `lidar_type`: LiDAR type (1 = Livox Avia)
- `scan_line`: Scan line count (Avia: 6, Mid360: 4)
- `blind`: Blind spot distance in meters (default: 0.8)
- `point_filter_num`: Point downsampling factor (default: 1)

**Publish** (lines 76-80):
- `dense_map_en`: Publish dense map
- `pub_effect_en`: Publish effective points for visualization

**Point Cloud Saving** (lines 87-91):
- `save_en`: Enable PCD file saving
- `filter_size_pcd`: Voxel filter size for saved PCD
- `interval`: Frames per PCD file (10 = save every 10 frames)

**Evaluation** (lines 82-85):
- `seq_name`: Sequence name for trajectory evaluation output

**External IMU** (lines 92-99):
- `external_imu/enable`: Enable external IMU fusion
- `external_imu/external_imu_init_frame`: Initialization frames (code fallback: 3; shipped configs use 30–50)
- `external_imu/external_imu_only`: Use only external IMU (default: false)
- `external_imu/external_R`, `external_T`: Extrinsic calibration

**Voxel Mapping** (lines 32-47):
- `lio/voxel_size`: Voxel resolution (default: 1.0 meter)
- `lio/capacity`: LRU cache capacity, <=1 = disabled (default: 100000; all
  shipped configs set 0 = disabled, so voxel count is unbounded unless this or
  `local_map/map_sliding_en` is enabled)
- `lio/intensity_fusion_en`: Enable intensity-based fusion
- `lio/intensity_gate_en`: Enable intensity-based association rejection (default: false)
- `lio/intensity_gate_k`: Gate threshold in units of intensity sigma (default: 3.0)
- `lio/intensity_noise_est_en`: Online estimation of intensity measurement noise during the init window (default: true)
- `lio/intensity_ema_alpha`: EMA adaptation rate of plane intensity stats, in (0,1] (default: 0.5)

**Pillar Voxel System** (`pillar_voxel` block):
- `pillar_voxel/pillar_voxel_en`: Enable pillar voxel redundant point detection
- `pillar_voxel/voxel_size`: Pillar voxel resolution (default: 1.0)
- `pillar_voxel/redundant_detection_method`: 0=none (isolated only), 1=neighborhood
- `pillar_voxel/adjacent_redundant_threshold`: Minimum adjacent redundant voxels
- `pillar_voxel/adjacent_isolated_threshold`: Minimum adjacent voxels for isolation
- `pillar_voxel/redundant_neighbor_type` / `pillar_voxel/isolated_neighbor_type`: 0=4-neighbor, 1=8-neighbor
- `pillar_voxel/keep_num_per_voxel`: 0=skip all flagged points, n=keep n newest per voxel
- `pillar_voxel/keep_redundant` / `pillar_voxel/keep_isolated`: retention toggles per category
- `pillar_voxel/height_consistency_ratio`: Height tolerance as ratio of voxel_size (default: 0.25)

**Multi-LiDAR Merger** (launch file lines 8-12):
- Input topics: `/livox/lidar_192_168_1_159`, `_160`, `_161`
  - Topic naming: `159`, `160`, `161` refer to the last octet of LiDAR IP addresses (e.g., 192.168.1.159)
  - Modify these in launch file when using different LiDAR IP configurations
- LiDAR-IMU extrinsic calibration in `extrin_calib` section (lines 10-12)

## SubT-MRS Dataset Support

**Configuration Files:**
- `config/SubT_MRS.yaml`: Main configuration for SubT-MRS datasets (Velodyne VLP-16)
- `launch/mapping_subt_mrs.launch`: Launch file for SubT-MRS

**Key SubT-MRS Settings:**
- `lidar_type: 2` (Velodyne, uses `sensor_msgs/PointCloud2`)
- `scan_line: 16` (VLP-16 has 16 laser channels)
- Input topics: `/velodyne_points`, `/imu/data`

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
1. Missing or malformed config file (launch `rosparam load` path wrong)
2. ROS master not running when initialization occurs

**Debug Steps**:
1. Verify the YAML config file referenced by the launch file exists and parses (`rosparam load <file>`)
2. Run with gdb to get backtrace:
   ```xml
   <node ... launch-prefix="gdb -ex run --args">
   ```

### Rosbag Reading Error

**Symptoms**: `Error reading from file: wanted X bytes, read Y bytes`

**Cause**: Rosbag was not properly closed during conversion

**Fix**: Run `rosbag reindex your_file.bag`

## Data Processing and Evaluation

**Data Processing:**
- `Log/plot.py`: Plot trajectory, IMU data, and state estimation results (requires `mat_pre.txt`, `mat_out.txt`, `imu.txt`)

**Evaluation:**
- `Log/result/ntu_viral/evaluate_viral.py`: Convert trajectories for NTU VIRAL dataset evaluation
  - Converts SLAM trajectories from IMU frame to PRISM coordinate system
  - Converts Leica ground truth to TUM format
  - Usage: Modify file paths in script, then run to convert trajectories

## Development Notes

**Key Data Structures:**
- `StatesGroup` (common_lib.h:125-214): 18-dimensional state with covariance
- `ExternalIMUData` (common_lib.h:113-123): External IMU message with position, velocity, orientation, covariance
- `VoxelPlane` (voxel_map.h:77+): Voxel-based plane representation
- `PointToPlane` (voxel_map.h:61-75): Point-to-plane correspondence for optimization
- `PillarVoxelConfig` (voxel_map.h): Pillar voxel system configuration
  - Controls redundant point detection pipeline behavior
  - Includes adjacency filtering, retention and threshold parameters
- `pointWithVar` (common_lib.h): Point with variance. Its `is_redundant`/`is_isolated`
  bool fields are legacy — the pillar system now tracks per-point state in
  `PillarVoxelMap::point_labels_` instead

**State Machine** (common_lib.h:39-44):
- `WAIT`: Initial state waiting for data
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
- Time synchronization handles offsets between heterogeneous sensors (`imu_time_offset`, `lidar_time_offset`)
- Multi-LiDAR calibration and real-time data fusion via extrinsic parameters

**Memory Management:**
- Efficient voxel octree with LRU caching
- Pillar voxel cleanup: `ClearPillarVoxels()` called after each frame (LIVMapper.cpp:437)
- LAStools integration for point cloud processing (optional)
- Architecture-specific compile flags for optimal performance

**Pillar Voxel Redundant Point Detection Pipeline:**
The pillar voxel system operates independently of the main voxel map:
1. **Input**: Downsampled world point cloud (`feats_down_world`)
2. **Pillar Organization**: Points grouped by (x,y) pillar, vertical voxels by z; per-pillar voxel arrays sorted by z; each voxel keeps point indices and a running-average virtual point
3. **Detection** (`pillarDetection()`, three steps):
   - Step 1: per-pillar flags — the bottom voxel without a close voxel above (< 2·voxel_size) is a redundant candidate; a voxel with ≥ 2·voxel_size gaps to both vertical neighbors is an isolated candidate; early-exit when nothing is flagged
   - Step 2: horizontal adjacency check — redundant candidates need ≥ `adjacent_redundant_threshold` occupied neighbor voxels in the same z-layer with consistent height (≤ `height_consistency_ratio`·voxel_size); isolated candidates with ≥ `adjacent_isolated_threshold` such neighbors are cancelled
   - Step 3: assign per-point labels (LABEL_REDUNDANT / LABEL_ISOLATED)
4. **Skip Point Definition** (`DefineSkipPoints()`):
   - Method 0: skips isolated points only (subject to `keep_isolated`/`keep_num_per_voxel` retention)
   - Method 1: also skips redundant points (subject to `keep_redundant`/`keep_num_per_voxel` retention; newest-n-per-voxel retention via `applyVoxelRetention()`)
5. **Output**: `/cloud_redundant` and `/cloud_isolated` (only assembled and published when subscribers exist)
6. **Cleanup**: All pillar voxels cleared after each frame (no persistence)

**Important Implementation Notes:**
- Pillar voxel functions are **sequential only** - no parallelization (do not add OpenMP)
- Flagged points are **deleted from the frame outright** in the pillar block of
  `handleLIO` (`PillarVoxelMap::removeFlaggedPoints` compacts the index-aligned
  `feats_down_body`/`feats_down_world` together and clears `skip_list_`): they
  enter neither ICP residuals nor the voxel map. The pillar block runs before
  first-frame `BuildVoxelMap` so the first frame is filtered as well. With
  `dense_map_en` enabled, published/saved clouds still come from the full
  `feats_undistort` (untouched).
- Per-point state lives in `PillarVoxelMap::point_labels_` (LABEL_NORMAL/REDUNDANT/ISOLATED), reset each frame in `BuildPillarMap()`
- Publishing skips cloud assembly and serialization when a topic has no subscribers
