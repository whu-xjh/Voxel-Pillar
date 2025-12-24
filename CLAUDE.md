# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is **fast_livo** (package name), a high-speed LiDAR-Inertial Odometry system based on the LOAM algorithm. It supports multi-LiDAR fusion, visual-inertial odometry, and external IMU integration optimized for high-speed scenarios (>5 m/s). The system features 250Hz IMU propagation and efficient voxel-based mapping.

## Build System

This is a ROS Catkin workspace. Build from the catkin workspace root:

```bash
cd /home/xjh/Doc/highspeed-lio/catkin_ws
catkin_make
source devel/setup.bash
```

For development with debugging:
```bash
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

The CMakeLists.txt includes architecture-specific optimizations:
- ARM (32/64-bit): Native CPU optimizations with NEON support (`-mcpu=native -mtune=native`)
- x86-64: Conservative optimizations (`-O2 -march=native`) to avoid compiler crashes
- Automatic multi-threading configuration based on CPU core count (`MP_EN` flag)

## Key Dependencies

- **ROS**: roscpp, rospy, std_msgs, sensor_msgs, geometry_msgs, nav_msgs, tf, pcl_ros, vikit_common, vikit_ros
- **External Libraries**: PCL (>=1.8), Eigen3 (>=3.3.4), OpenCV, Sophus, Boost
- **Optional**: mimalloc (memory allocator), OpenMP (multithreading), LAStools
- **Architecture**: Optimized for both ARM and x86 with native optimizations

## Core Architecture

### Key Features
- **Multi-Sensor Fusion**: LiDAR-Inertial-Visual with seamless switching between pure LIO and VIO modes
- **External IMU Integration**: Covariance-based fusion with configurable initialization periods
- **High-Frequency Propagation**: 250Hz IMU propagation optimized for high-speed applications (>5 m/s)
- **Voxel-Based Mapping**: Efficient octree structure with LRU caching for memory management
- **Multi-LiDAR Support**: Real-time heterogeneous LiDAR sensor fusion and synchronization
- **Architecture Optimization**: Native CPU optimizations for ARM and x86 with automatic multi-threading

### Data Flow Architecture
1. **Multi-LiDAR Merger** (`livox_multi_lidar` node) fuses multiple Livox LiDAR point clouds into single topic
2. **Preprocessing** (`pre` library) filters and voxelizes point cloud data
3. **IMU Processing** (`imu_proc` library) propagates state at 250Hz and estimates biases
4. **LIO/VIO Fusion** (`laser_mapping` library) performs LiDAR-inertial or visual-inertial odometry
5. **Voxel Mapping** (`lio` library) maintains efficient octree map with LRU caching

## Configuration

Configuration is managed through YAML files in `config/`:

**Main Configuration Files:**
- `livox_multi_lidar.yaml`: Multi-LiDAR setup with external IMU integration
- `HILTI22.yaml`: Hesai XT32 + Hilti dataset configuration
- `NTU_VIRAL.yaml`, `MARS_LVIG.yaml`: Dataset-specific configurations
- Sensor configs: `avia.yaml`, `mid360.yaml`, `kitti.yaml`, `hap.yaml`

**Key Configuration Sections:**
- `external_imu/`: External IMU enable/disable and topic configuration
- `imu/`: IMU covariance, bias parameters, and propagation settings
- `lio/`: LiDAR odometry parameters (voxel size, max iterations)
- `uav/`: High-speed application settings (imu_rate_odom, gravity_alignment)
- Camera configs for VIO mode (camera_pinhole.yaml, camera_*.yaml)

## Usage

### Multi-LiDAR Mapping with External IMU
```bash
# Terminal 1: Launch mapping with multi-LiDAR fusion
roslaunch fast_livo mapping_livox_multi_lidar.launch

# Terminal 2: Play rosbag data
rosbag play your_multi_lidar.bag
```

### Debugging and Development
The launch file (`launch/mapping_livox_multi_lidar.launch:29-30`) contains commented debugging prefixes:
- `launch-prefix="gdb -ex run --args"` for GDB debugging
- `launch-prefix="valgrind --leak-check=full"` for memory leak detection

Uncomment and add to the `<node>` tag to enable debugging.

### Available Launch Files
- `mapping_livox_multi_lidar.launch`: Multi-LiDAR setup with external IMU support
- `mapping_avia.launch`: Livox Avia LiDAR configuration
- `mapping_mid360.launch`: Livox Mid360 configuration
- `mapping_hesaixt32_hilti22.launch`: Hesai XT32 + Hilti dataset setup
- `mapping_kitti.launch`: KITTI dataset configuration
- `mapping_ouster_ntu.launch`: Ouster NTU dataset configuration

### Data Topics (from livox_multi_lidar.yaml)
- **LiDAR**: `/livox/multi_lidar` (merged multi-LiDAR data)
- **IMU**: `/livox/imu_192_168_1_159` (default internal IMU from LiDAR 159)
- **External IMU**: `/novatel/oem7/odom` (external IMU odometry, enabled in config)
- **Camera**: `/left_camera/image` (optional, used in VIO mode)
- **Odometry**: `/aft_mapped_to_init` (fused odometry output)

The multi-LiDAR merger (`livox_multi_lidar` node) merges three Livox LiDARs:
- `/livox/lidar_192_168_1_159`
- `/livox/lidar_192_168_1_160`
- `/livox/lidar_192_168_1_161`

## Code Structure

### Source Organization
- `src/main.cpp`: Entry point for fastlivo_mapping
- `src/LIVMapper.cpp`, `include/LIVMapper.h`: Core LiDAR-IMU fusion handling multi-sensor data
- `src/voxel_map.cpp`, `include/voxel_map.h`: Efficient voxel octree with LRU caching and neighbor search
- `src/IMU_Processing.cpp`, `include/IMU_Processing.h`: High-frequency (250Hz) IMU propagation and bias estimation with external IMU support
- `src/vio.cpp`, `include/vio.h`: Visual-inertial odometry with feature tracking (optional)
- `src/preprocess.cpp`, `include/preprocess.h`: Point cloud preprocessing and filtering
- `src/livox_multi_lidar.cpp`: Multi-LiDAR data merger (standalone executable)
- `src/frame.cpp`, `include/frame.h`: VIO frame management
- `src/visual_point.cpp`, `include/visual_point.h`: VIO feature point handling
- `src/imu_filter.cpp`: IMU filtering utilities

### Header Files
- `include/common_lib.h`: Common data structures and utilities
- `include/feature.h`: Feature detection definitions
- `utils/`: Mathematical utilities (SO3, types, color)

## Development Notes

**Architecture & Performance:**
- C++17 standard with CPU architecture-specific optimizations (ARM NEON, x86 native)
- Automatic multi-threading configuration based on CPU core count (`MP_EN` and `MP_PROC_NUM` defines)
- OpenMP support for parallel processing when available
- Optional mimalloc integration for improved memory allocation

**Sensor Integration:**
- Covariance-based external IMU fusion with configurable initialization periods (`external_imu_init_frame`)
- Time synchronization handles offsets between heterogeneous sensors (`imu_time_offset`, `img_time_offset`)
- Seamless switching between pure LIO and VIO modes (`img_en` flag)
- Multi-LiDAR calibration and real-time data fusion via extrinsic parameters

**Memory Management:**
- Efficient voxel octree with LRU caching (`lio/capacity`: 100000 voxels, 0 = disable)
- VIO feature map LRU cache (`vio/capacity`: 0 = disable)
- LAStools integration for point cloud processing (optional)
- Architecture-specific compile flags for optimal performance

**Pillar Voxel System:**
- Ground detection using pillar voxels (`pillar_voxel/pillar_voxel_en`)
- Configurable elevation axis and ground height angle threshold
- Adjacent voxel count threshold for ground classification

## Key Configuration Parameters

**External IMU (config/livox_multi_lidar.yaml:108-115):**
- `external_imu/enable`: Enable external IMU fusion
- `external_imu/external_imu_init_frame`: Frames for external IMU initialization (default: 30)
- `external_imu/time_offset`: Time offset for external IMU (seconds)
- `external_imu/external_R`, `external_T`: Extrinsic calibration between IMUs

**Voxel Mapping (config/livox_multi_lidar.yaml:54-65):**
- `lio/voxel_size`: Voxel map resolution (default: 1.0 meter)
- `lio/capacity`: LRU cache capacity (default: 100000, 0 = disable)
- `lio/intensity_fusion_en`: Enable intensity-based fusion

**Multi-LiDAR System:**
- Input topics configurable in launch file (lines 9-11)
- Extrinsic calibration in `extrin_calib` section (lines 10-16)

## Scripts and Tools

**Testing:**
- `scripts/test_external_imu.py`: External IMU integration validation
- `scripts/mesh.py`: Mesh generation from point cloud data
- `scripts/colmap_output.sh`: COLMAP integration for visual reconstruction

**Dataset Configurations:**
- HILTI22 (`config/HILTI22.yaml`): Hesai XT32 + Hilti industrial dataset
- NTU_VIRAL (`config/NTU_VIRAL.yaml`): NTU viral dataset with visual-inertial data
- MARS_LVIG (`config/MARS_LVIG.yaml`): MARS LiDAR-Visual-Inertial-GPS dataset

**Log and Results:**
- `Log/guide.md`: Dataset processing guidelines
- `Log/result/`: Evaluation results and trajectory comparisons

