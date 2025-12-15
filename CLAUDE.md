# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is **fast_livo** (package name), a high-speed LiDAR-Inertial Odometry system based on the LOAM algorithm. It supports multi-LiDAR fusion, visual-inertial odometry, and external IMU integration optimized for high-speed scenarios (>5 m/s). The system features 250Hz IMU propagation and efficient voxel-based mapping.

## Build System

This is a ROS Catkin workspace using standard catkin build system:

```bash
# Build the workspace (from HighSpeed-LIO-main directory)
cd /home/xjh/Doc/highspeed-lio/catkin_ws
catkin_make

# Source the workspace
source devel/setup.bash

# For development with debugging support
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

The CMakeLists.txt includes architecture-specific optimizations:
- ARM (32/64-bit): Native CPU optimizations with NEON support
- x86-64: Conservative optimizations to avoid compiler crashes
- Automatic multi-threading configuration based on CPU core count

## Key Dependencies

- **ROS**: roscpp, rospy, std_msgs, sensor_msgs, geometry_msgs, nav_msgs, tf, pcl_ros, vikit_common, vikit_ros
- **External Libraries**: PCL (>=1.8), Eigen3 (>=3.3.4), OpenCV, Sophus, Boost
- **Optional**: mimalloc (memory allocator), OpenMP (multithreading), LAStools
- **Architecture**: Optimized for both ARM and x86 with native optimizations

## Core Architecture

### Executables and Libraries
The build creates two main executables and four shared libraries:

**Executables:**
- `fastlivo_mapping`: Main LIO/VIO mapping node
- `livox_multi_lidar`: Multi-LiDAR data merger

**Libraries:**
- `laser_mapping` (src/LIVMapper.cpp): Main fusion logic and state management
- `vio` (src/vio.cpp): Visual-Inertial Odometry with feature tracking
- `lio` (src/voxel_map.cpp): Voxel-based mapping with efficient octree structure
- `pre` (src/preprocess.cpp): Point cloud preprocessing and filtering
- `imu_proc` (src/IMU_Processing.cpp): IMU processing, bias estimation, external IMU integration

### Main Components
- **LIVMapper** (`src/LIVMapper.cpp`): Core LiDAR-IMU fusion handling multi-sensor data
- **VoxelMapManager** (`src/voxel_map.cpp`): Efficient voxel octree with LRU caching and neighbor search
- **IMU_Processing** (`src/IMU_Processing.cpp`): High-frequency (250Hz) IMU propagation and bias estimation
- **Multi-LiDAR Handler** (`src/livox_multi_lidar.cpp`): Real-time multi-LiDAR data fusion and synchronization
- **VIO Module** (`src/vio.cpp`): Visual-inertial odometry with feature tracking (optional)

### Key Features
- **Multi-Sensor Fusion**: LiDAR-Inertial-Visual with seamless switching between pure LIO and VIO modes
- **External IMU Integration**: Covariance-based fusion with configurable initialization periods
- **High-Frequency Propagation**: 250Hz IMU propagation optimized for high-speed applications (>5 m/s)
- **Voxel-Based Mapping**: Efficient octree structure with LRU caching for memory management
- **Multi-LiDAR Support**: Real-time heterogeneous LiDAR sensor fusion and synchronization
- **Architecture Optimization**: Native CPU optimizations for ARM and x86 with automatic multi-threading

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

# Terminal 3: Optional - Test external IMU integration
python3 scripts/test_external_imu.py
```

### Debugging and Development
The launch file contains commented debugging prefixes:
- `launch-prefix="gdb -ex run --args"` for GDB debugging
- `launch-prefix="valgrind --leak-check=full"` for memory leak detection

### Available Launch Files
- `mapping_livox_multi_lidar.launch`: Multi-LiDAR setup with external IMU support
- `mapping_avia.launch`: Livox Avia LiDAR configuration
- `mapping_mid360.launch`: Livox Mid360 configuration
- `mapping_hesaixt32_hilti22.launch`: Hesai XT32 + Hilti dataset setup
- `mapping_kitti.launch`: KITTI dataset configuration

### Data Topics
- **LiDAR**: `/livox/multi_lidar` (merged multi-LiDAR data)
- **IMU**: `/livox/imu_192_168_1_159` (default internal IMU)
- **External IMU**: `/novatel/oem7/odom` (external IMU odometry)
- **Camera**: `/alphasense/cam0/image_raw` (optional, used in VIO mode)
- **Odometry**: `/aft_mapped_to_init` (fused odometry output)

## Code Structure

### Header Organization
- `include/`: Core header files
  - `LIVMapper.h`: Main mapper class with external IMU integration
  - `voxel_map.h`: Voxel octree structure and LRU cache management
  - `IMU_Processing.h`: IMU processing with external IMU support
  - `common_lib.h`: Common data structures and utilities
  - `utils/`: Mathematical utilities (SO3, types, color)

### Source Files
- `src/main.cpp`: Entry point
- `src/LIVMapper.cpp`: Main fusion logic
- `src/voxel_map.cpp`: Voxel mapping implementation
- `src/IMU_Processing.cpp`: IMU processing and bias estimation
- `src/preprocess.cpp`: Point cloud preprocessing
- `src/livox_multi_lidar.cpp`: Multi-LiDAR data handling

### Launch Files
- `launch/mapping_livox_multi_lidar.launch`: Main mapping launch file

## Development Notes

**Architecture & Performance:**
- C++17 standard with CPU architecture-specific optimizations (ARM NEON, x86 native)
- Automatic multi-threading configuration based on CPU core count
- OpenMP support for parallel processing when available
- Optional mimalloc integration for improved memory allocation

**Sensor Integration:**
- Covariance-based external IMU fusion with configurable initialization periods
- Time synchronization handles offsets between heterogeneous sensors
- Seamless switching between pure LIO and VIO modes
- Multi-LiDAR calibration and real-time data fusion

**Memory Management:**
- Efficient voxel octree with LRU caching for memory-constrained environments
- LAStools integration for point cloud processing (optional)
- Architecture-specific compile flags for optimal performance

## Data Processing and Scripts

**Testing:**
- `scripts/test_external_imu.py`: External IMU integration validation
- `scripts/mesh.py`: Mesh generation from point cloud data
- `scripts/colmap_output.sh`: COLMAP integration for visual reconstruction

**Dataset Configurations:**
- HILTI22: Hesai XT32 + Hilti industrial dataset
- NTU_VIRAL: NTU viral dataset with visual-inertial data
- MARS_LVIG: MARS LiDAR-Visual-Inertial-GPS dataset

**Log and Results:**
- `Log/guide.md`: Dataset processing guidelines
- `Log/result/`: Evaluation results and trajectory comparisons

