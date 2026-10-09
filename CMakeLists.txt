cmake_minimum_required(VERSION 2.8.3)
project(voxel_pillar)

# Build configuration
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Prefer gold when available; otherwise use the default linker.
find_program(GOLD_LINKER ld.gold)
if(GOLD_LINKER)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=gold")
  set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fuse-ld=gold")
  message(STATUS "Using gold linker: ${GOLD_LINKER}")
else()
  message(STATUS "ld.gold not found, using default linker")
endif()

# Architecture-specific Release optimizations
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(RELEASE_OPTIMIZATION_FLAGS "-O3 -mcpu=native -mtune=native -ffast-math")
  else()
    set(RELEASE_OPTIMIZATION_FLAGS "-O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  add_definitions(-DARM_ARCH)
else()
  # Keep Eigen alignment compatible with the system's prebuilt PCL libraries.
  set(RELEASE_OPTIMIZATION_FLAGS "-O3")
  add_definitions(-DX86_ARCH)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${RELEASE_OPTIMIZATION_FLAGS}")
message(STATUS "Release optimizations: ${RELEASE_OPTIMIZATION_FLAGS}")

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Runtime multithreading (independent of make's parallel job count)
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

if(N GREATER 4)
  set(PROC_NUM 4)
elseif(N GREATER 2)
  set(PROC_NUM 2)
elseif(N GREATER 1)
  set(PROC_NUM ${N})
else()
  set(PROC_NUM 1)
endif()

if(PROC_NUM GREATER 1)
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# Optional dependencies
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# Required dependencies and catkin exports
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
)

# Libraries and executables
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)

add_executable(voxelpillar_mapping src/main.cpp)
add_executable(merge_lidar src/merge_lidar.cpp)

# Target dependencies
target_link_libraries(lio
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(laser_mapping
  lio
  pre
  imu_proc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(voxelpillar_mapping
  laser_mapping
  lio
  pre
  imu_proc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(merge_lidar
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(mimalloc_FOUND)
  target_link_libraries(voxelpillar_mapping mimalloc)
endif()

# Explicitly resolve OpenMP runtime symbols in both executables.
if(OpenMP_CXX_FOUND)
  target_link_libraries(voxelpillar_mapping OpenMP::OpenMP_CXX)
  target_link_libraries(merge_lidar OpenMP::OpenMP_CXX)
endif()
