cmake_minimum_required(VERSION 3.5)
project(curve_model)

set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})

# Ceres
set(Ceres_DIR /home/zyp/bin/Ceres-2.1.0/lib/cmake/Ceres)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# gflags
find_package(gflags REQUIRED)
include_directories(${gflags_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# opencv 4
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(TBB REQUIRED)
include_directories(${TBB_INCLUDE_DIRS})
include_directories(camera)
include_directories(fileio)
include_directories(line)

add_subdirectory(model)
add_subdirectory(simulation)
add_subdirectory(lidar_preprocess)
add_subdirectory(img_preprocess)