cmake_minimum_required(VERSION 3.5)
project(curve_model)

# export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(GTest REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
find_package(VTK REQUIRED)

include_directories(${gflags_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})
message(WARNING glog::glog)


include_directories(${PROJECT_SOURCE_DIR}/lc_core/camera)
include_directories(${PROJECT_SOURCE_DIR}/lc_core/fileio)
include_directories(${PROJECT_SOURCE_DIR}/lc_core/line)

add_subdirectory(lc_simulation)
add_subdirectory(lc_core)
add_subdirectory(lc_preprocess)