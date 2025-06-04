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

# yaml-cpp
find_package(yaml-cpp REQUIRED)
set(yaml-cpp_INCLUDE_DIRS /usr/include/yaml-cpp)
include_directories(${yaml-cpp_INCLUDE_DIRS})
message(WARNING ${yaml-cpp_INCLUDE_DIRS})

# Ceres
set(Ceres_DIR /usr/lib/cmake/Ceres)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
message(WARNING ${CERES_INCLUDE_DIRS})
# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# #glog
include_directories(/usr/include)

if(NOT TARGET glog::glog)
    add_library(glog::glog UNKNOWN IMPORTED)
    set_target_properties(glog::glog PROPERTIES
        IMPORTED_LOCATION /usr/lib/x86_64-linux-gnu/libglog.so
        INTERFACE_INCLUDE_DIRECTORIES /usr/include
    )
endif()


# gflags
find_package(gflags REQUIRED)
include_directories(${gflags_INCLUDE_DIRS})
message(WARNING ${gflags_LIBRARIES})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# opencv 4
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# find_package(TBB REQUIRED)
set(TBB_INCLUDE_DIRS "/usr/local/include")
set(TBB_LIBRARIES "/usr/lib/x86_64-linux-gnu/libtbb.so")

include_directories(${TBB_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/lc_core/camera)
include_directories(${PROJECT_SOURCE_DIR}/lc_core/fileio)
include_directories(${PROJECT_SOURCE_DIR}/lc_core/line)
# include_directories(${PROJECT_SOURCE_DIR}/thirdparty/CannyLines/src)


# add_subdirectory(thirdparty/CannyLines)
add_subdirectory(lc_simulation)
add_subdirectory(lc_core)
add_subdirectory(lc_preprocess)