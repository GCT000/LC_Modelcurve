include_directories(include)
include_directories(../lc_core/fileio)
add_executable(points_generation src/points_generation.cpp ../lc_core/fileio/output.cpp)
target_link_libraries(points_generation ${yaml-cpp_LIBRARIES} yaml-cpp glog::glog ${PCL_LIBRARIES} ${gflags_LIBRARIES})

add_executable(image_generation src/image_generation.cpp ../lc_core/camera/camera.cpp ../lc_core/fileio/output.cpp)
target_link_libraries(image_generation ${yaml-cpp_LIBRARIES} yaml-cpp glog::glog ${PCL_LIBRARIES} ${gflags_LIBRARIES} ${OpenCV_LIBS})

add_executable(curve_detection src/curve_detection.cpp src/curve.cpp ../lc_core/fileio/output.cpp)
target_link_libraries(curve_detection glog::glog ${PCL_LIBRARIES} ${gflags_LIBRARIES} ${OpenCV_LIBS})