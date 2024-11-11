add_executable(ippTest ippTest.cpp imgPreProcess.cpp)
target_link_libraries(ippTest ${OpenCV_LIBS} glog::glog)