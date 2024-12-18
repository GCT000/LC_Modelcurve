# If this macro definition is turned on, 
# the optimization elapsed time is reduced by half the time, 
# but the error at the endpoints increases to the centimeter level.
# add_definitions(-DMAUNAL_DIFF)

add_executable(ppTest ppTest.cpp lidarPreProcess.cpp)
target_link_libraries(ppTest ${PCL_LIBRARIES} glog::glog TBB::tbb ${OpenCV_LIBS})