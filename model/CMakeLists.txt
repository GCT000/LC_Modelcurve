include_directories(include)

# If this macro definition is turned on, 
# the optimization elapsed time is reduced by half the time, 
# but the error at the endpoints increases to the centimeter level.
# add_definitions(-DMAUNAL_DIFF)

add_library(curveModel_lib SHARED
    src/curve_modeling.cpp
)
target_link_libraries(curveModel_lib ${CERES_LIBRARIES} ${PCL_LIBRARIES} glog::glog ${yaml-cpp_LIBRARIES} yaml-cpp)

enable_testing()

add_executable(test test.cpp)
target_link_libraries(test curveModel_lib ${GTEST_LIBRARIES} ${OpenCV_LIBS} pthread)