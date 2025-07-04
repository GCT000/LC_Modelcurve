include_directories(model)
include_directories(factor)
include_directories(utils)
include_directories(matcher)
include_directories(transmission_model)
include_directories(../lc_preprocess/img_preprocess)
include_directories(cloud_registration)
include_directories(cal_distance)


# If this macro definition is turned on, 
# the optimization elapsed time is reduced by half the time, 
# but the error at the endpoints increases to the centimeter level.
# add_definitions(-DMAUNAL_DIFF)

add_library(curveModel_lib SHARED
    camera/camera.cpp
    fileio/output.cpp
    matcher/matcher.cpp
    matcher/kdtree.cpp
    model/curve_modeling.cpp
    transmission_model/parabola.cpp
    transmission_model/catenary.cpp
    transmission_model/transmissionModel.cpp
    utils/ex_optimization.cpp
    utils/evaluation.cpp
    utils/visualization.cpp
    ../lc_preprocess/img_preprocess/imgPreProcess.cpp
)

add_library(cloud_registration_lib SHARED
    cloud_registration/cloud_registration.cpp
)

add_library(cal_distance_lib SHARED
    cal_distance/cal_distance.cpp
)

target_link_libraries(curveModel_lib ${CERES_LIBRARIES} ${PCL_LIBRARIES} glog::glog ${yaml-cpp_LIBRARIES} yaml-cpp ${OpenCV_LIBS})
target_link_libraries(cloud_registration_lib ${catkin_LIBRARIES})
target_link_libraries(cal_distance_lib ${catkin_LIBRARIES})

add_executable(read_imu read_imu.cpp)
target_link_libraries(read_imu curveModel_lib ${GTEST_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIBRARIES} pthread ${catkin_LIBRARIES})
# add_executable(cal_distance cal_distance/cal_distance.cpp)
# target_link_libraries(cal_distance curveModel_lib ${GTEST_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIBRARIES} pthread ${catkin_LIBRARIES})

enable_testing()

add_executable(curve test.cpp)
target_link_libraries(curve curveModel_lib cloud_registration_lib cal_distance_lib ${GTEST_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIBRARIES} pthread)
