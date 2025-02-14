include_directories(model)
include_directories(factor)
include_directories(utils)
include_directories(matcher)
include_directories(transmission_model)

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
)
target_link_libraries(curveModel_lib ${CERES_LIBRARIES} ${PCL_LIBRARIES} glog::glog ${yaml-cpp_LIBRARIES} yaml-cpp ${OpenCV_LIBS})

enable_testing()

add_executable(test test.cpp)
target_link_libraries(test curveModel_lib ${GTEST_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIBRARIES} pthread)

add_executable(outputPoints outputPoints.cpp)
target_link_libraries(outputPoints curveModel_lib ${PCL_LIBRARIES} glog::glog)

add_executable(evTest evTest.cpp)
target_link_libraries(evTest curveModel_lib ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(wh_ev wh_ev.cpp)
target_link_libraries(wh_ev curveModel_lib ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${GTEST_LIBRARIES} pthread)