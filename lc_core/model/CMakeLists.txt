include_directories(include)
include_directories(../transmission_model)

# If this macro definition is turned on, 
# the optimization elapsed time is reduced by half the time, 
# but the error at the endpoints increases to the centimeter level.
# add_definitions(-DMAUNAL_DIFF)

add_library(curveModel_lib SHARED
    src/curve_modeling.cpp
    src/ex_optimization.cpp
    src/matcher.cpp
    src/kdtree.cpp
    src/evaluation.cpp
    ../transmission_model/parabola.cpp
    ../transmission_model/catenary.cpp
    ../transmission_model/transmissionModel.cpp
)
target_link_libraries(curveModel_lib ${CERES_LIBRARIES} ${PCL_LIBRARIES} glog::glog ${yaml-cpp_LIBRARIES} yaml-cpp ${OpenCV_LIBS})

enable_testing()

add_executable(test test.cpp)
target_link_libraries(test curveModel_lib ${GTEST_LIBRARIES} ${OpenCV_LIBS} pthread)

add_executable(outputPoints ../fileio/outputPoints.cpp)
target_link_libraries(outputPoints curveModel_lib ${PCL_LIBRARIES} glog::glog)

add_executable(evTest evTest.cpp)
target_link_libraries(evTest curveModel_lib ${CERES_LIBRARIES} ${GTEST_LIBRARIES} pthread)