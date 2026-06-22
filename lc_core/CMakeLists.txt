 include_directories(model)
include_directories(factor)
include_directories(utils)
include_directories(matcher)
include_directories(transmission_model)
include_directories(../lc_preprocess/img_preprocess)
include_directories(cal_distance)     
include_directories(kf_filter)       
include_directories(optical_flow)    
include_directories(generate_curve)                    
include_directories(line_extractor)        
#include_directories(${CMAKE_SOURCE_DIR}/lc_core)


add_library(curveModel_lib SHARED
    camera/camera.cpp
    fileio/output.cpp
    matcher/matcher.cpp
    matcher/kdtree.cpp
    model/curve_modeling.cpp
    transmission_model/catenary.cpp
    transmission_model/transmissionModel.cpp
    utils/ex_optimization.cpp
    utils/evaluation.cpp
    utils/visualization.cpp
    ../lc_preprocess/img_preprocess/imgPreProcess.cpp
    line_extractor/line_extractor.cpp
)

add_library(imgprocess SHARED
../lc_preprocess/img_preprocess/imgPreProcess.cpp
)

add_library(camera SHARED
    camera/camera.cpp
)

add_library(cal_distance_lib SHARED
    cal_distance/cal_distance.cpp
)

add_executable(generate_curve generate_curve/generate_curve.cpp)
target_link_libraries(generate_curve  ${OpenCV_LIBS} Ceres::ceres)


target_link_libraries(curveModel_lib PRIVATE Ceres::ceres ${PCL_LIBRARIES} ${VTK_LIBRARIES} glog::glog yaml-cpp::yaml-cpp ${OpenCV_LIBS})
target_link_libraries(cal_distance_lib )

add_executable(kf_filter kf_filter/filter.cpp)
target_link_libraries(kf_filter Ceres::ceres ${PCL_LIBRARIES} ${VTK_LIBRARIES} glog::glog  yaml-cpp::yaml-cpp)
add_executable(optical_flow optical_flow/optical_flow.cpp)
target_link_libraries(optical_flow glog::glog camera yaml-cpp::yaml-cpp imgprocess ${OpenCV_LIBS} Ceres::ceres)

enable_testing()

add_executable(curve test.cpp)
target_link_libraries(curve curveModel_lib cal_distance_lib GTest::gtest ${OpenCV_LIBS} TBB::tbb pthread glog::glog)

include_directories(daemon)
add_executable(lc_daemon daemon/lc_daemon.cpp)
target_link_libraries(lc_daemon yaml-cpp::yaml-cpp glog::glog gflags ${PCL_LIBRARIES})