/**
 * @file   test.cpp
 * @brief  Test.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#include "curve_modeling.h"
#include "loadPCD.hpp"
#include "bSpline.hpp"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <memory>
#include <fstream>
#include <ceres/ceres.h>

DEFINE_string(input_pcd, "/home/zyp/HD2/DATA/Transmisson/0912/test5/filter.pcd", "输入的点云");
DEFINE_string(yaml, "/home/zyp/Lidar/LC-CurveModel/config/model1.yaml", "yaml文件");
DEFINE_bool(visualize, false, "是否可视化");

/// @brief 测试加载pcd文件是否正常
TEST(loadPcdFile, loadPcd)
{
    std::string pcd_file = FLAGS_input_pcd;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    lc_core::LoadPCD input_pcd;
    input_pcd(pcd_file, cloud);
    LOG(INFO) << "input points num: " << input_pcd.getPoints().size();
    EXPECT_EQ(input_pcd.getPoints().empty(), false);
}

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    // testing::InitGoogleTest(&argc, argv);
    // RUN_ALL_TESTS();
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    lc_core::CurveModeling curve_modeling(FLAGS_yaml);
    // curve_modeling.optimizationEx();
    lc_core::LoadPCD input_pcd;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    input_pcd(FLAGS_input_pcd, cloud);
    curve_modeling.loadLidarPoints(input_pcd);
    // curve_modeling.optimizationEx();
    
    curve_modeling.lidarPreprocessing();
    curve_modeling.optimization();
    if (FLAGS_visualize) {
        curve_modeling.visualization();
    }

    return 0;
}
