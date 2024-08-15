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

DEFINE_string(input_pcd, "/home/zyp/DATA/lidar/0809/filter_1.pcd", "输入的点云");
DEFINE_string(yaml, "/home/zyp/Lidar/LC-CurveModel/config/model.yaml", "yaml文件");
DEFINE_string(points_selected, "/home/zyp/Lidar/LC-CurveModel/tools/points.txt", "在图像上选择的点");
DEFINE_bool(visualize, false, "是否可视化");

/// @brief 测试加载pcd文件是否正常
TEST(loadPcdFile, loadPcd)
{
    std::string pcd_file = FLAGS_input_pcd;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    LoadPCD input_pcd;
    input_pcd(pcd_file, cloud);
    std::cout << "input points num: " << input_pcd.getPoints().size() << std::endl;
    EXPECT_EQ(input_pcd.getPoints().empty(), false);
}

/// @brief  人为修改外参（TODO），这个方法还不够准，外参精度还是比较重要的
void modifyEx(const Eigen::Vector3d& eulerAngle, const cv::Mat& input, const LoadPCD& input_pcd, std::shared_ptr<Camera> cam) {
    int total = 672;
    int index = 1;
    for (double dy = -0.15; dy <= -0.08; dy += 0.01) {
        for (double dx = -0.05; dx <= 0.00; dx += 0.01) {
            Eigen::Matrix3d R_cur;
            R_cur = Eigen::AngleAxisd(eulerAngle(2) + 0.01, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(eulerAngle(1) + dy, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(eulerAngle(0) + dx, Eigen::Vector3d::UnitX());
            Eigen::Vector3d t_l_c(0.07, 0.52, -0.27);

            for (double z = 0.07; z <= 0.20; z += 0.01) {
                cv::Mat cur = input.clone();
                Eigen::Vector3d t = t_l_c;
                t(2) += z;

                Trans Tcl(R_cur, t);
                Tcl.inverse();

                for (const Eigen::Vector3d& p : input_pcd.getPoints()) {
                    Eigen::Vector3d p_c = Tcl.R * p + Tcl.t;
                    Eigen::Vector2d p_img;
                    cam->spaceToPlane(p_c, p_img);
                    cv::circle(cur, cv::Point(p_img(0), p_img(1)), 1, cv::Scalar(0, 0, 255), -1);
                }

                cv::imwrite("results/result_" + 
                    std::to_string(1) + "_" + 
                    std::to_string(std::round(dy * 100)) + "_" + 
                    std::to_string(std::round(dx * 100)) + "_" + 
                    std::to_string(std::round(z * 100)) + ".jpg", cur);

                LOG(INFO) << index << "/" << total;
                index++;
            }
        }
    }
}

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    // testing::InitGoogleTest(&argc, argv);
    // RUN_ALL_TESTS();
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    
    CurveModeling curve_modeling(FLAGS_yaml);
    LoadPCD input_pcd;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    input_pcd(FLAGS_input_pcd, cloud);
    curve_modeling.loadLidarPoints(input_pcd);
    
    curve_modeling.curveLidarFitting();
    curve_modeling.optimization();
    if (FLAGS_visualize) {
        curve_modeling.visualization();
    }

    return 0;
}
