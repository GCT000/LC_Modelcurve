/**
 * @file   image_generation.cpp
 * @brief  This file is used to generate simulated-image.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#include "camera.h"
#include "output.h"

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <gflags/gflags.h>
#include <memory>
#include <vector>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>

DEFINE_string(camera_yaml_file, "/home/zyp/Lidar/LC-CurveModel/simulation/config/cam0.yaml", "相机内参");
DEFINE_string(input_points_file, "/home/zyp/Lidar/LC-CurveModel/simulation/data/points.txt", "输入点");
DEFINE_string(output_image, "/home/zyp/Lidar/LC-CurveModel/simulation/data/image.png", "输出图像");
DEFINE_bool(debug, false, "是否输出点坐标调试信息");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::WARNING;
    FLAGS_colorlogtostderr = true;

    std::string camera_yaml_file = FLAGS_camera_yaml_file;
    std::string input_points_file = FLAGS_input_points_file;
    std::string output_file = FLAGS_output_image;

    // initialize camera
    std::shared_ptr<lc_core::Camera> camera = std::make_shared<lc_core::Camera>();
    if (!camera->readIntrinsicParameters(camera_yaml_file))
    {
        LOG(ERROR) << "Read camera intrinsic parameters failed!";
        return -1;
    }

    // rotation matrix
    Eigen::Matrix3d R;
    R << 0, -1, 0,
         0, 0, -1,
         1, 0, 0;

    // read input points
    std::fstream file(input_points_file, std::ios::in);
    if (!file.is_open())
    {
        LOG(ERROR) << "Open input points file failed!";
        return -1;
    }

    std::vector<Eigen::Vector3d> points;
    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        double x, y, z;
        iss >> x >> y >> z;
        points.push_back(R * Eigen::Vector3d(x, y, z) + Eigen::Vector3d(-0.5, 0.2, 0.0));
    }
    file.close();
    if (FLAGS_debug) lc_core::outputPoints("/home/zyp/Lidar/LC-CurveModel/simulation/data/camera_points.txt", points);

    std::vector<cv::Point2f> image_points;
    for (auto p : points)
    {
        Eigen::Vector2d image_point;
        camera->spaceToPlane(p, image_point);
        if (image_point.y() < 0 || image_point.y() > camera->img_h_ || image_point.x() < 0 || image_point.x() > camera->img_w_)
        {
            continue;
        }
        image_points.push_back(cv::Point2f(image_point.x(), image_point.y()));
    }
    if (FLAGS_debug) lc_core::outputPoints("/home/zyp/Lidar/LC-CurveModel/simulation/data/image_points.txt", image_points);

    // generate image
    cv::Mat image = cv::Mat::zeros(camera->img_h_, camera->img_w_, CV_8UC3);
    for (auto ip : image_points)
    {
        image.at<cv::Vec3b>(ip.y, ip.x) = cv::Vec3b(255, 255, 255); 
    }

    cv::imwrite(output_file, image);

    return 0;
}