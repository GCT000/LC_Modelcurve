/**
 * @file   image_generation.cpp
 * @brief  This file is used to detect curve based on simulated-image.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#include "curve.h"
#include "output.h"

#include <gflags/gflags.h>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>

DEFINE_string(input_image, "/home/zyp/Lidar/LC-CurveModel/simulation/data/image.png", "输入图像");
DEFINE_bool(visualize, false, "是否可视化");
DEFINE_bool(debug, false, "是否输出调试信息");
DEFINE_bool(resize, false, "是否调整图像大小");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    std::string input_image = FLAGS_input_image;

    auto curve = std::make_shared<lc_core::Curve>(FLAGS_visualize);
    cv::Mat image = cv::imread(input_image);
    if (FLAGS_resize)   cv::resize(image, image, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);
    curve->curveDetection(image);
    // curve->featuresDetection(image);

    std::vector<std::vector<cv::Point>> points = curve->getCurveLines();

    if (FLAGS_debug)
    {
        for (int i = 0; i < points.size(); i++)
        {
            std::string file_name = "/home/zyp/Lidar/LC-CurveModel/simulation/data/curveline" + std::to_string(i + 1) + ".txt";
            lc_core::outputPoints(file_name, points[i]);
        }

        // generate image
        cv::Mat image = cv::Mat::zeros(720, 1224, CV_8UC3);
        for (auto pts : points)
        {
            for (auto ip : pts)
            {
                image.at<cv::Vec3b>(ip.y, ip.x) = cv::Vec3b(255, 255, 255); 
            }               
        }

        cv::imshow("curve", image);
        cv::waitKey(0);
    }


    google::ShutdownGoogleLogging();
    return 0;
}