#include "lidarPreProcess.h"
#include <random>
#include <iostream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

DEFINE_double(radius_threshold, 0.10, "radius threshold");
DEFINE_double(linear_threshold, 2.50, "linear threshold");
DEFINE_double(x_threshold, 90.0, "x threshold");
DEFINE_double(z_threshold, 0.0, "z threshold");
DEFINE_string(pcd_path, "/home/zyp/HD2/DATA/Transmisson/0912/test5/extracted_points.pcd", "pcd file path");

cv::Mat GenerateBEVImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // compute the min and max x and y
    auto minmax_x = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) { return p1.x < p2.x; });
    auto minmax_y = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) { return p1.y < p2.y; });
    double min_x = minmax_x.first->x;
    double max_x = minmax_x.second->x;
    double min_y = minmax_y.first->y;
    double max_y = minmax_y.second->y;

    const double inv_r = 1.0 / 0.1;

    const int image_rows = int((max_y - min_y) * inv_r);
    const int image_cols = int((max_x - min_x) * inv_r);

    float x_center = 0.5 * (max_x + min_x);
    float y_center = 0.5 * (max_y + min_y);
    float x_center_image = image_cols / 2;
    float y_center_image = image_rows / 2;

    // generate the bev image
    cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(255, 255, 255));

    for (const auto& pt : cloud->points) {
        int x = int((pt.x - x_center) * inv_r + x_center_image);
        int y = int((pt.y - y_center) * inv_r + y_center_image);
        if (x < 0 || x >= image_cols || y < 0 || y >= image_rows) {
            continue;
        }

        image.at<cv::Vec3b>(y, x) = cv::Vec3b(227, 143, 79);    // color the point
    }

    return image;
}

void detectLine(cv::Mat &bev_image) {
    // preprocess the bev image
    cv::Mat gray_image, edges;
    cv::cvtColor(bev_image, gray_image, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 0);
    cv::Canny(gray_image, edges, 10, 30);

    // detect lines using hough transform
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180, 50, 50, 10);

    LOG(INFO) << "detected lines: " << lines.size();
    // draw the detected lines on the original image
    for(size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::line(bev_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1);
    }
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_minloglevel = google::INFO;
    FLAGS_logtostderr = true;

    lc_core::LidarPreProcess pp(lc_core::LidarPPConfig(FLAGS_radius_threshold, FLAGS_linear_threshold, FLAGS_x_threshold, FLAGS_z_threshold));
    pp(FLAGS_pcd_path);

    // make pcl point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->points.reserve(pp.points().size());

    // transform Eigen::Vector3d points to PCL points
    for (const auto& line : pp.points()) {
        // assign a random color to each line
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(0, 255);
        uint8_t r = dis(gen);
        uint8_t g = dis(gen);
        uint8_t b = dis(gen);

        for (const auto& point : line) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x();
            colored_point.y = point.y();
            colored_point.z = point.z();
            
            colored_point.r = r;
            colored_point.g = g;
            colored_point.b = b;
            
            cloud->points.push_back(colored_point);
        }
    }

    // set point cloud width and height
    cloud->width = cloud->points.size();
    cloud->height = 1;

    // bev image is not good
    // cv::Mat bev_image = GenerateBEVImage(cloud);
    // cv::imwrite("./bev.png", bev_image);
    // detectLine(bev_image);
    // cv::imwrite("./bev_with_lines.png", bev_image);

    // save as pcd file
    std::string output_file = "output_cloud.pcd";
    pcl::io::savePCDFileBinary(output_file, *cloud);
    LOG(INFO) << "Saved processed point cloud to file: " << output_file;

    return 0;
}