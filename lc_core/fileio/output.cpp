#include "output.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <random>

namespace lc_core
{

// load txt file
pcl::PointCloud<pcl::PointXYZ>::Ptr loadTxtFile(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        LOG(ERROR) << "Could not open file: " << filename;
        return cloud;
    }

    std::string line;
    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        float x, y, z;
        if (!(iss >> x >> y >> z)) { break; } // x, y, z
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->points.push_back(point);
    }

    inputFile.close();
    return cloud;
}

// generate random points in a radius
pcl::PointCloud<pcl::PointXYZ>::Ptr generateRandomPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float radius, int num_points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr random_points(new pcl::PointCloud<pcl::PointXYZ>);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-radius, radius); // random generate range is [-radius, radius]

    for (const auto& point : cloud->points) {
        for (int i = 0; i < num_points; ++i) {
            pcl::PointXYZ random_point;
            random_point.x = point.x + dis(gen);
            random_point.y = point.y + dis(gen);
            random_point.z = point.z + dis(gen);
            // ensure the generated point is in the 3cm sphere radius
            if (std::sqrt((random_point.x - point.x) * (random_point.x - point.x) +
                          (random_point.y - point.y) * (random_point.y - point.y) +
                          (random_point.z - point.z) * (random_point.z - point.z)) <= radius) {
                random_points->points.push_back(random_point);
            }
        }
    }

    return random_points;
}

void outputPoints(const std::string &file_name, const std::vector<Eigen::Vector3d> &points)
{
    std::ofstream out_file(file_name, std::ios::out);
    if (!out_file.is_open())
    {
        LOG(ERROR) << "Can't open file: " << file_name;
        return;
    }

    for (const auto &point : points)
    {
        out_file << point.x() << " " << point.y() << " " << point.z() << std::endl;
    }

    out_file.close();
}

void outputPoints(const std::string &file_name, const std::vector<cv::Point2d> &points)
{
    std::ofstream out_file(file_name, std::ios::out);
    if (!out_file.is_open())
    {
        LOG(ERROR) << "Can't open file: " << file_name;
        return;
    }

    for (const auto &point : points)
    {
        out_file << point.x << " " << point.y << std::endl;
    }

    out_file.close();
}

void outputPCD(const std::string &input_file, const std::string &output_file)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 1. load txt file
    pcl::PointCloud<pcl::PointXYZ>::Ptr txt_cloud = loadTxtFile(input_file);
    
    // 2. generate random points in a radius
    pcl::PointCloud<pcl::PointXYZ>::Ptr random_points = generateRandomPoints(txt_cloud, 0.03, 20);

    // 3. merge point cloud
    *cloud = *txt_cloud;        // original txt file points
    *cloud += *random_points;   // add generated random points
    // 4. save merged point cloud
    pcl::io::savePCDFileASCII(output_file, *cloud);
    LOG(INFO) << "Saved merged point cloud to " << output_file;
}

}