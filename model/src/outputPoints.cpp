#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <random>
#include <gflags/gflags.h>

DEFINE_string(lidar_points, "/home/zyp/Lidar/LC-CurveModel/build/output_lidar_points.txt", "要添加的lidar点");
DEFINE_string(input_pcd, "/home/zyp/HD2/DATA/Transmisson/0912/test5/extracted_points.pcd", "输入的点云");

// 读取 txt 文件中的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr loadTxtFile(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        std::cerr << "Could not open file: " << filename << std::endl;
        return cloud;
    }

    std::string line;
    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        float x, y, z;
        if (!(iss >> x >> y >> z)) { break; } // 读取x, y, z坐标
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->points.push_back(point);
    }

    inputFile.close();
    return cloud;
}

// 在每个点周围生成100个随机点（半径4cm内）
pcl::PointCloud<pcl::PointXYZ>::Ptr generateRandomPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float radius, int num_points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr random_points(new pcl::PointCloud<pcl::PointXYZ>);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-radius, radius); // 随机生成范围为 [-radius, radius]

    for (const auto& point : cloud->points) {
        for (int i = 0; i < num_points; ++i) {
            pcl::PointXYZ random_point;
            random_point.x = point.x + dis(gen);
            random_point.y = point.y + dis(gen);
            random_point.z = point.z + dis(gen);
            // 确保生成的点在4cm球体半径内
            if (std::sqrt((random_point.x - point.x) * (random_point.x - point.x) +
                          (random_point.y - point.y) * (random_point.y - point.y) +
                          (random_point.z - point.z) * (random_point.z - point.z)) <= radius) {
                random_points->points.push_back(random_point);
            }
        }
    }

    return random_points;
}

int main() {
    // 1. 读取 extracted.pcd 文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(FLAGS_input_pcd, *cloud) == -1) {
        PCL_ERROR("Couldn't read extracted.pcd file \n");
        return (-1);
    }

    // 2. 读取 output_lidar_points.txt 文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr txt_cloud = loadTxtFile(FLAGS_lidar_points);
    
    // 3. 为 txt_cloud 中的每个点生成 100 个随机点，半径为4cm
    pcl::PointCloud<pcl::PointXYZ>::Ptr random_points = generateRandomPoints(txt_cloud, 0.04, 100);

    // 4. 合并点云
    *cloud += *txt_cloud;       // 添加原始 txt 文件的点
    *cloud += *random_points;   // 添加生成的随机点

    // 5. 保存合并后的点云
    pcl::io::savePCDFileASCII("merged_cloud.pcd", *cloud);
    std::cout << "Saved merged point cloud to merged_cloud.pcd" << std::endl;

    return 0;
}
