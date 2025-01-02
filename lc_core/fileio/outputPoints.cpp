#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <random>
#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(lidar_points, "/home/zyp/Lidar/LC-CurveModel/temp/final_output_lidar_points.txt", "要添加的lidar点");
// DEFINE_string(input_pcd, "/home/zyp/HD2/DATA/Transmisson/0912/test5/extracted_points.pcd", "输入的点云");
DEFINE_string(pcd_path, "/ssd/DATA/Transmisson/PJ/data/20241226_0340/extracted/", "输入的点云");
DEFINE_string(output_pcd, "line_1.pcd", "输出的点云");

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

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    // testing::InitGoogleTest(&argc, argv);
    // RUN_ALL_TESTS();
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 1. load txt file
    pcl::PointCloud<pcl::PointXYZ>::Ptr txt_cloud = loadTxtFile(FLAGS_lidar_points);
    
    // 2. generate random points in a radius
    pcl::PointCloud<pcl::PointXYZ>::Ptr random_points = generateRandomPoints(txt_cloud, 0.03, 20);

    // 3. merge point cloud
    *cloud = *txt_cloud;        // original txt file points
    *cloud += *random_points;   // add generated random points
    // 4. save merged point cloud
    std::string output_file = FLAGS_pcd_path + FLAGS_output_pcd;
    pcl::io::savePCDFileASCII(output_file, *cloud);
    LOG(INFO) << "Saved merged point cloud to " << output_file;

    return 0;
}
