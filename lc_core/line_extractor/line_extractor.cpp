#include "line_extractor.h"

LineExtractor::LineExtractor()
    : line1_cloud_(new PointCloudT),
      line2_cloud_(new PointCloudT),
      line_angle_(0.0) {}

double LineExtractor::deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

double LineExtractor::rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}

bool LineExtractor::calculateAvgYAndZ(const PointCloudT::Ptr &cloud,
                                      double &avg_y,
                                      double &avg_z)
{
    if (!cloud || cloud->empty())
    {
        avg_y = 0.0;
        avg_z = 0.0;
        std::cerr << "Error: Point cloud is empty or null!" << std::endl;
        return false;
    }

    // 1. 定义旋转参数：围绕X轴逆时针旋转10度
    const double rotate_angle_deg = -12.0;
    const double rotate_angle_rad = rotate_angle_deg * M_PI / 180.0;
    const double cos_theta = cos(rotate_angle_rad);
    const double sin_theta = sin(rotate_angle_rad);

    // 2. 初始化变量：累加值 + 旋转后点云
    double sum_y = 0.0;
    double sum_z = 0.0;
    const size_t point_num = cloud->size();
    PointCloudT::Ptr rotated_cloud(new PointCloudT); // 存储旋转后的完整点云
    rotated_cloud->resize(point_num);                // 预分配内存，提升效率

    // 3. 遍历点云：旋转坐标 + 累加值 + 填充旋转后点云
    for (size_t i = 0; i < point_num; ++i)
    {
        const auto &original_point = cloud->points[i];
        auto &rotated_point = rotated_cloud->points[i];

        // 保留X坐标不变，计算旋转后的Y/Z
        rotated_point.x = original_point.x * cos_theta + original_point.z * sin_theta;
        rotated_point.y = original_point.y; // Y坐标保持不变
        rotated_point.z = -original_point.x * sin_theta + original_point.z * cos_theta;

        // 累加旋转后的Y/Z值用于计算平均值
        sum_y += rotated_point.y;
        sum_z += rotated_point.z;
    }

    // 4. 计算平均值
    avg_y = sum_y / point_num;
    avg_z = sum_z / point_num;

    // // 5. 保存旋转后的点云为PCD文件（如果指定了路径）
    // if (!output_pcd_path.empty())
    // {
    //     // 设置点云的header（可选，保持和原云一致的坐标系）
    //     rotated_cloud->header = cloud->header;
    //     rotated_cloud->width = cloud->width;
    //     rotated_cloud->height = cloud->height;
    //     rotated_cloud->is_dense = cloud->is_dense;

    //     // 保存PCD文件
    //     if (pcl::io::savePCDFileASCII(output_pcd_path, *rotated_cloud) == -1)
    //     {
    //         std::cerr << "Error: Failed to save rotated point cloud to "
    //                   << output_pcd_path << std::endl;
    //         // 保存失败不影响平均值计算，仅打印错误，返回true
    //     }
    //     else
    //     {
    //         std::cout << "Successfully saved rotated point cloud to: "
    //                   << output_pcd_path << std::endl;
    //         std::cout << "Rotated cloud size: " << rotated_cloud->size() << " points" << std::endl;
    //     }
    // }

    return true;
}
float findMinXValue(const PointCloudT::Ptr &cloud)
{
    if (cloud->empty())
    {
        std::cerr << "点云为空，无法获取最小X坐标！" << std::endl;
        // 返回NaN表示无效值
        return std::numeric_limits<float>::quiet_NaN();
    }

    float min_x = cloud->points[0].x;
    // 遍历所有点，仅比较X坐标
    for (const auto &point : cloud->points)
    {
        if (point.x < min_x)
        {
            min_x = point.x;
        }
    }
    return min_x;
}

bool LineExtractor::fit3DLinePCL(const PointCloudT::Ptr &cloud,
                                 double distance_threshold,
                                 int max_iterations,
                                 pcl::PointIndices::Ptr &inliers,
                                 pcl::ModelCoefficients::Ptr &line_coeffs,
                                 Eigen::Vector4d &line_centroid,
                                 Eigen::Vector3d &line_dir)
{
    if (cloud->empty() || cloud->size() < 2)
    {
        std::cerr << "empty cloud" << std::endl;
        return false;
    }
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);
    seg.setMaxIterations(max_iterations);
    seg.setInputCloud(cloud);

    seg.segment(*inliers, *line_coeffs);

    if (inliers->indices.empty())
    {
        std::cerr << "fail to fit line！" << std::endl;
        return false;
    }
    PointCloudT::Ptr inlier_cloud(new PointCloudT);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*inlier_cloud);
    pcl::compute3DCentroid(*inlier_cloud, line_centroid);

    line_dir = Eigen::Vector3d(
                   line_coeffs->values[3],
                   line_coeffs->values[4],
                   line_coeffs->values[5])
                   .normalized();

    return true;
}

// 过滤漏检点的实现
void LineExtractor::filterSameLinePoints(const PointCloudT::Ptr &input_cloud,
                                         const Eigen::Vector4d &line_centroid,
                                         const Eigen::Vector3d &line_dir,
                                         double angle_threshold,
                                         double distance_threshold,
                                         PointCloudT::Ptr &output_cloud)
{
    output_cloud->clear();
    double angle_thresh_rad = deg2rad(angle_threshold);

    Eigen::Vector3d centroid_3d(line_centroid.x(), line_centroid.y(), line_centroid.z());

    for (const auto &point : input_cloud->points)
    {
        Eigen::Vector3d p(point.x, point.y, point.z);
        Eigen::Vector3d vec = p - centroid_3d;
        Eigen::Vector3d cross = vec.cross(line_dir);
        double dist = cross.norm();

        vec.normalize();
        double dot_product = std::abs(vec.dot(line_dir));
        dot_product = std::max(std::min(dot_product, 1.0), -1.0);
        double angle = std::acos(dot_product);

        if (angle > angle_thresh_rad || dist > distance_threshold)
        {
            output_cloud->push_back(point);
        }
    }
}

// 核心接口：提取两条隔离直线的实现
bool LineExtractor::extractTwoLinesIsolated(std::vector<Eigen::Vector3d> &lidar_points_,
                                            std::vector<Eigen::Vector3d> &line_points_,
                                            int flag,
                                            double angle_threshold,
                                            double distance_threshold,
                                            double ransac_dist_thresh,
                                            int ransac_max_iter,
                                            std::string path_)
{
    path = path_;
    // 1. 加载原始点云
    PointCloudT::Ptr cloud_original(new PointCloudT);
    cloud_original->reserve(lidar_points_.size());
    for (const auto &pt : lidar_points_)
    {
        PointT pcl_point;
        pcl_point.x = pt.x();
        pcl_point.y = pt.y();
        pcl_point.z = pt.z();
        cloud_original->push_back(pcl_point);
    }
    std::cout << "原始点云数量: " << cloud_original->size() << std::endl;

    if (cloud_original->empty())
    {
        std::cerr << "PCD文件为空！" << std::endl;
        return false;
    }

    // 2. 拟合第一条线
    std::cout << "提取第一条线（核心特征）..." << std::endl;
    pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr line_coeffs1(new pcl::ModelCoefficients);
    Eigen::Vector4d centroid1;
    Eigen::Vector3d dir1;

    if (!fit3DLinePCL(cloud_original, ransac_dist_thresh, ransac_max_iter,
                      inliers1, line_coeffs1, centroid1, dir1))
    {
        return false;
    }

    // 提取第一条线的点云
    pcl::ExtractIndices<PointT> extract1;
    extract1.setInputCloud(cloud_original);
    extract1.setIndices(inliers1);
    extract1.setNegative(false);
    extract1.filter(*line1_cloud_);
    std::cout << "第一条线提取点数: " << line1_cloud_->size() << std::endl;

    // 3. 提取剩余点云
    PointCloudT::Ptr remaining_cloud(new PointCloudT);
    extract1.setNegative(true);
    extract1.filter(*remaining_cloud);
    std::cout << "第一条线提取后剩余点数: " << remaining_cloud->size() << std::endl;

    // 4. 过滤第一条线的漏检点
    std::cout << "过滤第一条线的漏检点..." << std::endl;
    PointCloudT::Ptr filtered_cloud(new PointCloudT);
    filterSameLinePoints(remaining_cloud, centroid1, dir1,
                         angle_threshold, distance_threshold, filtered_cloud);
    std::cout << "过滤前剩余点数: " << remaining_cloud->size()
              << ", 过滤后剩余点数: " << filtered_cloud->size() << std::endl;

    if (filtered_cloud->empty())
    {
        std::cerr << "过滤后无剩余点，无法提取第二条线！" << std::endl;
        return false;
    }

    // 5. 拟合第二条线
    std::cout << "提取第二条线（无第一条线混入）..." << std::endl;
    pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr line_coeffs2(new pcl::ModelCoefficients);
    Eigen::Vector4d centroid2;
    Eigen::Vector3d dir2;
    if (!fit3DLinePCL(filtered_cloud, ransac_dist_thresh, ransac_max_iter,
                      inliers2, line_coeffs2, centroid2, dir2))
    {
        return false;
    }

    // 提取第二条线的点云
    pcl::ExtractIndices<PointT> extract2;
    extract2.setInputCloud(filtered_cloud);
    extract2.setIndices(inliers2);
    extract2.setNegative(false);
    extract2.filter(*line2_cloud_);
    std::cout << "第二条线提取点数: " << line2_cloud_->size() << std::endl;

    // 6. 计算两条线的夹角
    double dot_product = std::abs(dir1.dot(dir2));
    dot_product = std::max(std::min(dot_product, 1.0), -1.0);
    line_angle_ = rad2deg(std::acos(dot_product));
    std::cout << "两条线的夹角: " << std::fixed << std::setprecision(2) << line_angle_ << " 度" << std::endl;

    double line1_avg_y, line1_avg_z;
    calculateAvgYAndZ(line1_cloud_, line1_avg_y, line1_avg_z);
    double line2_avg_y, line2_avg_z;
    calculateAvgYAndZ(line2_cloud_, line2_avg_y, line2_avg_z);

    // float line1_min_x = findMinXValue(line1_cloud_);
    // // 2. 计算第二条线的最小X值
    // float line2_min_x = findMinXValue(line2_cloud_);
    // 7. 保存结果
    if (flag == 1)
    {
        if (line1_avg_z < line2_avg_z)
        {
            line_points_.reserve(line1_cloud_->size());

            for (const auto &pcl_point : *line1_cloud_)
            {
                Eigen::Vector3d eigen_point(pcl_point.x, pcl_point.y, pcl_point.z);
                line_points_.emplace_back(eigen_point);
            }
            pcl::io::savePCDFileASCII(path + "line1_isolated.pcd", *line1_cloud_);
        }
        else
        {
            line_points_.reserve(line2_cloud_->size());

            for (const auto &pcl_point : *line2_cloud_)
            {
                Eigen::Vector3d eigen_point(pcl_point.x, pcl_point.y, pcl_point.z);
                line_points_.emplace_back(eigen_point);
            }
            pcl::io::savePCDFileASCII(path + "line2_isolated.pcd", *line2_cloud_);
        }
    }
    else
    {
        if (line1_avg_z >= line2_avg_z)
        {
            line_points_.reserve(line1_cloud_->size());

            for (const auto &pcl_point : *line1_cloud_)
            {
                Eigen::Vector3d eigen_point(pcl_point.x, pcl_point.y, pcl_point.z);
                line_points_.emplace_back(eigen_point);
            }
            pcl::io::savePCDFileASCII(path + "line1_isolated.pcd", *line1_cloud_);
        }
        else
        {
            line_points_.reserve(line2_cloud_->size());

            for (const auto &pcl_point : *line2_cloud_)
            {
                Eigen::Vector3d eigen_point(pcl_point.x, pcl_point.y, pcl_point.z);
                line_points_.emplace_back(eigen_point);
            }
            pcl::io::savePCDFileASCII(path + "line2_isolated.pcd", *line2_cloud_);
        }
    }
    return true;
}
