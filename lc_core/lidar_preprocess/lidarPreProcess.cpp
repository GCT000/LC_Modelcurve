#include "lidarPreProcess.h"
#include <glog/logging.h>
#include <execution>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <random>
#include <thread>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <algorithm>
#include <numeric>

using namespace lc_core;

void LidarPreProcess::operator()(const std::string &file_name)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // load pcd file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", file_name.c_str());
        return;
    }

    for (auto &point : cloud->points)
    {
        input_points_.emplace_back(point.x, point.y, point.z);
    }
    // filter ground points and far away points
    filterPoints();

    // filter linear points
    filterLinearPoints();

    // separate power lines
    separatePowerLines();
}

void LidarPreProcess::filterPoints()
{
    std::vector<Eigen::Vector3d> filtered_points;
    filtered_points.reserve(input_points_.size() / 3);
    
    // use copy_if to filter points
    // parallel and unsequenced
    std::copy_if(std::execution::par_unseq, 
                 input_points_.begin(), 
                 input_points_.end(),
                 std::back_inserter(filtered_points),
                 [&](const auto &point) {
                     return point.x() > 10.0 && 
                            point.x() < config_.x_threshold && 
                            point.z() > config_.z_threshold;
                 });
    
    input_points_ = std::move(filtered_points);
    LOG(INFO) << "After filtering ground points and far away points, size: " << input_points_.size();
}

void LidarPreProcess::filterLinearPoints()
{
    std::vector<Eigen::Vector3d> filtered_points;

    for (const auto &point : input_points_)
    {
        // get neighborhood
        std::vector<Eigen::Vector3d> neighborhood = getNeighborhood(point);

        // calculate covariance matrix
        Eigen::Matrix3d M = calculateCovarianceMatrix(neighborhood);

        // calculate eigenvalues and eigenvectors
        Eigen::Vector3d eigenvalues;
        Eigen::Matrix3d eigenvectors;
        calculateEigenvaluesAndEigenvectors(M, eigenvalues, eigenvectors);

        // calculate dimensionality features
        Eigen::Vector3d dim_features = calculateDimensionalityFeatures(eigenvalues);
        // LOG(INFO) << "dimensionality features: " << dim_features.transpose();

        // keep the point according to the dimensionality features
        if (dim_features[2] > config_.linear_threshold)
        {
            filtered_points.push_back(point);
        }
    }

    // update input_points_ with filtered points
    input_points_ = std::move(filtered_points);
    LOG(INFO) << "after filter linear points size: " << input_points_.size();
}

std::vector<Eigen::Vector3d> LidarPreProcess::getNeighborhood(const Eigen::Vector3d &point)
{
    std::vector<Eigen::Vector3d> neighborhood;
    neighborhood.reserve(std::min(static_cast<size_t>(100), input_points_.size()));

    // use C++17 parallel algorithm to accelerate
    std::mutex mtx;
    std::for_each(std::execution::par_unseq, input_points_.begin(), input_points_.end(),
                  [&](const auto &p)
                  {
                      if ((p - point).norm() < config_.radius_threshold)
                      {
                          std::lock_guard<std::mutex> lock(mtx);
                          neighborhood.emplace_back(p);
                      }
                  });

    return neighborhood;
}

Eigen::Matrix3d LidarPreProcess::calculateCovarianceMatrix(const std::vector<Eigen::Vector3d> &points)
{
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &p : points) {
        mean += p;
    }
    mean /= points.size();

    // calculate covariance matrix
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    std::mutex mtx;
    std::for_each(std::execution::par_unseq, points.begin(), points.end(),
                  [&](const auto &p) {
                      Eigen::Vector3d centered = p - mean;
                      Eigen::Matrix3d temp = centered * centered.transpose();
                      std::lock_guard<std::mutex> lock(mtx);
                      cov += temp;
                  });
    cov /= points.size();

    return cov;
}

void LidarPreProcess::calculateEigenvaluesAndEigenvectors(const Eigen::Matrix3d &M, 
                                                        Eigen::Vector3d &eigenvalues, Eigen::Matrix3d &eigenvectors)
{
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(M);
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors = eigensolver.eigenvectors();
}

Eigen::Vector3d LidarPreProcess::calculateDimensionalityFeatures(const Eigen::Vector3d &eigenvalues)
{
    // Calculate the probability of conforming to the three spatial dimensional feature divisions
    Eigen::Vector3d features;
    features << (sqrt(eigenvalues[0]) - sqrt(eigenvalues[1])) / sqrt(eigenvalues[0]),
                (sqrt(eigenvalues[1]) - sqrt(eigenvalues[2])) / sqrt(eigenvalues[0]),
                sqrt(eigenvalues[2]) / sqrt(eigenvalues[0]);

    return features;
}

std::vector<std::vector<Eigen::Vector3d>> LidarPreProcess::performCustomClustering(
    const std::vector<Eigen::Vector3d>& points, 
    double yzClusterTolerance,
    double xClusterTolerance, 
    int minClusterSize, 
    int maxClusterSize)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(points.size());
    for (const auto& point : points)
    {
        cloud->points.emplace_back(point.x(), point.y(), point.z());
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::ConditionalEuclideanClustering<pcl::PointXYZ> cec;
    cec.setClusterTolerance(yzClusterTolerance);
    cec.setMinClusterSize(minClusterSize);
    cec.setMaxClusterSize(maxClusterSize);
    cec.setInputCloud(cloud);
    cec.setConditionFunction(
        [xClusterTolerance, yzClusterTolerance](const pcl::PointXYZ& point1, const pcl::PointXYZ& point2, float squared_distance) {
            float dx = std::abs(point1.x - point2.x);
            float dy = std::abs(point1.y - point2.y);
            float dz = std::abs(point1.z - point2.z);
            return (dx <= xClusterTolerance) && (std::sqrt(dy*dy + dz*dz) <= yzClusterTolerance);
        }
    );
    cec.segment(cluster_indices);

    std::vector<std::vector<Eigen::Vector3d>> clusters;
    for (const auto& indices : cluster_indices)
    {
        std::vector<Eigen::Vector3d> cluster;
        for (int index : indices.indices)
        {
            cluster.push_back(points[index]);
        }
        clusters.push_back(cluster);
    }

    return clusters;
}

void LidarPreProcess::separatePowerLines()
{
    // Use custom clustering method
    std::vector<std::vector<Eigen::Vector3d>> rough_separated_lines = performCustomClustering(input_points_, 0.5, 2.0, 100, 25000);

    // Further separate each group of lines based on Y values
    std::vector<std::vector<Eigen::Vector3d>> fine_separated_lines;
    for (const auto& line : rough_separated_lines)
    {
        std::vector<double> y_values;
        y_values.reserve(line.size());
        for (const auto& point : line)
        {
            y_values.push_back(point.y());
        }

        // Use K-means clustering to separate Y values
        std::vector<double> centroids = kMeansCluster(y_values, 2);

        std::vector<Eigen::Vector3d> line1, line2;
        for (const auto& point : line)
        {
            if (std::abs(point.y() - centroids[0]) < std::abs(point.y() - centroids[1]))
            {
                line1.push_back(point);
            }
            else
            {
                line2.push_back(point);
            }
        }

        // Only add lines if they have enough points
        if (line1.size() > 200) fine_separated_lines.push_back(line1);
        if (line2.size() > 200) fine_separated_lines.push_back(line2);
    }

    // Output results
    LOG(INFO) << "Separated " << fine_separated_lines.size() << " power lines";
    for (size_t i = 0; i < fine_separated_lines.size(); ++i)
    {
        LOG(INFO) << "Power line " << i + 1 << " contains " << fine_separated_lines[i].size() << " points";
    }

    separated_lines_ = std::move(fine_separated_lines);

    // Visualize results
    // visualizeSeparatedLines(fine_separated_lines);
}

std::vector<double> LidarPreProcess::kMeansCluster(const std::vector<double>& data, int k, int max_iterations)
{
    std::vector<double> centroids(k);
    // Initialize centroids
    double min_val = *std::min_element(data.begin(), data.end());
    double max_val = *std::max_element(data.begin(), data.end());
    for (int i = 0; i < k; ++i)
    {
        centroids[i] = min_val + (max_val - min_val) * i / (k - 1);
    }

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        std::vector<std::vector<double>> clusters(k);

        // Assign points to the nearest centroid
        for (const auto& point : data)
        {
            int closest_centroid = 0;
            double min_distance = std::abs(point - centroids[0]);
            for (int j = 1; j < k; ++j)
            {
                double distance = std::abs(point - centroids[j]);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    closest_centroid = j;
                }
            }
            clusters[closest_centroid].push_back(point);
        }

        // Update centroids
        bool centroids_changed = false;
        for (int i = 0; i < k; ++i)
        {
            if (!clusters[i].empty())
            {
                double new_centroid = std::accumulate(clusters[i].begin(), clusters[i].end(), 0.0) / clusters[i].size();
                if (std::abs(new_centroid - centroids[i]) > 1e-6)
                {
                    centroids[i] = new_centroid;
                    centroids_changed = true;
                }
            }
        }

        if (!centroids_changed)
        {
            break;
        }
    }

    return centroids;
}

void LidarPreProcess::visualizeSeparatedLines(const std::vector<std::vector<Eigen::Vector3d>> &separated_lines)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Power Lines Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    for (size_t i = 0; i < separated_lines.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // Generate a random color for each line
        int r = dis(gen);
        int g = dis(gen);
        int b = dis(gen);

        for (const auto& point : separated_lines[i])
        {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x();
            colored_point.y = point.y();
            colored_point.z = point.z();
            colored_point.r = r;
            colored_point.g = g;
            colored_point.b = b;
            cloud->points.push_back(colored_point);
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;

        std::string cloud_name = "power_line_" + std::to_string(i);
        viewer->addPointCloud(cloud, cloud_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
    }

    LOG(INFO) << "Visualizing separated power lines. Press 'q' to close the viewer.";
    
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
