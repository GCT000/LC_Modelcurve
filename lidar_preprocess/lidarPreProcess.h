/**
 * @file    lidarPreProcess.h
 * @brief   This file defines the lidar preprocessing class
 * @author  Yipeng Zhao
 * @date    2024-10
 */

#ifndef LIDAR_PREPROCESS_H
#define LIDAR_PREPROCESS_H

#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

/// @brief  Lidar preprocessing configuration
struct LidarPPConfig
{
    double radius_threshold;
    double linear_threshold;
    double x_threshold;
    double z_threshold;

    LidarPPConfig(double _radius_threshold, double _linear_threshold,
                  double _x_threshold, double _z_threshold) : radius_threshold(_radius_threshold), linear_threshold(_linear_threshold),
                                                              x_threshold(_x_threshold), z_threshold(_z_threshold) {}
};

class LidarPreProcess
{
public:
    LidarPreProcess(const LidarPPConfig &config) : config_(config) {}

    ~LidarPreProcess() = default;

    /// @brief  main function
    void operator()(const std::string &file_name);

    /// @brief  data interface
    std::vector<std::vector<Eigen::Vector3d>> points() const {
        return separated_lines_;
    }

private:
    /// @brief  filter ground points and far away points
    void filterPoints();

    /// @brief  filter linear points
    void filterLinearPoints();

    /// @brief  get neighborhood
    std::vector<Eigen::Vector3d> getNeighborhood(const Eigen::Vector3d &point);

    /// @brief  calculate covariance matrix
    Eigen::Matrix3d calculateCovarianceMatrix(const std::vector<Eigen::Vector3d> &neighborhood);

    /// @brief  calculate eigenvalues and eigenvectors
    void calculateEigenvaluesAndEigenvectors(const Eigen::Matrix3d &M,
                                             Eigen::Vector3d &eigenvalues, Eigen::Matrix3d &eigenvectors);

    /// @brief  calculate dimensionality features
    Eigen::Vector3d calculateDimensionalityFeatures(const Eigen::Vector3d &eigenvalues);

    /// @brief  separate power lines
    void separatePowerLines();

    /// @brief  k-means clustering
    std::vector<double> kMeansCluster(const std::vector<double> &data, int k, int max_iterations = 100);

    /// @brief  visualize separated lines
    void visualizeSeparatedLines(const std::vector<std::vector<Eigen::Vector3d>> &separated_lines);

    /// @brief  perform DBSCAN clustering
    std::vector<std::vector<Eigen::Vector3d>> performDBSCANClustering(
        const std::vector<Eigen::Vector3d>& points, 
        double clusterTolerance, 
        int minClusterSize, 
        int maxClusterSize);

    /// @brief  perform custom clustering
    std::vector<std::vector<Eigen::Vector3d>> performCustomClustering(
        const std::vector<Eigen::Vector3d>& points, 
        double yzClusterTolerance,
        double xClusterTolerance, 
        int minClusterSize, 
        int maxClusterSize);

private:
    LidarPPConfig config_;
    std::vector<Eigen::Vector3d> input_points_;
    std::vector<std::vector<Eigen::Vector3d>> separated_lines_;
};

#endif // LIDAR_PREPROCESS_H
