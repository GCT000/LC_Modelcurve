/**
 * @file   curve_modeling.h
 * @brief  This file contains 3D-curve-line modeling.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#ifndef CURVE_MODELING_H
#define CURVE_MODELING_H

#include "camera.hpp"
#include "curve.hpp"
#include "output.hpp"
#include "loadPCD.hpp"
#include "ex_optimization.h"
#include "matcher.h"

#include <iostream>
#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <execution>
#include <thread>

class CurveModeling
{
public:
    CurveModeling(const std::string &yaml_file);

    ~CurveModeling();

    /// @brief  load lidar points from file
    void loadLidarPoints(const std::string &lidar_points_file);
    void loadLidarPoints(const LoadPCD &load_pcd);

    /// @brief  merge lidar points when line number bigger than 1
    void mergeLidarPoints(const std::vector<Eigen::Vector3d>& lidar_points);

    /// @brief  load camera from file
    void loadCamera(const YAML::Node &yaml, const std::string &yaml_file);

    /// @brief  load lidar2camera extrinsic Parameters
    void loadLidar2CameraExtrinsic(const YAML::Node &yaml);
    
    /// @brief  lidar preprocessing
    void lidarPreprocessing();
    
    /// @brief  fitting 3D-curve-line with input lidar points
    void curveLidarFitting();

    /// @brief  generate points based on curve parameters
    void generateCurvePoints();

    /// @brief  generate curve points on image
    void generateCurveImagePoints(const std::string& selected_points);

    /// @brief  generate line points on image
    void generateLineImagePoints(const cv::Point2d &start, const cv::Point2d &end);

    /// @brief  for debug visualization
    void visualization();

    /// @brief  optimization
    void optimization();
    
    /// @brief  optimize 3D-curve-points
    void optimization3DCurve(const P2LMatchResult& lines);
    void optimization3DCurve(const P2PMatchResult& points, int time = 1);

    /// @brief  update match and re-optimization
    void updateMatchAndReOptimization();

    /// @brief  optimize ex
    void optimizationEx();

    /// @brief  debug
    void lidarP2img();

private:
    std::vector<Eigen::Vector3d> lidar_points_;
    std::shared_ptr<Camera> cam_;
    std::shared_ptr<Matcher> matcher_;
    bool merge_;
    // Eigen::Vector3d end_point_;
    double x_interval_start_;
    double x_interval_end_;
    double x_interval_sample_start_;
    cv::Mat img_;
    Eigen::Matrix3d R_c_l_;
    Eigen::Vector3d t_c_l_;

    double plane_param_[1][2];
    double mesh_param_[1][3];

    std::vector<cv::Point2d> img_points_;
    std::vector<cv::Point2d> ori_lidar2img_points_;

    std::shared_ptr<ExOptimization> ex_optimization_;
};

/// @brief  transformation struct
struct Trans{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    Trans() = default;

    Trans(const Eigen::Matrix3d& _R, const Eigen::Vector3d& _t) 
        : R(_R), t(_t) {}

    void inverse() {
        R.transposeInPlace();
        t = -R * t;
    }
};

#endif