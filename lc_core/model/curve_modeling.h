/**
 * @file   curve_modeling.h
 * @brief  This file contains 3D-curve-line modeling.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#ifndef CURVE_MODELING_H
#define CURVE_MODELING_H

#include "camera.h"
#include "output.h"
#include "loadPCD.h"
#include "ex_optimization.h"
#include "matcher.h"
#include "parabola.h"
#include "catenary.h"
#include "base_type.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <execution>
#include <thread>

namespace lc_core
{

class CurveModeling
{
public:
    CurveModeling(const std::string &yaml_file);

    ~CurveModeling();

    /// @brief  load lidar points from file
    void loadLidarPoints(const std::string &lidar_points_path);
    
    /// @brief  lidar preprocessing
    void lidarPreprocessing();

    /// @brief  for debug visualization
    void visualization();

    /// @brief  optimization
    void optimization();

    /// @brief  optimize dark
    void optimizationDark();

    /// @brief  optimize ex
    void optimizationEx();

private:
    /// @brief  lidar 2 pixel
    Eigen::Vector2d lidar2pixel(const Eigen::Vector3d& p_l);

    /// @brief  load camera from file
    void loadCamera(const YAML::Node &yaml, const std::string &yaml_file);

    /// @brief  load lidar2camera extrinsic Parameters
    void loadLidar2CameraExtrinsic(const YAML::Node &yaml);

    /// @brief  fitting 3D-curve-line with input lidar points
    void curveLidarFitting();

    /// @brief  generate curve points on image
    void generateCurveImagePoints(const std::string& selected_points);

    /// @brief  generate line points on image
    void generateLineImagePoints(const std::string& selected_points);

    /// @brief  update match and re-optimization
    void updateMatchAndReOptimization(const OptimizationInput& input);

    /// @brief  update lidar project to pixel points
    void updateLidar2PixelPoints();

    /// @brief  output 3D points to txt file
    void output3DPointsToTxt(const std::string& filename);

    /// @brief  debug
    void lidarP2img();

private:
    std::vector<Eigen::Vector3d> lidar_points_;
    std::shared_ptr<Camera> cam_;
    std::shared_ptr<Matcher> matcher_;
    double x_interval_start_;
    double x_interval_end_;
    cv::Mat img_;
    Eigen::Matrix3d R_c_l_;
    Eigen::Vector3d t_c_l_;

    std::vector<cv::Point2d> img_points_;
    std::vector<cv::Point2d> ori_lidar2img_points_;

    std::shared_ptr<ExOptimization> ex_optimization_;

    std::shared_ptr<TransmissionModel> transmission_model_;
};

} // namespace lc_core

#endif