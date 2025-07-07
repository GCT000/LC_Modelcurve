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
#include "imgPreProcess.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <memory>
#include <vector>
#include <cassert>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <execution>
#include <thread>
#include <utility>

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

    /// @brief  is exist end_point
    bool if_no_end_point();

    /// @brief  set end point
    void set_end_point(const Eigen::Vector4f& point);

    /// @brief  get pcd_files and end_point_wgs84
    std::pair<std::vector<std::string>, Eigen::Vector4f> get_files_point();
    
    /// @brief  get res path 
    std::string get_res_path(){return res_path;}

    /// @brief  get cal_distance files
    std::pair<std::string, std::vector<std::string>> get_cal_distance_files();

    /// @brief  get ndt and icp paragram
    std::pair<std::vector<float>, std::vector<float>> get_ndt_icp_para()
    {
        return std::make_pair(ndt,icp);
    }


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

    /// @brief  Optical flow
    void optical_flow();

private:
    std::string curve_point_file;
    std::string res_path;
    std::string raw_pcd_file;

    std::vector<Eigen::Vector3d> lidar_points_;
    std::vector<std::string> cal_dis_output_files;
    std::vector<float> ndt, icp;

    std::shared_ptr<Camera> cam_;
    std::shared_ptr<Matcher> matcher_;
    double y_interval_start_;
    double y_interval_end_;
    cv::Mat img_, last_img_;
    Eigen::Matrix3d R_c_l_;
    Eigen::Vector3d t_c_l_;
    Eigen::Vector3d end_point;
    Eigen::Vector4f end_point_wgs84;
    
    std::vector<std::string> pcd_files;
    std::vector<cv::Point2d> img_points_;
    std::vector<cv::Point2d> ori_lidar2img_points_;

    std::shared_ptr<ExOptimization> ex_optimization_;

    std::shared_ptr<TransmissionModel> transmission_model_;
};

} // namespace lc_core

#endif