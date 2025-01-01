/**
 * @file   ex_optimization.h
 * @brief  This file is used for ex optimization.
 * @author Yipeng Zhao
 * @date   2024-09
 */

#ifndef EX_OPTIMIZATION_H
#define EX_OPTIMIZATION_H

#include <ceres/ceres.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include "camera.h"

namespace lc_core
{

class ExOptimization
{
public:
    ExOptimization(Eigen::Matrix3d R_c_l, Eigen::Vector3d t_c_l, std::shared_ptr<Camera> cam)
        : R_c_l_(R_c_l), t_c_l_(t_c_l), cam_(cam)
    {
    }

    ExOptimization(const ExOptimization &other);

    void loadLidarPoints(const std::string& lidar_points_file);

    void loadImgPoints(const std::string& img_ref_points_file);

    void optimization();

    /// @brief  var interface
    Eigen::Matrix3d getR() const { 
        return R_c_l_; 
    }

    Eigen::Vector3d getT() const { 
        return t_c_l_; 
    }

public:
    std::vector<Eigen::Vector3d> lidar_points_;
    std::vector<cv::Point> img_ref_points_;

private:
    Eigen::Matrix3d R_c_l_;
    Eigen::Vector3d t_c_l_;
    std::shared_ptr<Camera> cam_;
};

} // namespace lc_core

#endif  // EX_OPTIMIZATION_H