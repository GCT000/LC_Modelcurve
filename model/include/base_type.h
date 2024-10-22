/**
 * @file   base_type.h
 * @brief  This file defines some common types.
 * @author Yipeng Zhao
 * @date   2024-10
 */

#ifndef BASE_TYPE_H
#define BASE_TYPE_H

#include <Eigen/Dense>
#include <vector>
#include "camera.hpp"

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

struct OptimizationInput {
    std::vector<double> xSamples;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    std::shared_ptr<Camera> cam;

    OptimizationInput(const std::vector<double>& _xSamples, const Eigen::Matrix3d& _R, const Eigen::Vector3d& _t, std::shared_ptr<Camera> _cam)
        : xSamples(_xSamples), R(_R), t(_t), cam(_cam) {}
};


#endif // BASE_TYPE_H