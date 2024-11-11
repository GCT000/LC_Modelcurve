/**
 * @file    curve_factor.h
 * @brief   Curve factor for ceres optimization
 * @author  Yipeng Zhao
 * @date    2024-08
*/

#ifndef CURVE_FACTOR_H
#define CURVE_FACTOR_H

#include <iostream>
#include <ceres/ceres.h>
#include "camera.hpp"
#include "base_type.h"

namespace lc_core
{

class CurveFactor {
public:
    CurveFactor(const std::pair<double, double>& _line, const double& _x, const Trans& _Tcl, std::shared_ptr<Camera> _cam) 
        : line(_line), x(_x), Tcl(_Tcl), cam(_cam), sqrt_info(sqrt(1)) {}

    template <typename T>
    bool operator()(const T* const a, const T* const b, const T* const c, const T* const k, const T* const m, T* residual) const {
        Eigen::Matrix<T, 3, 1> pLidar;
        pLidar << T(x), k[0] * T(x) + m[0], a[0] * T(x) * T(x) + b[0] * T(x) + c[0];
        Eigen::Matrix<T, 3, 1> pCam = Tcl.R.cast<T>() * pLidar + Tcl.t.cast<T>();
        Eigen::Matrix<T, 2, 1> pImg;
        cam->spaceToPlane(pCam, pImg);

        T lineNorm = ceres::sqrt(T(line.first * line.first) + T(1.0));
        residual[0] = T(sqrt_info) * ceres::abs(line.first * pImg(0) - pImg(1) + T(line.second)) / lineNorm;
        return true;
    }

    static ceres::CostFunction* Create(const std::pair<double, double>& _line, const double& _x, const Trans& _Tcl, std::shared_ptr<Camera> _cam) {
        return (new ceres::AutoDiffCostFunction<CurveFactor, 1, 1, 1, 1, 1, 1>(
            new CurveFactor(_line, _x, _Tcl, _cam)));
    }

private:
    std::pair<double, double> line;
    double x;
    Trans Tcl;
    std::shared_ptr<Camera> cam;
    double sqrt_info;
};

} // namespace lc_core

#endif