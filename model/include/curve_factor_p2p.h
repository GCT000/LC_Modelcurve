/**
 * @file    curve_factor.h
 * @brief   Curve factor(p2p match) for ceres optimization
 * @author  Yipeng Zhao
 * @date    2024-09
*/

#ifndef CURVE_FACTOR_P2P_H
#define CURVE_FACTOR_P2P_H

#include <iostream>
#include <ceres/ceres.h>
#include <opencv2/core/types.hpp>
#include "camera.hpp"

class CurveP2PFactor {
public:
    CurveP2PFactor(const cv::Point2d& _img_p, const double& _x, const Trans& _Tcl, std::shared_ptr<Camera> _cam) 
        : img_p(_img_p), x(_x), Tcl(_Tcl), cam(_cam), sqrt_info(sqrt(_x)) {}

    template <typename T>
    bool operator()(const T* const a, const T* const b, const T* const c, const T* const k, const T* const m, T* residual) const {
        Eigen::Matrix<T, 3, 1> pLidar;
        pLidar << T(x), (T(x) - m[0]) / k[0], a[0] * T(x) * T(x) + b[0] * T(x) + c[0];
        Eigen::Matrix<T, 3, 1> pCam = Tcl.R.cast<T>() * pLidar + Tcl.t.cast<T>();
        Eigen::Matrix<T, 2, 1> pImg;
        cam->spaceToPlane(pCam, pImg);

        residual[0] = T(sqrt_info) * (pImg(0) - T(img_p.x));
        residual[1] = T(sqrt_info) * (pImg(1) - T(img_p.y));
        return true;
    }

    static ceres::CostFunction* Create(const cv::Point2d& _img_p, const double& _x, const Trans& _Tcl, std::shared_ptr<Camera> _cam) {
        return (new ceres::AutoDiffCostFunction<CurveP2PFactor, 2, 1, 1, 1, 1, 1>(
            new CurveP2PFactor(_img_p, _x, _Tcl, _cam)));
    }

private:
    cv::Point2d img_p;
    double x;
    Trans Tcl;
    std::shared_ptr<Camera> cam;
    double sqrt_info;
};

#endif