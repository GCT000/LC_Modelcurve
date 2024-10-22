/**
 * @file    curve_factor.h
 * @brief   Curve factor(p2p match) for ceres optimization
 * @author  Yipeng Zhao
 * @date    2024-09
*/

#ifndef CATENARY_P2P_FACTOR_H
#define CATENARY_P2P_FACTOR_H

#include <iostream>
#include <ceres/ceres.h>
#include <opencv2/core/types.hpp>
#include "camera.hpp"
#include "base_type.h"

enum class WeightType {
    Equal = 1,
    Distance = 2
};

class CatenaryP2PFactor {
public:
    CatenaryP2PFactor(const cv::Point2d& _img_p, const double& _x, const Trans& _Tcl, std::shared_ptr<Camera> _cam, const WeightType& _weight_type = WeightType::Equal) 
        : img_p(_img_p), x(_x), Tcl(_Tcl), cam(_cam), weight_type(_weight_type) {
        if (weight_type == WeightType::Distance) {
            sqrt_info = sqrt(_x);
        } else {
            sqrt_info = 1.0;
        }
    }

    template <typename T>
    bool operator()(const T* const c, const T* const c1, const T* const c2, const T* const k, const T* const m, T* residual) const {
        Eigen::Matrix<T, 3, 1> pLidar;
        pLidar << T(x), k[0] * T(x) + m[0], c[0] * ceres::cosh((T(x) + c1[0]) / c[0]) + c2[0];
        Eigen::Matrix<T, 3, 1> pCam = Tcl.R.cast<T>() * pLidar + Tcl.t.cast<T>();
        Eigen::Matrix<T, 2, 1> pImg;
        cam->spaceToPlane(pCam, pImg);

        residual[0] = T(sqrt_info) * (pImg(0) - T(img_p.x));
        residual[1] = T(sqrt_info) * (pImg(1) - T(img_p.y));
        return true;
    }

    static ceres::CostFunction* Create(const cv::Point2d& _img_p, const double& _x, const Trans& _Tcl, std::shared_ptr<Camera> _cam, const WeightType& _weight_type = WeightType::Equal) {
        return (new ceres::AutoDiffCostFunction<CatenaryP2PFactor, 2, 1, 1, 1, 1, 1>(
            new CatenaryP2PFactor(_img_p, _x, _Tcl, _cam, _weight_type)));
    }

private:
    cv::Point2d img_p;
    double x;
    Trans Tcl;
    std::shared_ptr<Camera> cam;
    double sqrt_info;
    WeightType weight_type;
};

#endif