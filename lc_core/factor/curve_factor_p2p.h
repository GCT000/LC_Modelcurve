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
#include "camera.h"
#include "base_type.h"

namespace lc_core
{

    enum class WeightType
    {
        Equal = 1,
        Distance = 2
    };

    class CurveP2PFactor
    {
    public:
        CurveP2PFactor(const cv::Point2d &_img_p, const double &_x, const Trans &_Tcl, std::shared_ptr<Camera> _cam, const WeightType &_weight_type = WeightType::Equal)
            : img_p(_img_p), x(_x), Tcl(_Tcl), cam(_cam), weight_type(_weight_type)
        {
            if (weight_type == WeightType::Distance)
            {
                sqrt_info = sqrt(_x);
            }
            else
            {
                sqrt_info = 1.0;
            }
        }

        template <typename T>
        bool operator()(const T *const a, const T *const b, const T *const c, const T *const k, const T *const m, T *residual) const
        {
            Eigen::Matrix<T, 3, 1> pLidar;
            pLidar << T(x), k[0] * T(x) + m[0], a[0] * T(x) * T(x) + b[0] * T(x) + c[0];
            Eigen::Matrix<T, 3, 1> pCam = Tcl.R.cast<T>() * pLidar + Tcl.t.cast<T>();
            Eigen::Matrix<T, 2, 1> pImg;
            cam->spaceToPlane(pCam, pImg);

            T dist = ceres::sqrt(T(pImg(0) - T(img_p.x)) * T(pImg(0) - T(img_p.x)) + T(pImg(1) - T(img_p.y)) * T(pImg(1) - T(img_p.y)));
            residual[0] = T(sqrt_info) * dist;

            return true;
        }

        static ceres::CostFunction *Create(const cv::Point2d &_img_p, const double &_x, const Trans &_Tcl, std::shared_ptr<Camera> _cam, const WeightType &_weight_type = WeightType::Equal)
        {
            return (new ceres::AutoDiffCostFunction<CurveP2PFactor, 1, 1, 1, 1, 1, 1>(
                new CurveP2PFactor(_img_p, _x, _Tcl, _cam, _weight_type)));
        }

    private:
        cv::Point2d img_p;
        double x;
        Trans Tcl;
        std::shared_ptr<Camera> cam;
        double sqrt_info;
        WeightType weight_type;
    };

    class CurveP2PFactorA : public ceres::SizedCostFunction<1, 1, 1, 1, 1, 1>
    {
    public:
        CurveP2PFactorA(const cv::Point2d &_img_p, const double &_x, const Trans &_Tcl, std::shared_ptr<Camera> _cam, const WeightType &_weight_type = WeightType::Equal)
            : img_p(_img_p), x(_x), Tcl(_Tcl), cam(_cam), weight_type(_weight_type)
        {
            if (weight_type == WeightType::Distance)
            {
                sqrt_info = sqrt(_x);
            }
            else
            {
                sqrt_info = 1.0;
            }
        }

        virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const
        {
            const double k = parameters[0][0];
            const double m = parameters[1][0];
            const double c = parameters[2][0];
            const double c1 = parameters[3][0];
            const double c2 = parameters[4][0];

            Eigen::Matrix<double, 3, 1> pLidar;
            pLidar << x, k * x + m, c * x * x + c1 * x + c2;
            Eigen::Matrix<double, 3, 1> pCam = Tcl.R.cast<double>() * pLidar + Tcl.t.cast<double>();
            Eigen::Matrix<double, 2, 1> pImg;
            cam->spaceToPlane(pCam, pImg);

            double dist = ceres::sqrt((pImg(0) - img_p.x) * (pImg(0) - img_p.x) + (pImg(1) - img_p.y) * (pImg(1) - img_p.y));
            residual[0] = sqrt_info * dist;

            Eigen::Matrix<double, 1, 2> matrix_V2Pimg;
            matrix_V2Pimg << (pImg(0) - img_p.x) / dist, (pImg(1) - img_p.y) / dist;

            Eigen::Matrix<double, 2, 3> matrix_Pimg2Pcam;
            matrix_Pimg2Pcam << cam->fx_ / pCam(2), 0, -((cam->fx_ * pCam(0)) / (pCam(2) * pCam(2))),
                0, cam->fy_ / pCam(2), -((cam->fy_ * pCam(1)) / (pCam(2) * pCam(2)));

            if (jacobians)
            {
                double inex = (x + c1) / c;
                if (jacobians[0])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobians_p2k(jacobians[0]);
                    Eigen::Matrix<double, 3, 1> p2k;
                    p2k << 0, x, 0;
                    jacobians_p2k = matrix_V2Pimg * matrix_Pimg2Pcam * Tcl.R.cast<double>() * p2k * sqrt_info;
                }

                if (jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobians_p2m(jacobians[1]);
                    Eigen::Matrix<double, 3, 1> p2m;
                    p2m << 0, 1, 0;
                    jacobians_p2m = matrix_V2Pimg * matrix_Pimg2Pcam * Tcl.R.cast<double>() * p2m * sqrt_info;
                }

                if (jacobians[2])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobians_p2c(jacobians[2]);
                    Eigen::Matrix<double, 3, 1> p2c;
                    p2c << 0, 0, 2 * x * c;
                    jacobians_p2c = matrix_V2Pimg * matrix_Pimg2Pcam * Tcl.R.cast<double>() * p2c * sqrt_info;
                }

                if (jacobians[3])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobians_p2c1(jacobians[3]);
                    Eigen::Matrix<double, 3, 1> p2c1;
                    p2c1 << 0, 0, c1;
                    jacobians_p2c1 = matrix_V2Pimg * matrix_Pimg2Pcam * Tcl.R.cast<double>() * p2c1 * sqrt_info;
                }

                if (jacobians[4])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobians_p2c2(jacobians[4]);
                    Eigen::Matrix<double, 3, 1> p2c2;
                    p2c2 << 0, 0, 1;
                    jacobians_p2c2 = matrix_V2Pimg * matrix_Pimg2Pcam * Tcl.R.cast<double>() * p2c2 * sqrt_info;
                }
            }
            return true;
        }

    private:
        cv::Point2d img_p;
        double x;
        Trans Tcl;
        std::shared_ptr<Camera> cam;
        double sqrt_info;
        WeightType weight_type;
    };

} // namespace lc_core

#endif