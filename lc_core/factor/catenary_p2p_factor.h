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
#include "camera.h"
#include "base_type.h"

namespace lc_core
{

    enum class WeightType
    {
        Equal = 1,
        Distance = 2
    };

    class CatenaryP2PFactor
    {
    public:
        CatenaryP2PFactor(const cv::Point2d &_img_p, const double &_x, const Trans &_Tcl, std::shared_ptr<Camera> _cam, const WeightType &_weight_type = WeightType::Equal)
            : img_p(_img_p), x(_x), Tcl(_Tcl), cam(_cam), weight_type(_weight_type)
        {
            if (weight_type != WeightType::Distance)
            {
                sqrt_info = sqrt(_x);
            }
            else
            {
                sqrt_info = 1.0;
            }
        }

        template <typename T>
        bool operator()(const T *const c, const T *const c1, const T *const c2, const T *const k, const T *const m, T *residual) const
        {
            Eigen::Matrix<T, 3, 1> pLidar;
            pLidar << T(x), k[0] * T(x) + m[0], c[0] * ceres::cosh((T(x) + c1[0]) / c[0]) + c2[0];
            Eigen::Matrix<T, 3, 1> pCam = Tcl.R.cast<T>() * pLidar + Tcl.t.cast<T>();
            Eigen::Matrix<T, 2, 1> pImg;
            cam->spaceToPlane(pCam, pImg);

            T dist = ceres::sqrt(T(pImg(0) - T(img_p.x)) * T(pImg(0) - T(img_p.x)) + T(pImg(1) - T(img_p.y)) * T(pImg(1) - T(img_p.y)));
            residual[0] = T(sqrt_info) * dist;

            return true;
        }

        static ceres::CostFunction *Create(const cv::Point2d &_img_p, const double &_x, const Trans &_Tcl, std::shared_ptr<Camera> _cam, const WeightType &_weight_type = WeightType::Equal)
        {
            return (new ceres::AutoDiffCostFunction<CatenaryP2PFactor, 1, 1, 1, 1, 1, 1>(
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

    class CatenaryP2PFactorA : public ceres::SizedCostFunction<1, 1, 1, 1, 1, 1, 1>
    {
    public:
        CatenaryP2PFactorA(const cv::Point2d &_img_p, const double &_y, const Trans &_Tcl, std::shared_ptr<Camera> _cam, const WeightType &_weight_type = WeightType::Equal)
            : img_p(_img_p), y(_y), Tcl(_Tcl), cam(_cam), weight_type(_weight_type)
        {
            if (weight_type != WeightType::Distance)
            {
                sqrt_info = sqrt(ceres::abs(_y));
            }
            else
            {
                sqrt_info = 1.0;
            }
        }
        virtual ~CatenaryP2PFactorA() {};

        virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const
        {
            const double T1 = parameters[0][0];
            const double T2 = parameters[1][0];
            const double T3 = parameters[2][0];
            const double F1 = parameters[3][0];
            const double F2 = parameters[4][0];
            const double F3 = parameters[5][0];

            Eigen::Matrix<double, 3, 1> pLidar;
            double x = T1 + T2 * y + T3 * y * y;
            pLidar << x, y, F1 * x * x + F2 + F3 * x;
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
                if (jacobians[0])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobians_p2t1(jacobians[0]);
                    Eigen::Matrix<double, 3, 1> p2t1;
                    p2t1 << 1, 0, 2 * F1 * x + F3;
                    jacobians_p2t1 = matrix_V2Pimg * matrix_Pimg2Pcam * Tcl.R.cast<double>() * p2t1 * sqrt_info;
                }

                if (jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobians_p2t2(jacobians[1]);
                    Eigen::Matrix<double, 3, 1> p2t2;
                    p2t2 << y, 0, (2 * F1 * x + F3) * y;
                    jacobians_p2t2 = matrix_V2Pimg * matrix_Pimg2Pcam * Tcl.R.cast<double>() * p2t2 * sqrt_info;
                }

                if (jacobians[2])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobians_p2t3(jacobians[2]);
                    Eigen::Matrix<double, 3, 1> p2t3;
                    p2t3 << y * y, 0, (2 * F1 * x + F3) * y * y;
                    jacobians_p2t3 = matrix_V2Pimg * matrix_Pimg2Pcam * Tcl.R.cast<double>() * p2t3 * sqrt_info;
                }

                if (jacobians[3])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobians_p2f1(jacobians[3]);
                    Eigen::Matrix<double, 3, 1> p2f1;
                    p2f1 << 0, 0, x*x;
                    jacobians_p2f1 = matrix_V2Pimg * matrix_Pimg2Pcam * Tcl.R.cast<double>() * p2f1 * sqrt_info;
                }

                if (jacobians[4])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobians_p2f2(jacobians[4]);
                    Eigen::Matrix<double, 3, 1> p2f2;
                    p2f2 << 0, 0, 1;
                    jacobians_p2f2 = matrix_V2Pimg * matrix_Pimg2Pcam * Tcl.R.cast<double>() * p2f2 * sqrt_info;
                }

                if (jacobians[5])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobians_p2f3(jacobians[5]);
                    Eigen::Matrix<double, 3, 1> p2f3;
                    p2f3 << 0, 0, x;
                    jacobians_p2f3 = matrix_V2Pimg * matrix_Pimg2Pcam * Tcl.R.cast<double>() * p2f3 * sqrt_info;
                }
            }
            return true;
        }

    private:
        cv::Point2d img_p;
        double y;
        Trans Tcl;
        std::shared_ptr<Camera> cam;
        double sqrt_info;
        WeightType weight_type;
    };

} // namespace lc_core

#endif