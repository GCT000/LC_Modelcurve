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
#include "camera.h"
#include "base_type.h"

namespace lc_core
{

    class CurveFactor
    {
    public:
        CurveFactor(const std::pair<double, double> &_line, const double &_x, const Trans &_Tcl, std::shared_ptr<Camera> _cam)
            : line(_line), x(_x), Tcl(_Tcl), cam(_cam), sqrt_info(sqrt(1)) {}

        template <typename T>
        bool operator()(const T *const a, const T *const b, const T *const c, const T *const k, const T *const m, T *residual) const
        {
            Eigen::Matrix<T, 3, 1> pLidar;
            pLidar << T(x), k[0] * T(x) + m[0], a[0] * T(x) * T(x) + b[0] * T(x) + c[0];
            Eigen::Matrix<T, 3, 1> pCam = Tcl.R.cast<T>() * pLidar + Tcl.t.cast<T>();
            Eigen::Matrix<T, 2, 1> pImg;
            cam->spaceToPlane(pCam, pImg);

            T lineNorm = ceres::sqrt(T(line.first * line.first) + T(1.0));
            residual[0] = T(sqrt_info) * ceres::abs(line.first * pImg(0) - pImg(1) + T(line.second)) / lineNorm;
            return true;
        }

        static ceres::CostFunction *Create(const std::pair<double, double> &_line, const double &_x, const Trans &_Tcl, std::shared_ptr<Camera> _cam)
        {
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

    class CurveFactorA : public ceres::SizedCostFunction<1,1,1,1,1,1>
    {
    public:
        CurveFactorA(const std::pair<double, double> &_line, const double &_x, const Trans &_Tcl, std::shared_ptr<Camera> _cam)
            : line(_line), x(_x), Tcl(_Tcl), cam(_cam), sqrt_info(sqrt(1)) {}

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

            double lineNorm = ceres::sqrt((line.first * line.first) + (1.0));
            residual[0] = (sqrt_info)*ceres::abs(line.first * pImg(0) - pImg(1) + (line.second)) / lineNorm;

            Eigen::Matrix<double, 1, 2> matrix_V2Pimg;
            matrix_V2Pimg << line.first / lineNorm, (-1) / lineNorm;

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
                    p2c << 0, 0, 2 * c * x;
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
        std::pair<double, double> line;
        double x;
        Trans Tcl;
        std::shared_ptr<Camera> cam;
        double sqrt_info;
    };

} // namespace lc_core

#endif