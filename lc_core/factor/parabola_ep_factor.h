/**
 * @file    parabola_ep_factor.h
 * @brief   Parabola end point factor for ceres optimization
 * @author  Yipeng Zhao
 * @date    2025-01
 */

#ifndef PARABOLA_EP_FACTOR_H
#define PARABOLA_EP_FACTOR_H

#include <ceres/ceres.h>

namespace lc_core
{

    class ParabolaEpFactor
    {
    public:
        ParabolaEpFactor(double _ex, double _ey, double _ez) : ex_(_ex), ey_(_ey), ez_(_ez) {}

        template <typename T>
        bool operator()(const T *const a, const T *const b, const T *const c, const T *const k, const T *const m, T *residual) const
        {
            residual[0] = T(1e2) * ceres::abs(ez_ - a[0] * T(ex_) * T(ex_) - b[0] * T(ex_) - c[0]);
            residual[1] = T(1e2) * ceres::abs(ey_ - k[0] * T(ex_) - m[0]);
            return true;
        }

        static ceres::CostFunction *Create(double _ex, double _ey, double _ez)
        {
            return (new ceres::AutoDiffCostFunction<ParabolaEpFactor, 2, 1, 1, 1, 1, 1>(
                new ParabolaEpFactor(_ex, _ey, _ez)));
        }

    private:
        double ex_, ey_, ez_;
    };

    class ParabolaEpFactorA : public ceres::SizedCostFunction<2, 1, 1, 1, 1, 1>
    {
    public:
        ParabolaEpFactorA(double _ex, double _ey, double _ez) : ex_(_ex), ey_(_ey), ez_(_ez) {}

        virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const
        {
            const double k = parameters[0][0];
            const double m = parameters[1][0];
            const double c = parameters[2][0];
            const double c1 = parameters[3][0];
            const double c2 = parameters[4][0];

            residual[0] = 1e2 * ceres::abs(ez_ - c * ex_ * ex_ - c1 * ex_ - c2);
            residual[1] = 1e2 * ceres::abs(ey_ - k * ex_ - m);

            if (jacobians)
            {
                double dz = ez_ - c * ex_ * ex_ - c1 * ex_ - c2;
                double dy = ey_ - k * ex_ - m;
                double inex = (ex_ + c1) / c;
                if (jacobians[0])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2k(jacobians[0]);
                    Eigen::Matrix<double, 2, 1> v2k;
                    v2k = dy > 0 ? Eigen::Vector2d(0, -ex_) : Eigen::Vector2d(0, ex_);
                    jacobians_v2k = 1e2 * v2k;
                }

                if (jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2m(jacobians[1]);
                    Eigen::Matrix<double, 2, 1> v2m;
                    v2m = dy > 0 ? Eigen::Vector2d(0, -1) : Eigen::Vector2d(0, 1);
                    jacobians_v2m = 1e2 * v2m;
                }

                if (jacobians[2])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2c(jacobians[2]);
                    Eigen::Matrix<double, 2, 1> v2c;
                    v2c = dz > 0 ? Eigen::Vector2d(-ex_ * ex_, 0) : Eigen::Vector2d(ex_ * ex_, 0);
                    jacobians_v2c = 1e2 * v2c;
                }

                if (jacobians[3])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2c1(jacobians[3]);
                    Eigen::Matrix<double, 2, 1> v2c1;
                    v2c1 = dz > 0 ? Eigen::Vector2d(-ex_, 0) : Eigen::Vector2d(ex_, 0);
                    jacobians_v2c1 = 1e2 * v2c1;
                }

                if (jacobians[4])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2c2(jacobians[4]);
                    Eigen::Matrix<double, 2, 1> v2c2;
                    v2c2 = dz > 0 ? Eigen::Vector2d(-1, 0) : Eigen::Vector2d(1, 0);
                    jacobians_v2c2 = 1e2 * v2c2;
                }
            }

            return true;
        }

    private:
        double ex_, ey_, ez_;
    };

}

#endif