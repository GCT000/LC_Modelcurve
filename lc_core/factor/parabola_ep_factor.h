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

    class ParabolaEpFactorA : public ceres::SizedCostFunction<2, 1, 1, 1, 1, 1, 1>
    {
    public:
        ParabolaEpFactorA(double _ex, double _ey, double _ez) : ex_(_ex), ey_(_ey), ez_(_ez) {}

        virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const
        {
            const double a1 = parameters[0][0];
            const double b1 = parameters[1][0];
            const double c1 = parameters[2][0];
            const double a2 = parameters[3][0];
            const double b2 = parameters[4][0];
            const double c2 = parameters[5][0];

            residual[0] = 1e2 * ceres::abs(ez_ - a2 * ey_ * ey_ - b2 * ey_ - c2);
            residual[1] = 1e2 * ceres::abs(ex_ - a1 * ey_ * ey_ - b1 * ey_ - c1);

            if (jacobians)
            {
                double dz = ez_ - a2 * ey_ * ey_ - b2 * ey_ - c2;
                double dx = ex_ - a1 * ey_ * ey_ - b1 * ey_ - c1;
                if (jacobians[0])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2a1(jacobians[0]);
                    Eigen::Matrix<double, 2, 1> v2a1;
                    v2a1 = dx > 0 ? Eigen::Vector2d(0, -ey_ * ey_) : Eigen::Vector2d(0, ey_ * ey_);
                    jacobians_v2a1 = 1e2 * v2a1;
                }

                if (jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2b1(jacobians[1]);
                    Eigen::Matrix<double, 2, 1> v2b1;
                    v2b1 = dx > 0 ? Eigen::Vector2d(0, -ey_) : Eigen::Vector2d(0, ey_);
                    jacobians_v2b1 = 1e2 * v2b1;
                }

                if (jacobians[2])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2c1(jacobians[2]);
                    Eigen::Matrix<double, 2, 1> v2c1;
                    v2c1 = dx > 0 ? Eigen::Vector2d(0, -1) : Eigen::Vector2d(0, 1);
                    jacobians_v2c1 = 1e2 * v2c1;
                }


                if (jacobians[3])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2a2(jacobians[3]);
                    Eigen::Matrix<double, 2, 1> v2a2;
                    v2a2 = dz > 0 ? Eigen::Vector2d(-ey_ * ey_, 0) : Eigen::Vector2d(ey_ * ey_, 0);
                    jacobians_v2a2 = 1e2 * v2a2;
                }

                if (jacobians[4])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2b2(jacobians[4]);
                    Eigen::Matrix<double, 2, 1> v2b2;
                    v2b2 = dz > 0 ? Eigen::Vector2d(-ey_, 0) : Eigen::Vector2d(ey_, 0);
                    jacobians_v2b2 = 1e2 * v2b2;
                }

                if (jacobians[5])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2c2(jacobians[5]);
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