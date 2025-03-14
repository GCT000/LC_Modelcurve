/**
 * @file    catenary_ep_factor.h
 * @brief   Catenary end point factor for ceres optimization
 * @author  Yipeng Zhao
 * @date    2025-01
 */

#ifndef CATENARY_EP_FACTOR_H
#define CATENARY_EP_FACTOR_H

#include <ceres/ceres.h>

namespace lc_core
{

    class CatenaryEpFactor
    {
    public:
        CatenaryEpFactor(double _ex, double _ey, double _ez) : ex_(_ex), ey_(_ey), ez_(_ez) {}

        template <typename T>
        bool operator()(const T *const c, const T *const c1, const T *const c2, const T *const k, const T *const m, T *residual) const
        {
            residual[0] = T(1e3) * ceres::abs(ez_ - c[0] * ceres::cosh((T(ex_) + c1[0]) / c[0]) - c2[0]);
            residual[1] = T(1e3) * ceres::abs(ey_ - k[0] * T(ex_) - m[0]);
            return true;
        }

        static ceres::CostFunction *Create(double _ex, double _ey, double _ez)
        {
            return (new ceres::AutoDiffCostFunction<CatenaryEpFactor, 2, 1, 1, 1, 1, 1>(
                new CatenaryEpFactor(_ex, _ey, _ez)));
        }

    private:
        double ex_, ey_, ez_;
    };

    class CatenaryEpFactorA : public ceres::SizedCostFunction<2, 1, 1, 1, 1, 1>
    {
    public:
        CatenaryEpFactorA(double _ex, double _ey, double _ez) : ex_(_ex), ey_(_ey), ez_(_ez) {}

        virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const
        {
            const double k = parameters[0][0];
            const double m = parameters[1][0];
            const double c = parameters[2][0];
            const double c1 = parameters[3][0];
            const double c2 = parameters[4][0];

            residual[0] = 1e3 * ceres::abs(ez_ - c * ceres::cosh((ex_ + c1) / c) - c2);
            residual[1] = 1e3 * ceres::abs(ey_ - k * ex_ - m);

            if (jacobians)
            {
                double dz = ez_ - c * ceres::cosh((ex_ + c1) / c) - c2;
                double dy = ey_ - k * ex_ - m;
                double inex = (ex_ + c1) / c;
                if (jacobians[0])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2k(jacobians[0]);
                    Eigen::Matrix<double, 2, 1> v2k;
                    if (dy > 0)
                    {
                        v2k << 0, -ex_;
                    }
                    else
                    {
                        v2k << 0, ex_;
                    }
                    jacobians_v2k = v2k;
                }

                if (jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2m(jacobians[1]);
                    Eigen::Matrix<double, 2, 1> v2m;
                    if (dy > 0)
                    {
                        v2m << 0, -1;
                    }
                    else
                    {
                        v2m << 0, 1;
                    }
                    jacobians_v2m = v2m;
                }

                if (jacobians[2])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2c(jacobians[2]);
                    Eigen::Matrix<double, 2, 1> v2c;
                    if (dz > 0)
                    {
                        v2c << -ceres::cosh(inex) + inex * ceres::sinh(inex), 0;
                    }
                    else
                    {
                        v2c << ceres::cosh(inex) - inex * ceres::sinh(inex), 0;
                    }
                    jacobians_v2c = v2c;
                }

                if (jacobians[3])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2c1(jacobians[3]);
                    Eigen::Matrix<double, 2, 1> v2c1;
                    if (dz > 0)
                    {
                        v2c1 << -ceres::sinh(inex), 0;
                    }
                    else
                    {
                        v2c1 << ceres::sinh(inex), 0;
                    }
                    jacobians_v2c1 = v2c1;
                }

                if (jacobians[4])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2c2(jacobians[4]);
                    Eigen::Matrix<double, 2, 1> v2c2;
                    if (dz > 0)
                    {
                        v2c2 << -1, 0;
                    }
                    else
                    {
                        v2c2 << 1, 0;
                    }
                    jacobians_v2c2 = v2c2;
                }
            }

            return true;
        }

    private:
        double ex_, ey_, ez_;
    };

}

#endif