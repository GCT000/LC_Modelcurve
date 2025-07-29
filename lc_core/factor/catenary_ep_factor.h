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
            residual[0] = T(1e2) * ceres::abs(ez_ - c[0] * ceres::cosh((T(ex_) + c1[0]) / c[0]) - c2[0]);
            residual[1] = T(1e2) * ceres::abs(ey_ - k[0] * T(ex_) - m[0]);
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

    class CatenaryEpFactorA : public ceres::SizedCostFunction<2, 1, 1, 1, 1, 1, 1>
    {
    public:
        CatenaryEpFactorA(double _ex, double _ey, double _ez) : ex_(_ex), ey_(_ey), ez_(_ez) {}

        virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const
        {
            const double T1 = parameters[0][0];
            const double T2 = parameters[1][0];
            const double T3 = parameters[2][0];
            const double F1 = parameters[3][0];
            const double F2 = parameters[4][0];
            const double F3 = parameters[5][0];

            residual[0] = 1e2 * ceres::abs(ez_ - F1 * ex_ * ex_ - F2 - F3 * ex_);
            residual[1] = 1e2 * ceres::abs(ex_ - T1 - T2 * ey_ - T3 * ey_ * ey_);

            if (jacobians)
            {
                double dz = ez_ - F1 * ex_ * ex_ - F2 - F3 * ex_;
                double dx = ex_ - T1 - T2 * ey_ - T3 * ey_ * ey_;
                if (jacobians[0])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2t1(jacobians[0]);
                    Eigen::Matrix<double, 2, 1> v2t1;
                    v2t1 = dx > 0 ? Eigen::Vector2d(0 , -1): Eigen::Vector2d(0 , 1);
                    jacobians_v2t1 = 1e2 * v2t1;
                }

                if (jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2t2(jacobians[1]);
                    Eigen::Matrix<double, 2, 1> v2t2;
                    v2t2 = dx > 0 ? Eigen::Vector2d(0, -ey_) : Eigen::Vector2d(0, ey_);
                    jacobians_v2t2 = 1e2 * v2t2;
                }

                if (jacobians[2])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2t3(jacobians[2]);
                    Eigen::Matrix<double, 2, 1> v2t3;
                    v2t3 = dx > 0 ? Eigen::Vector2d(0, -ey_*ey_) : Eigen::Vector2d(0, ey_*ey_);
                    jacobians_v2t3 = 1e2 * v2t3;
                }

                if (jacobians[3])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2f1(jacobians[3]);
                    Eigen::Matrix<double, 2, 1> v2f1;
                    v2f1 = dz > 0 ? Eigen::Vector2d(-ex_*ex_, 0) : Eigen::Vector2d(ex_*ex_, 0);
                    jacobians_v2f1 = 1e2 * v2f1;
                }

                if (jacobians[4])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2f2(jacobians[4]);
                    Eigen::Matrix<double, 2, 1> v2f2;
                    v2f2 = dz > 0 ? Eigen::Vector2d(-1, 0) : Eigen::Vector2d(1, 0);
                    jacobians_v2f2 = 1e2 * v2f2;
                }

                if (jacobians[5])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::ColMajor>> jacobians_v2f3(jacobians[5]);
                    Eigen::Matrix<double, 2, 1> v2f3;
                    v2f3 = dz > 0 ? Eigen::Vector2d(-ex_, 0) : Eigen::Vector2d(ex_, 0);
                    jacobians_v2f3 = 1e2 * v2f3;
                }
            }

            return true;
        }

    private:
        double ex_, ey_, ez_;
    };

    class CatenaryEpFactorxy : public ceres::SizedCostFunction<1, 1, 1, 1>
    {
    public:
        CatenaryEpFactorxy(double _ex, double _ey, double _ez) : ex_(_ex), ey_(_ey), ez_(_ez) {}

        virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const
        {
            const double T1 = parameters[0][0];
            const double T2 = parameters[1][0];
            const double T3 = parameters[2][0];

            residual[0] = 1e2 * ceres::abs(ex_ - T1 - T2/10 * ey_ - T3/1000 * ey_ * ey_);

            if (jacobians)
            {
                double dx = ex_ - T1 - T2/10 * ey_ - T3/1000 * ey_ * ey_;
                if (jacobians[0])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::ColMajor>> jacobians_v2t1(jacobians[0]);
                    Eigen::Matrix<double, 1, 1> v2t1;
                    v2t1 << (dx > 0 ? -1:  1);
                    jacobians_v2t1 = 1e2 * v2t1;
                }

                if (jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::ColMajor>> jacobians_v2t2(jacobians[1]);
                    Eigen::Matrix<double, 1, 1> v2t2;
                    v2t2 << (dx > 0 ? -ey_/10 : ey_/10);
                    jacobians_v2t2 = 1e2 * v2t2;
                }

                if (jacobians[2])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::ColMajor>> jacobians_v2t3(jacobians[2]);
                    Eigen::Matrix<double, 1, 1> v2t3;
                    v2t3 << (dx > 0 ? -ey_*ey_/1000 : ey_*ey_/1000);
                    jacobians_v2t3 = 1e2 * v2t3;
                }
            }

            return true;
        }

    private:
        double ex_, ey_, ez_;
    };


    class CatenaryEpFactorxz : public ceres::SizedCostFunction<1, 1, 1, 1>
    {
    public:
        CatenaryEpFactorxz(double _ex, double _ey, double _ez) : ex_(_ex), ey_(_ey), ez_(_ez) {}

        virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const
        {
            const double F1 = parameters[0][0];
            const double F2 = parameters[1][0];
            const double F3 = parameters[2][0];

            residual[0] = 1e2 * ceres::abs(ez_ - F1/10000 * ex_ * ex_ - F2 - F3/10 * ex_);

            if (jacobians)
            {
                double dz = ez_ - F1/10000 * ex_ * ex_ - F2 - F3/10 * ex_;

                if (jacobians[0])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::ColMajor>> jacobians_v2f1(jacobians[0]);
                    Eigen::Matrix<double, 1, 1> v2f1;
                    v2f1 << (dz > 0 ? -ex_*ex_/10000 : ex_*ex_/10000);
                    jacobians_v2f1 = 1e2 * v2f1;
                }

                if (jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::ColMajor>> jacobians_v2f2(jacobians[1]);
                    Eigen::Matrix<double, 1, 1> v2f2;
                    v2f2 << (dz > 0 ? -1 : 1);
                    jacobians_v2f2 = 1e2 * v2f2;
                }

                if (jacobians[2])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::ColMajor>> jacobians_v2f3(jacobians[2]);
                    Eigen::Matrix<double, 1, 1> v2f3;
                    v2f3 << (dz > 0 ? -ex_/10 : ex_/10);
                    jacobians_v2f3 = 1e2 * v2f3;
                }
            }


            return true;
        }

    private:
        double ex_, ey_, ez_;
    };

}

#endif