/**
 * @file    catenary_init_factor.h
 * @brief   Catenary initialization factor for ceres optimization
 * @author  Yipeng Zhao
 * @date    2024-10
 */

#include <ceres/ceres.h>
#include <iostream>
#include <vector>

namespace lc_core
{

class CatenaryInitFactor {
public:
    CatenaryInitFactor(double x, double z_obs) : x_(x), z_obs_(z_obs) {}

    template <typename T>
    bool operator()(const T* const k, const T* const C1, const T* const C2, T* residual) const {
        // two-degree taylor expansion
        T term1 = T(1) + T(0.5) * ceres::pow((x_ + *C1) / *k, 2);
        T term2 = T(1.0/24.0) * ceres::pow((x_ + *C1) / *k, 4);
        T z_model = (*k) * (term1 + term2) + *C2;
        
        // residual
        residual[0] = z_model - z_obs_;
        return true;
    }

private:
    const double x_;
    const double z_obs_;
};


class CatenaryInitFactorA : public ceres::SizedCostFunction<1,1,1,1>{
public:
    CatenaryInitFactorA(double x, double z_obs) : x_(x), z_obs_(z_obs) {}

    virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const
    {
        const double c = parameters[0][0];
        const double c1 = parameters[1][0];
        const double c2 = parameters[2][0];
        double inex = (x_ + c1)/c;

        residual[0] = c * ceres::cosh(inex) +c2 - z_obs_;

        if (jacobians)
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_c(jacobians[0]);
                Eigen::Matrix<double, 1, 1> c;
                c << ceres::cosh(inex) - inex * ceres::sinh(inex);
                jacobian_c = c ;
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_c1(jacobians[1]);
                Eigen::Matrix<double, 1, 1> c1;
                c1 << ceres::sinh(inex);
                jacobian_c1 = c1;
            }
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_c2(jacobians[2]);
                Eigen::Matrix<double, 1, 1> c2;
                c2 << 1;
                jacobian_c2 = c2 ;
            }
        }
        return true;
    }

private:
    const double x_;
    const double z_obs_;
};

} // namespace lc_core
