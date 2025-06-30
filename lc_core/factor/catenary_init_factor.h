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

class CatenaryInitFactor_xy : public ceres::SizedCostFunction<1,1,1,1>
{
public:
    CatenaryInitFactor_xy(double x_obs, double y_obs) : x_obs(x_obs), y_obs(y_obs) {};
    virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const 
    {
        const double T1 = parameters[0][0];
        const double T2 = parameters[1][0];
        const double T3 = parameters[2][0];
        residual[0] = T1 + T2 * y_obs + T3 * y_obs * y_obs - x_obs ;
        if (jacobians)
        {
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_t1(jacobians[0]);
                Eigen::Matrix<double, 1, 1> t1;
                t1 << 1;
                jacobian_t1 = t1;
            }
            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_t2(jacobians[1]);
                Eigen::Matrix<double, 1, 1> t2;
                t2 << y_obs;
                jacobian_t2 = t2;
            }
            if(jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_t3(jacobians[2]);
                Eigen::Matrix<double, 1, 1> t3;
                t3 << y_obs*y_obs;
                jacobian_t3 = t3;
            }
        } 
        return true;
    }
private:
    const double x_obs;
    const double y_obs;
};

class CatenaryInitFactor_xz : public ceres::SizedCostFunction<1,1,1,1>
{
public:
    CatenaryInitFactor_xz(double x_obs, double z_obs) : x_obs(x_obs), z_obs(z_obs) {};
    virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const 
    {
        const double F1 = parameters[0][0];
        const double F2 = parameters[1][0];
        const double F3 = parameters[2][0];

        residual[0] = F1 * x_obs*x_obs + F2 + F3 *x_obs-z_obs;
        if (jacobians)
        {
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_f1(jacobians[0]);
                Eigen::Matrix<double, 1, 1> f1;
                f1 << x_obs*x_obs;
                jacobian_f1 = f1;
            }
            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_f2(jacobians[1]);
                Eigen::Matrix<double, 1, 1> f2;
                f2 << 1;
                jacobian_f2 = f2;
            }
            if(jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_f3(jacobians[2]);
                Eigen::Matrix<double, 1, 1> f3;
                f3 << x_obs;
                jacobian_f3 = f3;
            }
        } 
        return true;
    }
private:
    const double x_obs;
    const double z_obs;
};

} // namespace lc_core
