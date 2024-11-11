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

// 定义代价函数
class CatenaryInitFactor {
public:
    CatenaryInitFactor(double x, double z_obs) : x_(x), z_obs_(z_obs) {}

    template <typename T>
    bool operator()(const T* const k, const T* const C1, const T* const C2, T* residual) const {
        // 模型方程
        T term1 = T(1) + T(0.5) * ceres::pow((x_ + *C1) / *k, 2);
        T term2 = T(1.0/24.0) * ceres::pow((x_ + *C1) / *k, 4);
        T z_model = (*k) * (term1 + term2) + *C2;
        
        // 计算残差 (观测值 - 预测值)
        residual[0] = z_model - z_obs_;
        return true;
    }

private:
    const double x_;
    const double z_obs_;
};

} // namespace lc_core
