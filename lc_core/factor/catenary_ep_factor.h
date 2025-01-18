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
    bool operator()(const T* const c, const T* const c1, const T* const c2, const T* const k, const T* const m, T* residual) const {
        residual[0] = T(1e3) * ceres::abs(ez_ - c[0] * ceres::cosh((T(ex_) + c1[0]) / c[0]) - c2[0]);
        residual[1] = T(1e3) * ceres::abs(ey_ - k[0] * T(ex_) - m[0]);
        return true;
    }

    static ceres::CostFunction* Create(double _ex, double _ey, double _ez) {
        return (new ceres::AutoDiffCostFunction<CatenaryEpFactor, 2, 1, 1, 1, 1, 1>(
            new CatenaryEpFactor(_ex, _ey, _ez)));
    }

private:
    double ex_, ey_, ez_;
};

}

#endif