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
    bool operator()(const T* const a, const T* const b, const T* const c, const T* const k, const T* const m, T* residual) const {
        residual[0] = T(1e3) * ceres::abs(ez_ - a[0] * T(ex_) * T(ex_) - b[0] * T(ex_) - c[0]);
        residual[1] = T(1e3) * ceres::abs(ey_ - k[0] * T(ex_) - m[0]);
        return true;
    }

    static ceres::CostFunction* Create(double _ex, double _ey, double _ez) {
        return (new ceres::AutoDiffCostFunction<ParabolaEpFactor, 2, 1, 1, 1, 1, 1>(
            new ParabolaEpFactor(_ex, _ey, _ez)));
    }

private:
    double ex_, ey_, ez_;
};

}

#endif