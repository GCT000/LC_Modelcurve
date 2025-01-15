/**
 * @file     parabola.h
 * @brief    This file is the implementation of the parabola transmission model.
 * @details  z = ax^2 + bx + c, y = kx + m
 * @author   Yipeng Zhao
 * @date     2024-10
 */

#ifndef PARABOLA_H
#define PARABOLA_H

#include "transmissionModel.h"

namespace lc_core
{

class Parabola : public TransmissionModel
{
public:
    Parabola() = default;
    ~Parabola() = default;

    void fitTransmissionModel(std::vector<Eigen::Vector3d> &points) override;

    Eigen::Vector3d generateSinglePoint(const double &x) override;

    void optimizeTransmissionModel(const P2LMatchResult& lines, const OptimizationInput& input, int y_optimize = 0) override;

    void optimizeTransmissionModel(const P2PMatchResult& points, const OptimizationInput& input, int y_optimize = 0, int time = 1) override;

    void optimizeTransmissionModelDark(const P2LMatchResult& lines, const OptimizationInput& input) override;

    void optimizeTransmissionModelDark(const P2PMatchResult& points, const OptimizationInput& input) override;

    void setParams(const double &a, const double &b, const double &c, const double &k, const double &m) {
        a_ = a;
        b_ = b;
        c_ = c;
        k_ = k;
        m_ = m;
    }

private:
    double a_, b_, c_;
    double k_, m_;
};

} // namespace lc_core
#endif // PARABOLA_H