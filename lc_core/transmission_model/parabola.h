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

    Eigen::Vector3d generateSinglePoint(const double &y) override;

    void optimizeTransmissionModel(const P2LMatchResult& lines, const OptimizationInput& input, int y_optimize = 0) override;

    void optimizeTransmissionModel(const P2PMatchResult& points, const OptimizationInput& input, int y_optimize = 0, int time = 1) override;

    void optimizeTransmissionModelDark(const Eigen::Vector3d& end_point) override;

    void setParams(const double &a1, const double &b1, const double &c1, const double &a2, const double &b2, const double &c2) {
        a1_ = a1;
        b1_ = b1;
        c1_ = c1;
        a2_ = a2;
        b2_ = b2;
        c2_ = c2;
    }

private:
    double a1_, b1_, c1_;
    double a2_, b2_, c2_;
};

} // namespace lc_core
#endif // PARABOLA_H