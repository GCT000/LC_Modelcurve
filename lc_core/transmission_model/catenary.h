/**
 * @file   catenary.h
 * @brief  This file contains catenary model.
 * @author Yipeng Zhao
 * @date   2024-10
 */

#ifndef CATENARY_H
#define CATENARY_H

#include "transmissionModel.h"

namespace lc_core
{

class Catenary : public TransmissionModel
{
public:
    Catenary() : c_(100.0), c1_(0.0), c2_(0.0) {}
    ~Catenary() = default;

    void fitTransmissionModel(std::vector<Eigen::Vector3d> &points) override;

    Eigen::Vector3d generateSinglePoint(const double &x) override;

    void optimizeTransmissionModel(const P2LMatchResult& lines, const OptimizationInput& input, int y_optimize = 0) override;

    void optimizeTransmissionModel(const P2PMatchResult& points, const OptimizationInput& input, int y_optimize = 0, int time = 1) override;

    void optimizeTransmissionModelDark(const P2LMatchResult& lines, const OptimizationInput& input) override;

    void optimizeTransmissionModelDark(const P2PMatchResult& points, const OptimizationInput& input) override;

    void setParams(const double &c, const double &c1, const double &c2, const double &k, const double &m) {
        c_ = c;
        c1_ = c1;
        c2_ = c2;
        k_ = k;
        m_ = m;
    }

private:
    double c_, c1_, c2_;
    double k_, m_;
    bool is_manual;
};

} // namespace lc_core
#endif