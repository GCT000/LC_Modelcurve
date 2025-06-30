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
    Catenary(): T1(100){};
    ~Catenary() = default;

    void fitTransmissionModel(std::vector<Eigen::Vector3d> &points) override;

    Eigen::Vector3d generateSinglePoint(const double &y) override;

    void optimizeTransmissionModel(const P2LMatchResult& lines, const OptimizationInput& input, int y_optimize = 0) override;

    void optimizeTransmissionModel(const P2PMatchResult& points, const OptimizationInput& input, int y_optimize = 0, int time = 1) override;

    void optimizeTransmissionModelDark(const Eigen::Vector3d& end_point) override;

    void setParams(const double &c, const double &c1, const double &c2, const double &k, const double &m) {
    }

private:
    double F1, F2, F3;
    double T1, T2, T3;
    bool is_manual;
};

} // namespace lc_core
#endif