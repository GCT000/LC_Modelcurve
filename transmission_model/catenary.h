/**
 * @file   catenary.h
 * @brief  This file contains catenary model.
 * @author Yipeng Zhao
 * @date   2024-10
 */

#ifndef CATENARY_H
#define CATENARY_H

#include "transmissionModel.h"

class Catenary : public TransmissionModel
{
public:
    Catenary() : c_(1.0), c1_(0.0), c2_(0.0) {}
    ~Catenary() = default;

    void fitTransmissionModel(const std::vector<Eigen::Vector3d> &points) override;

    Eigen::Vector3d generateSinglePoint(const double &x) override;

    void optimizeTransmissionModel(const P2LMatchResult& lines, const OptimizationInput& input) override;

    void optimizeTransmissionModel(const P2PMatchResult& points, const OptimizationInput& input, int time = 1) override;

private:
    double c_, c1_, c2_;
    double k_, m_;
};

#endif