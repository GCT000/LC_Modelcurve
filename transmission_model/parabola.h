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

class Parabola : public TransmissionModel
{
public:
    Parabola() = default;
    ~Parabola() = default;

    void fitTransmissionModel(const std::vector<Eigen::Vector3d> &points) override;

    Eigen::Vector3d generateSinglePoint(const double &x) override;

    void optimizeTransmissionModel(const P2LMatchResult& lines, const OptimizationInput& input) override;

    void optimizeTransmissionModel(const P2PMatchResult& points, const OptimizationInput& input, int time = 2) override;

private:
    double a_, b_, c_;
    double k_, m_;
};


#endif // PARABOLA_H