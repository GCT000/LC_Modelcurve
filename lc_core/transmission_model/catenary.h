/**
 * @file   catenary.h
 * @brief  This file contains catenary model.
 * @author Yipeng Zhao
 * @date   2024-10
 */

#ifndef CATENARY_H
#define CATENARY_H

#include "transmissionModel.h"

typedef std::vector<cv::Point2d> P2PMatchResult;

namespace lc_core
{

class Catenary : public TransmissionModel
{
public:
    Catenary(): T1(100),T2(0),T3(0), F1(0),F2(0),F3(0){};
    ~Catenary() = default;

    void fitTransmissionModel(std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &end_point, std::vector<double> &cov_t, std::vector<double> &cov_f, std::vector<double> &para) override;

    Eigen::Vector3d generateSinglePoint(const double &y) override;

    void optimizeTransmissionModel(const P2PMatchResult& points, const OptimizationInput& input) override;

    void optimizeTransmissionModelDark(const Eigen::Vector3d& end_point) override;

    void setParams(const double &c, const double &c1, const double &c2, const double &k, const double &m) {
    }

    std::vector<std::vector<double>> getpara() override;

    bool get_is_visual() override;

    bool set_first_time() override;
private:
    double F1, F2, F3, F4, F5;
    double T1, T2, T3;
    bool is_manual;
    bool is_visual;
    bool first_time;
    std::vector<std::vector<double>> var_para;
    std::vector<double>para_;
};

} // namespace lc_core
#endif