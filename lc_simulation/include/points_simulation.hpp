/**
 * @file   points_simulation.hpp
 * @brief  This file is used to generate random points according to curve equation.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#ifndef POINTS_SIMULATION_H
#define POINTS_SIMULATION_H

#include "curve_simulation.hpp"

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace lc_simulation
{

template <typename T>
class PointsSimulation
{
public:
    PointsSimulation(T y_base, Eigen::Matrix<T, 3, 1> start_point, Eigen::Matrix<T, 3, 1> end_point, Eigen::Matrix<T, 3, 1> mid_point,
                     const int &point_num)
        : point_num_(point_num)
    {
        curve_ = std::make_shared<CurveSimulation<T>>(y_base, start_point, end_point, mid_point);

        points_.resize(point_num_);

        generatePoints();
    }

    ~PointsSimulation() = default;

    void generatePoints()
    {
        T x_min = curve_->start_point().x();
        T x_max = curve_->end_point().x();
        T x_step = (x_max - x_min) / (point_num_ - 1);
        for (int i = 0; i < point_num_; i++)
        {
            points_[i] = curve_->pointInCurve(x_min + i * x_step);
        }
    }

    std::vector<Eigen::Matrix<T, 3, 1>> getPoints()
    {
        return points_;
    }

private:
    std::shared_ptr<CurveSimulation<T>> curve_;
    int point_num_;
    std::vector<Eigen::Matrix<T, 3, 1>> points_;
};

} // namespace lc_simulation

#endif  // POINTS_SIMULATION_H