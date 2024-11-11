/**
 * @file   curve_simulation.hpp
 * @brief  This file is used to generate 3D curves.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#ifndef CURVE_SIMULATION_H
#define CURVE_SIMULATION_H

#include <Eigen/Dense>
#include <memory>

namespace lc_simulation
{

template <typename T>
class CurveSimulation
{
public:
    CurveSimulation(T y_base, Eigen::Matrix<T, 3, 1> start_point, Eigen::Matrix<T, 3, 1> end_point, Eigen::Matrix<T, 3, 1> mid_point)
        : y_base_(y_base), start_point_(start_point), end_point_(end_point), mid_point_(mid_point)
    {
        assert(end_point_.x() - mid_point_.x() != 0);
        a_ = (end_point_.z() - mid_point_.z()) / pow((end_point_.x() - mid_point_.x()), 2);
    }

    ~CurveSimulation() = default;

    Eigen::Matrix<T, 3, 1> pointInCurve(T x)
    {
        return Eigen::Matrix<T, 3, 1>(x, y_base_, a_ * pow((x - mid_point_.x()), 2) + mid_point_.z());
    }

    Eigen::Matrix<T, 3, 1> start_point()
    {
        return start_point_;
    }

    Eigen::Matrix<T, 3, 1> end_point()
    {
        return end_point_;
    }

private:
    T y_base_;
    Eigen::Matrix<T, 3, 1> start_point_;
    Eigen::Matrix<T, 3, 1> end_point_;
    Eigen::Matrix<T, 3, 1> mid_point_;
    // z = a * (x - mid_point_.x()) ^ 2 + mid_point_.z()
    T a_;
};

} // namespace lc_simulation
#endif // CURVE_SIMULATION_H
