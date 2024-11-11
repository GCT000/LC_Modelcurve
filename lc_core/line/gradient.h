/**
 * @file   gradient.h
 * @brief  calculate line points' gradient.
 * @author Yipeng Zhao
 * @date   2024-09
 */

#ifndef GRADIENT_H
#define GRADIENT_H

#include <utility>
#include <vector>
#include <opencv2/core/core.hpp>

namespace lc_core
{

typedef std::pair<double, double> Gradient;

/// @brief  calculate gradient of two points
template<typename PointType>
std::vector<std::pair<PointType, Gradient>> calculateGradient(const std::vector<PointType>& points) {
    std::vector<std::pair<PointType, Gradient>> gradients;
    int num = points.size();
    for (int i = 0; i < num; ++i) {
        double dx = 0.0, dy = 0.0;
        if (i == 0) {
            dx = points[i + 1].x - points[i].x;
            dy = points[i + 1].y - points[i].y;
        } else if (i == num - 1) {
            dx = points[i].x - points[i - 1].x;
            dy = points[i].y - points[i - 1].y;
        } else {
            dx = (points[i + 1].x - points[i - 1].x) / 2.0;
            dy = (points[i + 1].y - points[i - 1].y) / 2.0;
        }
        gradients.push_back(std::make_pair(points[i], std::make_pair(dx, dy)));
    }

    return gradients;
}

} // namespace lc_core

#endif