/**
 * @file   bSpline.hpp
 * @brief  B-spline function.
 * @author Yipeng Zhao
 * @date   2024-08
 */

#include <vector>

/// @brief Recursive definition function for B-spline, with added checks and debugging information
template<typename T>
T B_spline(int i, int k, double t, const std::vector<T>& knots) {
    if (k == 1) {
        return (knots[i] <= t && t < knots[i + 1]) ? 1.0 : 0.0;
    }
    T denom1 = knots[i + k - 1] - knots[i];
    T denom2 = knots[i + k] - knots[i + 1];
    T term1 = (denom1 != 0.0) ? (t - knots[i]) / denom1 * B_spline(i, k - 1, t, knots) : 0.0;
    T term2 = (denom2 != 0.0) ? (knots[i + k] - t) / denom2 * B_spline(i + 1, k - 1, t, knots) : 0.0;

    return term1 + term2;
}

/// @brief Calculate B-spline curve points with debugging and validation (2D)
template<typename PointType>
std::vector<PointType> calculateBSpline(const std::vector<PointType>& controlPoints, int numPoints) {
    int n = controlPoints.size() - 1;
    int k = 3; // degree = 3

    std::vector<double> knots(n + k + 2);
    for (int i = 0; i < knots.size(); ++i) {
        if (i < k) {
            knots[i] = 0.0;
        } else if (i <= n) {
            knots[i] = (i - k + 1) / (double)(n - k + 2);
        } else {
            knots[i] = 1.0;
        }
    }

    std::vector<PointType> splinePoints;
    for (int j = numPoints / 10; j < numPoints; ++j) {
        double t = j / (double)(numPoints - 1);
        double x = 0.0, y = 0.0;
        for (int i = 0; i <= n; ++i) {
            double basis = B_spline(i, k + 1, t, knots);
            x += basis * controlPoints[i].x;
            y += basis * controlPoints[i].y;
        }
        if (y > controlPoints[n - 1].y && y < controlPoints[0].y) {
            splinePoints.push_back(PointType(std::round(x), std::round(y))); 
        }         
    }
    return splinePoints;
}

/// @brief Sample points between two points on an image
template<typename PointType>
std::vector<PointType> samplePointsBetween(PointType p1, PointType p2, int num_samples) {
    std::vector<PointType> sampled_points;

    // 确保num_samples至少为2，这样才能包括p1和p2
    num_samples = std::max(2, num_samples);

    // 计算从p1到p2的向量
    double dx = (p2.x - p1.x) / static_cast<double>(num_samples - 1);
    double dy = (p2.y - p1.y) / static_cast<double>(num_samples - 1);

    // 生成均匀采样的点
    for (int i = 0; i < num_samples; ++i) {
        int x = static_cast<int>(p1.x + i * dx);
        int y = static_cast<int>(p1.y + i * dy);
        sampled_points.push_back(PointType(x, y));
    }

    return sampled_points;
}