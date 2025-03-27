/**
 * @file   bSpline.h
 * @brief  B-spline function.
 * @author Yipeng Zhao
 * @date   2024-08
 */

#ifndef BSPLINE_H
#define BSPLINE_H

#include <vector>
#include <cmath>
#include <glog/logging.h>

namespace lc_core
{

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

/// @brief Sample points between two points on an image
template<typename PointType>
std::vector<PointType> samplePointsBetween(PointType p1, PointType p2, int num_samples) {
    std::vector<PointType> sampled_points;

    // num_samples >= 2
    num_samples = std::max(2, num_samples);

    // p1 -> p2
    double dx = (p2.x - p1.x) / static_cast<double>(num_samples - 1);
    double dy = (p2.y - p1.y) / static_cast<double>(num_samples - 1);

    // generate points
    for (int i = 0; i < num_samples; ++i) {
        int x = static_cast<int>(p1.x + i * dx);
        int y = static_cast<int>(p1.y + i * dy);
        sampled_points.push_back(PointType(x, y));
    }

    return sampled_points;
}

/// @brief Calculate B-spline curve points with debugging and validation (2D)
template<typename PointType>
std::vector<PointType> calculateBSpline(const std::vector<PointType>& controlPoints, int numPoints) {
    if (controlPoints.size() < 4) {
        LOG(WARNING) << "Need at least 4 control points to generate a cubic B-spline curve";
        return std::vector<PointType>();
    }
    
    int n = controlPoints.size() - 1;
    int k = 3; // cubic B-spline

    // generate knots vector
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

    // ensure the number of sampling points is enough
    numPoints = std::max(numPoints, (n + 1) * 10);
    
    std::vector<PointType> splinePoints;
    splinePoints.reserve(numPoints);

    // sample the whole curve from 0
    for (int j = 0; j < numPoints; ++j) {
        double t = j / (double)(numPoints - 1);
        double x = 0.0, y = 0.0;
        
        for (int i = 0; i <= n; ++i) {
            double basis = B_spline(i, k + 1, t, knots);
            x += basis * controlPoints[i].x;
            y += basis * controlPoints[i].y;
        }
        
        // round the coordinates to get the pixel position
        splinePoints.push_back(PointType(std::round(x), std::round(y)));
    }

    // ensure the continuity of the curve
    if (splinePoints.size() >= 2) {
        std::vector<PointType> densifiedPoints;
        densifiedPoints.reserve(splinePoints.size() * 2);
        
        for (size_t i = 0; i < splinePoints.size() - 1; ++i) {
            const auto& p1 = splinePoints[i];
            const auto& p2 = splinePoints[i + 1];
            
            // calculate the distance between two points
            double distance = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
            
            // if the distance between two points is greater than 1 pixel, insert intermediate points
            if (distance > 1.0) {
                auto intermediatePoints = samplePointsBetween(p1, p2, std::ceil(distance));
                densifiedPoints.insert(densifiedPoints.end(), intermediatePoints.begin(), intermediatePoints.end());
            } else {
                densifiedPoints.push_back(p1);
            }
        }
        densifiedPoints.push_back(splinePoints.back());
        splinePoints = std::move(densifiedPoints);
    }

    LOG(INFO) << "Generate " << splinePoints.size() << " spline points.";
    return splinePoints;
}

/// @brief Calculate Catmull-Rom spline curve points with debugging and validation (2D)
template<typename PointType>
std::vector<PointType> calculateCatmullRomSpline(const std::vector<PointType>& controlPoints, int numPoints) {
    if (controlPoints.size() < 2) {
        LOG(WARNING) << "Need at least 2 control points to generate a Catmull-Rom spline";
        return std::vector<PointType>();
    }
    
    std::vector<PointType> splinePoints;
    splinePoints.reserve(numPoints);
    
    // add extra points for boundary
    std::vector<PointType> extendedPoints;
    extendedPoints.reserve(controlPoints.size() + 2);
    
    // add start point
    PointType startPoint;
    startPoint.x = 2 * controlPoints[0].x - controlPoints[1].x;
    startPoint.y = 2 * controlPoints[0].y - controlPoints[1].y;
    extendedPoints.push_back(startPoint);
    
    // add original points
    extendedPoints.insert(extendedPoints.end(), controlPoints.begin(), controlPoints.end());
    
    // add end point
    PointType endPoint;
    endPoint.x = 2 * controlPoints.back().x - controlPoints[controlPoints.size()-2].x;
    endPoint.y = 2 * controlPoints.back().y - controlPoints[controlPoints.size()-2].y;
    extendedPoints.push_back(endPoint);
    
    // ensure the number of sampling points is enough
    numPoints = std::max(numPoints, static_cast<int>(controlPoints.size()));
    
    // sample each segment
    for (size_t i = 1; i < extendedPoints.size() - 2; ++i) {
        const auto& p0 = extendedPoints[i-1];
        const auto& p1 = extendedPoints[i];
        const auto& p2 = extendedPoints[i+1];
        const auto& p3 = extendedPoints[i+2];
        
        // calculate the number of sampling points for each segment
        int segmentPoints = numPoints / (controlPoints.size() - 1);
        
        for (int j = 0; j < segmentPoints; ++j) {
            double t = j / static_cast<double>(segmentPoints);
            double t2 = t * t;
            double t3 = t2 * t;
            
            // Catmull-Rom basis functions
            double b0 = -0.5 * t3 + t2 - 0.5 * t;
            double b1 = 1.5 * t3 - 2.5 * t2 + 1.0;
            double b2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t;
            double b3 = 0.5 * t3 - 0.5 * t2;
            
            double x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
            double y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
            
            splinePoints.push_back(PointType(std::round(x), std::round(y)));
        }
    }
    
    // ensure the start and end points are included
    splinePoints.front() = controlPoints.front();
    splinePoints.back() = controlPoints.back();
    
    // densify the points to ensure pixel continuity
    std::vector<PointType> densifiedPoints;
    densifiedPoints.reserve(splinePoints.size() * 2);
    
    for (size_t i = 0; i < splinePoints.size() - 1; ++i) {
        const auto& p1 = splinePoints[i];
        const auto& p2 = splinePoints[i + 1];
        
        densifiedPoints.push_back(p1);
        
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance > 1.0) {
            int steps = std::ceil(distance * 2);
            for (int j = 1; j < steps; ++j) {
                double ratio = j / static_cast<double>(steps);
                int x = std::round(p1.x + dx * ratio);
                int y = std::round(p1.y + dy * ratio);
                densifiedPoints.push_back(PointType(x, y));
            }
        }
    }
    densifiedPoints.push_back(splinePoints.back());

    LOG(INFO) << "Generate " << densifiedPoints.size() << " spline points";
    return densifiedPoints;
}

} // namespace lc_core

#endif // BSPLINE_H
