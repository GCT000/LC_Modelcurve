#include "evaluation.h"

namespace lc_core
{

std::pair<double, double> calculateReprojectError(const P2PMatchResult& result, 
                                                    const std::vector<cv::Point2d>& curve_points)
{
    return calMatchError(result, curve_points);
}

std::pair<double, double> calMatchError(const P2PMatchResult& result, const std::vector<cv::Point2d>& curve_points)
{
    double avg_err = 0.0;
    double max_err = 0.0;
    for (size_t i = 0; i < result.size(); ++i) {
        double err = cv::norm(result[i] - curve_points[i]);
        avg_err += err;
        max_err = std::max(max_err, err);
    }
    return std::make_pair(avg_err / result.size(), max_err);
}

}