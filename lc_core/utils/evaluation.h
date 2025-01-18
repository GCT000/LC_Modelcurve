/**
 * @file   evaluation.h
 * @brief  This file contains the evaluation of the curve modeling.
 * @author Yipeng Zhao
 * @date   2025-01
 */

#ifndef EVALUATION_H
#define EVALUATION_H

#include "matcher.h"
#include <vector>
#include <opencv2/core/types.hpp>

namespace lc_core
{

///@brief  average reproject error in image plane
std::pair<double, double> calculateReprojectError(const Matcher::MatchResult& result,
                                                    const std::vector<cv::Point2d>& curve_points);

std::pair<double, double> calMatchError(const P2PMatchResult& result, const std::vector<cv::Point2d>& curve_points);
std::pair<double, double> calMatchError(const P2LMatchResult& result, const std::vector<cv::Point2d>& curve_points);

}

#endif // EVALUATION_H