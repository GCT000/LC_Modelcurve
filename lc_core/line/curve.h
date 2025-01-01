/**
 * @file   curve.h
 * @brief  This file defines curve model.
 * @todo   Processing for realistic images can be more complex.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#ifndef CURVE_H
#define CURVE_H

#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <glog/logging.h>

namespace lc_core
{

class Curve
{
public:
    Curve(bool visualize = false) : visualize_(visualize) {};

    ~Curve() = default;

    /// @brief  detection
    void curveDetection(const cv::Mat &image);

    [[maybe_unused]]void featuresDetection(const cv::Mat &image);

    /// @brief  curve judgment
    /// TODO: need to be improved
    bool isCurve([[maybe_unused]] const std::vector<cv::Point> &points, [[maybe_unused]] const cv::Vec4i &hierarchy)
    {
        return true;
    }

    /// @brief  get curve lines
    std::vector<std::vector<cv::Point>> getCurveLines()
    {
        return curve_lines_;
    }


private:
    std::vector<std::vector<cv::Point>> curve_lines_;
    bool visualize_;
};

} // namespace lc_core

#endif  // CURVE_H