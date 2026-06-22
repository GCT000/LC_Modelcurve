/**
 * @file   visualization.h
 * @brief  visualization utils
 * @author zyp
 * @date   2025-01-15
 */

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

typedef std::vector<cv::Point2d> P2PMatchResult;

namespace lc_core
{

/// @brief  draw points on image
void drawPointsOnImage(const cv::Mat& img, const std::vector<cv::Point2d>& points, const std::string& filename);

/// @brief  draw match result on image
void drawMatchResultOnImage(const cv::Mat& img, const std::string& match_file, const std::string& filename);

}

#endif