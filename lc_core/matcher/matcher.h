/**
 * @file   matcher.h
 * @brief  This file contains the declaration of the Matcher class
 * @author Yipeng Zhao
 * @date   2024/09
*/

#ifndef MATCHER_H
#define MATCHER_H

#include <vector>
#include <variant>
#include <glog/logging.h>
#include <opencv2/core/types.hpp>
#include "kdtree.h"

namespace lc_core
{

typedef std::vector<cv::Point2d> P2PMatchResult;

class Matcher {
public:
    /// @brief  constructor
    Matcher(){};

    /// @brief  match interface
    P2PMatchResult match(const std::vector<cv::Point2d>& input, const std::vector<cv::Point2d>& source);

    /// @brief  point to point match
    P2PMatchResult p2pMatch(const std::vector<cv::Point2d>& input, const std::vector<cv::Point2d>& source);

private:
    std::unique_ptr<KdTree> kd_tree_;
};

} // namespace lc_core
#endif