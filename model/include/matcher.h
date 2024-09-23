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

typedef std::vector<cv::Point2d> P2PMatchResult;
typedef std::vector<std::pair<double, double>> P2LMatchResult;

struct MatcherConfig {
    int type;
};

class Matcher {
public:
    enum class MatcherType {
        P2P = 1,    // point to point
        P2L = 2     // point to line
    };

    /// @brief  constructor
    Matcher(const MatcherConfig& config);

    /// @brief  match interface
    using MatchResult = std::variant<P2PMatchResult, P2LMatchResult>;
    MatchResult match(const std::vector<cv::Point2d>& input, const std::vector<cv::Point2d>& source);

    /// @brief  point to point match
    P2PMatchResult p2pMatch(const std::vector<cv::Point2d>& input, const std::vector<cv::Point2d>& source);

    /// @brief  resample input points to source points' size
    std::vector<cv::Point2d> resample(const std::vector<cv::Point2d>& vp1, const std::vector<cv::Point2d>& vp2);

    /// @brief  point to line match
    P2LMatchResult p2lMatch(const std::vector<cv::Point2d>& input, const std::vector<cv::Point2d>& source);

private:
    MatcherType matcher_type_;
};


#endif