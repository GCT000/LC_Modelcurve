#include "matcher.h"
#include <unordered_map>
#include <Eigen/Dense>
#include <fstream>

static std::string temp_path = "/home/gct/LC_Modelcurve/data/temp/";

using namespace lc_core;

P2PMatchResult Matcher::match(const std::vector<cv::Point2d>& input, const std::vector<cv::Point2d>& source) {
    kd_tree_ = std::make_unique<KdTree>();
    kd_tree_->build(source);
    return p2pMatch(input, source);
}

P2PMatchResult Matcher::p2pMatch(const std::vector<cv::Point2d>& input, const std::vector<cv::Point2d>& source) {
    P2PMatchResult result;
    result.reserve(input.size());
    // Ensure that points in the source are not matched repeatedly
    std::fstream output_points(temp_path + "match.txt", std::ios::out);
    for (const cv::Point2d& ip : input) {
        std::vector<int> closest_idx;
        kd_tree_->getClosestPoint(ip, closest_idx, 1);
        result.emplace_back(source[closest_idx[0]].x, source[closest_idx[0]].y);
        output_points << "Match result: " << ip.x << " " << ip.y << " -- " << source[closest_idx[0]].x << " " << source[closest_idx[0]].y << "\n";
    }
    output_points.close();
    return result;
}