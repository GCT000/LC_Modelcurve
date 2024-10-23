#include "matcher.h"
#include <unordered_map>
#include <Eigen/Dense>
#include <fstream>
[[maybe_unused]] std::vector<cv::Point2d> interpolate(const cv::Point2d& p1, const cv::Point2d& p2, int num) {
    // Edge case: if num == 0, return an empty vector
    if (num <= 0) {
        return {};
    }

    std::vector<cv::Point2d> interpolatedPoints;
    double dx = (p2.x - p1.x) / (num + 1);
    double dy = (p2.y - p1.y) / (num + 1);

    for (int i = 1; i <= num; ++i) {
        cv::Point2d newPoint(p1.x + i * dx, p1.y + i * dy);
        interpolatedPoints.push_back(newPoint);
    }
    return interpolatedPoints;
}

Matcher::Matcher(const MatcherConfig& config) {
    matcher_type_ = static_cast<MatcherType>(config.type);
}

Matcher::MatchResult Matcher::match(const std::vector<cv::Point2d>& input, const std::vector<cv::Point2d>& source) {
    kd_tree_ = std::make_unique<KdTree>();
    kd_tree_->build(source);
    switch (matcher_type_) {
        case MatcherType::P2P:
            return p2pMatch(input, source);
        case MatcherType::P2L:
            return p2lMatch(input, source);
        default:
            throw std::invalid_argument("Unknown match type");
    }
}

P2PMatchResult Matcher::p2pMatch(const std::vector<cv::Point2d>& input, const std::vector<cv::Point2d>& source) {
    P2PMatchResult result;
    result.reserve(input.size());
    // Ensure that points in the source are not matched repeatedly
    std::vector<int> visited(source.size(), 0);
    std::fstream output_points("match.txt", std::ios::out);
    for (const cv::Point2d& ip : input) {
        std::vector<int> closest_idx;
        kd_tree_->getClosestPoint(ip, closest_idx, 10);
        for (int idx : closest_idx) {
            if (visited[idx] == 0) {
                visited[idx] = 1;
                result.emplace_back(source[idx].x, source[idx].y);
                output_points << "Match result: " << ip.x << " " << ip.y << " -- " << source[idx].x << " " << source[idx].y << "\n";
                break;
            }
        }
    }

    return result;
}

P2LMatchResult Matcher::p2lMatch(const std::vector<cv::Point2d>& input, const std::vector<cv::Point2d>& source) {
    P2LMatchResult result;
    result.reserve(input.size());

    for (const cv::Point2d& ip : input) {
        std::vector<int> closest_idx;
        kd_tree_->getClosestPoint(ip, closest_idx, 8);

        // fit line using least squares
        Eigen::MatrixXd A2(closest_idx.size(), 2);
        Eigen::VectorXd b2(closest_idx.size());
        for (int i = 0; i < closest_idx.size(); ++i) {
            A2(i, 0) = source[closest_idx[i]].x;
            A2(i, 1) = 1.0;
            b2(i) = source[closest_idx[i]].y;
        }
        Eigen::VectorXd x2 = A2.colPivHouseholderQr().solve(b2);
        result.push_back(std::make_pair(x2(0), x2(1)));
    }

    return result;
}

P2PMatchResult Matcher::updateMatch(const std::vector<cv::Point2d>& input, const std::vector<cv::Point2d>& source) {
    // the same as p2pMatch
    P2PMatchResult result;
    for (const cv::Point2d& ip : input) {
        std::vector<int> closest_idx;
        kd_tree_->getClosestPoint(ip, closest_idx, 1);
        result.emplace_back(source[closest_idx[0]].x, source[closest_idx[0]].y);
    }
    // TODO update source points

    return result;
}