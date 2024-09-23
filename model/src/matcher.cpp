#include "matcher.h"
#include <unordered_map>
#include <Eigen/Dense>

double distance(const cv::Point2d& p1, const cv::Point2d& p2) {
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

/// @brief reverse points
void reverse(std::vector<cv::Point2d>& points) {
    int left = 0, right = points.size() - 1;
    while (left <= right) {
        cv::Point2d temp = points[left];
        points[left] = points[right];
        points[right] = temp;
        ++left;
        --right;
    }
}

std::vector<cv::Point2d> interpolate(const cv::Point2d& p1, const cv::Point2d& p2, int num) {
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
    // resample source points as match points
    // TODO: kd-tree?
    std::vector<cv::Point2d> vp2 = input;
    std::vector<cv::Point2d> vp1 = source;
    reverse(vp1);
    reverse(vp2);
    P2PMatchResult result = resample(vp1, vp2);
    reverse(result);

    return result;
}

std::vector<cv::Point2d> Matcher::resample(const std::vector<cv::Point2d>& vp1, const std::vector<cv::Point2d>& vp2) {
    std::vector<cv::Point2d> resampled_vp1;
    resampled_vp1.push_back(vp1[0]);

    int vp1_index = 1;
    double accumulatedDistance = 0.0;

    for (int i = 1; i < vp2.size(); ++i) {
        double vp2_distance = distance(vp2[i], vp2[i - 1]);

        while (vp1_index < vp1.size() && accumulatedDistance < vp2_distance) {
            double vp1_distance = distance(vp1[vp1_index], vp1[vp1_index - 1]);
            accumulatedDistance += vp1_distance;
            if (accumulatedDistance < vp2_distance) {
                resampled_vp1.push_back(vp1[vp1_index]);
            }
            vp1_index++;
        }

        if (accumulatedDistance > vp2_distance) {
            double overshoot = accumulatedDistance - vp2_distance;
            double lastSegmentDistance = distance(vp1[vp1_index - 1], vp1[vp1_index - 2]);
            double interpolationFactor = 1.0 - (overshoot / lastSegmentDistance);

            cv::Point2d interpolatedPoint = (1 - interpolationFactor) * vp1[vp1_index - 2] + interpolationFactor * vp1[vp1_index - 1];
            resampled_vp1.push_back(interpolatedPoint);
            accumulatedDistance = overshoot;
        }
    }

    return resampled_vp1;
}

P2LMatchResult Matcher::p2lMatch(const std::vector<cv::Point2d>& input, const std::vector<cv::Point2d>& source) {
    P2LMatchResult result;

    for (const cv::Point2d& ip : input) {
        std::vector<std::pair<double, Eigen::Vector2d>> distances;
        for (const cv::Point2d& p : source) {
            double dis = distance(ip, p);
            // LOG(INFO) << "img: " << p_img.x() << " " << p_img.y() << ", dis: " << distance;
            distances.push_back(std::make_pair(dis, Eigen::Vector2d(p.x, p.y)));
        }

        std::sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });

        // find 5 closest points
        std::vector<Eigen::Vector2d> closestPoints;
        for (size_t i = 0; i < 5 && i < distances.size(); ++i) {
            closestPoints.push_back(distances[i].second);
        }

        // fit line using least squares
        // TODO: try NDT?
        Eigen::MatrixXd A2(closestPoints.size(), 2);
        Eigen::VectorXd b2(closestPoints.size());
        for (int i = 0; i < closestPoints.size(); ++i) {
            A2(i, 0) = closestPoints[i](0);
            A2(i, 1) = 1.0;
            b2(i) = closestPoints[i](1);
        }
        Eigen::VectorXd x2 = A2.colPivHouseholderQr().solve(b2);
        result.push_back(std::make_pair(x2(0), x2(1)));
    }

    return result;
}