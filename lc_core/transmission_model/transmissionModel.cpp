#include "transmissionModel.h"

using namespace lc_core;

std::pair<double, double> TransmissionModel::ransacFitLine(std::vector<Eigen::Vector3d> &points)
{
    const int iterations = 1000;   // RANSAC
    const double threshold = 0.03; // inlier threshold
    double best_k = 0, best_m = 0;
    int max_inliers = 0;
    std::vector<bool> best_inliers;

    std::random_device rd;
    std::mt19937 gen(rd());

    for (int iter = 0; iter < iterations; ++iter)
    {
        // randomly select 2 points
        std::uniform_int_distribution<> dis(0, points.size() - 1);
        int idx1 = dis(gen);
        int idx2 = dis(gen);
        if (idx1 == idx2)
            continue;

        // calculate line parameters using 2 points
        double x1 = points[idx1](0), y1 = points[idx1](1);
        double x2 = points[idx2](0), y2 = points[idx2](1);
        double k = (y2 - y1) / (x2 - x1);
        double m = y1 - k * x1;

        // count inliers
        std::vector<bool> current_inliers(points.size(), false);
        int inlier_count = 0;
        for (size_t i = 0; i < points.size(); ++i)
        {
            double x = points[i](0);
            double y = points[i](1);
            double dist = std::abs(k * x + m - y);
            if (dist < threshold)
            {
                current_inliers[i] = true;
                inlier_count++;
            }
        }

        // update best result
        if (inlier_count > max_inliers)
        {
            max_inliers = inlier_count;
            best_k = k;
            best_m = m;
            best_inliers = current_inliers;
        }
    }

    // refit using all inliers
    std::vector<Eigen::Vector3d> inlier_points;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (best_inliers[i])
        {
            inlier_points.push_back(points[i]);
        }
    }

    points = std::move(inlier_points);

#if 0
    Eigen::MatrixXd A2(points.size(), 2);
    Eigen::VectorXd b2(points.size());
    for (int i = 0; i < points.size(); ++i)
    {
        A2(i, 0) = points[i](0);
        A2(i, 1) = 1.0;
        b2(i) = points[i](1);
    }
    Eigen::VectorXd x2 = A2.colPivHouseholderQr().solve(b2);
    double k = x2(0);
    double m = x2(1);
#endif
    return std::make_pair(best_k, best_m);
}