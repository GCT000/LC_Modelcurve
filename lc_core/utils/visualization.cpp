#include "visualization.h"
#include <fstream>

namespace lc_core
{

void drawPointsOnImage(const cv::Mat& img, const std::vector<cv::Point2d>& points, const std::string& filename)
{
    cv::Mat _img = img.clone();
    for (const auto& point : points) {
        cv::circle(_img, point, 1, cv::Scalar(0, 255, 0), -1);
    }
    cv::imwrite(filename, _img);
}


void drawMatchResultOnImage(const cv::Mat& img, const std::string& match_file, const std::string& filename)
{
    std::fstream output_points(match_file, std::ios::in);
    cv::Mat match_img = img.clone();
    std::string line;
    while (std::getline(output_points, line)) {
        double x1, y1, x2, y2;
        if (sscanf(line.c_str(), "Match result: %lf %lf -- %lf %lf", &x1, &y1, &x2, &y2) == 4) {
            // draw points
            cv::circle(match_img, cv::Point(x1, y1), 3, cv::Scalar(0, 0, 255), -1);  // red
            cv::circle(match_img, cv::Point(x2, y2), 3, cv::Scalar(0, 255, 0), -1);  // green
            // draw line
            cv::line(match_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), 1);
        }
    }
    cv::imwrite(filename, match_img);
    output_points.close();
}

}