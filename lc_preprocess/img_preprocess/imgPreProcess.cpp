#include "imgPreProcess.h"
#include <glog/logging.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace lc_preprocess;

void ImgPreProcess::setPoints(const std::string &file_name)
{
    std::ifstream in_file(file_name, std::ios::in);
    if (!in_file.is_open())
    {
        LOG(ERROR) << "Can't open file: " << file_name << std::endl;
        return;
    }
    cv::Point2d point;
    while (in_file >> point.x >> point.y)
    {
        pre_img_points_.push_back(point);
    }
    end_point = pre_img_points_.back();
    LOG(INFO) << "Read " << pre_img_points_.size() << " points from " << file_name;
}

void ImgPreProcess::track(cv::Mat &pre_img, cv::Mat &cur_img)
{
    // convert Point2d to Point2f
    std::vector<cv::Point2f> pre_points_f(pre_img_points_.begin(), pre_img_points_.end());
    std::vector<cv::Point2f> cur_points_f;
    cur_points_f.reserve(pre_points_f.size());

    // set optical flow parameters
    const auto window_size = cv::Size(35,35);
    const auto criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    // forward tracking
    std::vector<uchar> forward_status;
    std::vector<float> forward_err;
    cv::calcOpticalFlowPyrLK(pre_img, cur_img, pre_points_f, cur_points_f, forward_status, forward_err, window_size, 0, criteria);

    // backward tracking
    std::vector<cv::Point2f> reverse_points;
    std::vector<uchar> backward_status;
    std::vector<float> backward_err;
    cv::calcOpticalFlowPyrLK(cur_img, pre_img, cur_points_f, reverse_points, backward_status, backward_err, window_size, 0, criteria);

    // calculate bidirectional tracking error and filter points
    std::vector<cv::Point2d> filtered_pre_points, filtered_cur_points;
    const double max_err = 0.1;
    filtered_cur_points.push_back(end_point);
    for(size_t i = 0; i < pre_points_f.size(); i++) {
        if(forward_status[i] && backward_status[i]) {
            double tracking_err = cv::norm(cv::Point2d(pre_points_f[i].x, pre_points_f[i].y) -
                                         cv::Point2d(reverse_points[i].x, reverse_points[i].y));
            if(tracking_err < max_err) {
                filtered_pre_points.push_back(pre_img_points_[i]);
                filtered_cur_points.push_back(cv::Point2d(cur_points_f[i].x, cur_points_f[i].y));
            }
        }
    }

    // update tracking results
    pre_img_points_ = filtered_pre_points;
    cur_img_points_ = filtered_cur_points;
    LOG(INFO) << "After tracking, " << cur_img_points_.size() << " points are left";
}

void ImgPreProcess::reInterpolate()
{
    //cur_img_points_ = downsamplePoints(cur_img_points_);
    std::vector<cv::Point2d> temp_points(cur_img_points_.begin(), cur_img_points_.end());
    // re-interpolate points
    std::vector<cv::Point2d> interpolated_points;
    interpolated_points.reserve(temp_points.size() * 2);
    interpolated_points = lc_core::calculateCatmullRomSpline(temp_points, 1000);
    // add the last point
    interpolated_points.push_back(temp_points.back());
    cur_img_points_ = std::move(interpolated_points);
}

void ImgPreProcess::visualizeTracking(cv::Mat &cur_img)
{
    cv::Mat vis_img = cur_img.clone();
    // draw tracking points before tracking(blue)
    for (size_t i = 0; i < pre_img_points_.size(); i++)
    {
        cv::circle(vis_img, pre_img_points_[i], 2, cv::Scalar(255, 0, 0), -1);
    }
    // draw tracking points after tracking(red) and optical flow arrows(green)
    for (size_t i = 0; i < cur_img_points_.size(); i++)
    {
        cv::circle(vis_img, cur_img_points_[i], 2, cv::Scalar(0, 0, 255), -1);
        cv::arrowedLine(vis_img, pre_img_points_[i], cur_img_points_[i],
                        cv::Scalar(0, 255, 0), 1, cv::LINE_AA, 0, 0.2);
    }
    std::string name = "/home/gct/LC-CurveModel/data/tempp/track_result.png";
    cv::imwrite(name, vis_img);
}

void ImgPreProcess::visualizeReInterpolated(cv::Mat &cur_img)
{
    cv::Mat vis_img = cur_img.clone();
    for (size_t i = 0; i < cur_img_points_.size(); i++)
    {
        cv::circle(vis_img, cur_img_points_[i], 2, cv::Scalar(0, 0, 255), -1);
    }
    LOG(INFO) << "Re-interpolated points size: " << cur_img_points_.size();
    pre_img_points_ = cur_img_points_;
    std::string name = "/home/gct/LC-CurveModel/data/temp/re_interpolated_result.png";
    cv::imwrite(name, vis_img);
}

std::vector<cv::Point2d> ImgPreProcess::downsamplePoints(std::vector<cv::Point2d>& points) {
    std::vector<cv::Point2d> downsampledPoints;
    std::unordered_set<int> usedXValues;

    for (const auto& point : points) {
        int roundedX = static_cast<int>(point.x);
        if (usedXValues.find(roundedX) == usedXValues.end()) {
            downsampledPoints.push_back(point);
            usedXValues.insert(roundedX);
        }
    }

    return downsampledPoints;
}