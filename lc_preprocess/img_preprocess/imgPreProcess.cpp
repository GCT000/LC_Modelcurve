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
    LOG(INFO) << "Read " << pre_img_points_.size() << " points from " << file_name;
}

void ImgPreProcess::track(cv::Mat &pre_img, cv::Mat &cur_img)
{
    // convert Point2d to Point2f
    std::vector<cv::Point2f> pre_points_f(pre_img_points_.begin(), pre_img_points_.end());
    std::vector<cv::Point2f> cur_points_f;
    
    // forward tracking
    std::vector<uchar> forward_status;
    std::vector<float> forward_err;
    cv::calcOpticalFlowPyrLK(pre_img, cur_img, pre_points_f, cur_points_f, forward_status, forward_err, cv::Size(7, 7), 0, 
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));

    // backward tracking
    std::vector<cv::Point2f> reverse_points;
    std::vector<uchar> backward_status;
    std::vector<float> backward_err;
    cv::calcOpticalFlowPyrLK(cur_img, pre_img, cur_points_f, reverse_points, backward_status, backward_err, cv::Size(7, 7), 0, 
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));

    // calculate bidirectional tracking error and filter points
    std::vector<cv::Point2d> filtered_pre_points, filtered_cur_points;
    const double max_err = 1.0;
    
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
    std::vector<cv::Point2d> temp_points(cur_img_points_.begin(), cur_img_points_.end());
    // re-interpolate points
    std::vector<cv::Point2d> interpolated_points;
    for (size_t i = 0; i < temp_points.size() - 1; ++i) {
        const cv::Point2d& p1 = temp_points[i];
        const cv::Point2d& p2 = temp_points[i + 1];
        
        double distance = cv::norm(p2 - p1);
        int num_points = std::ceil(distance);
        
        for (int j = 0; j < num_points; ++j) {
            double t = static_cast<double>(j) / num_points;
            cv::Point2d new_point = p1 + t * (p2 - p1);
            interpolated_points.push_back(new_point);
        }
    }
    // add the last point
    interpolated_points.push_back(temp_points.back());

    cur_img_points_.clear();
    cur_img_points_ = std::move(interpolated_points);
}

void ImgPreProcess::visualizeTracking(cv::Mat &cur_img)
{
    cv::Mat vis_img = cur_img.clone();
    // draw tracking points before tracking(blue)
    for(size_t i = 0; i < pre_img_points_.size(); i++) {
        cv::circle(vis_img, pre_img_points_[i], 2, cv::Scalar(255, 0, 0), -1);
    }
    // draw tracking points after tracking(red) and optical flow arrows(green)
    for(size_t i = 0; i < cur_img_points_.size(); i++) {
        cv::circle(vis_img, cur_img_points_[i], 2, cv::Scalar(0, 0, 255), -1);
        cv::arrowedLine(vis_img, pre_img_points_[i], cur_img_points_[i], 
                       cv::Scalar(0, 255, 0), 1, cv::LINE_AA, 0, 0.2);
    }
    cv::imwrite("track_result.png", vis_img);
}

void ImgPreProcess::visualizeReInterpolated(cv::Mat &cur_img)
{
    cv::Mat vis_img = cur_img.clone();
    for(size_t i = 0; i < cur_img_points_.size(); i++) {
        cv::circle(vis_img, cur_img_points_[i], 2, cv::Scalar(0, 0, 255), -1);
    }
    LOG(INFO) << "Re-interpolated points size: " << cur_img_points_.size();
    cv::imwrite("re_interpolated_result.png", vis_img);
}