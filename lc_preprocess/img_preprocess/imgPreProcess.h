/**
 * @file    imgPreProcess.h
 * @brief   This file defines the image preprocessing class
 * @author  Yipeng Zhao
 * @date    2024-11
 */

#ifndef IMG_PREPROCESS_H
#define IMG_PREPROCESS_H

#include "bSpline.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>
#include <opencv2/core/core.hpp>

namespace lc_preprocess
{

    class ImgPreProcess
    {
    public:
        ImgPreProcess() = default;
        ~ImgPreProcess() = default;

        /// @brief  set pre-curve points
        void setPoints(const std::string &file_name);

        /// @brief  optical flow tracking
        void track(cv::Mat &pre_img, cv::Mat &cur_img);

        /// @brief  re-interpolate points
        void reInterpolate();

        /// @brief  visualization
        void visualizeTracking(cv::Mat &cur_img);

        /// @brief  visualize re-interpolated points
        void visualizeReInterpolated(cv::Mat &cur_img);

        /// @brief  downsample for points
        std::vector<cv::Point2d> downsamplePoints(std::vector<cv::Point2d> &points);

        std::vector<cv::Point2d> get_cur_points()
        {
            return cur_img_points_;
        }

        bool Is_track()
        {
            return is_track;
        }

        std::vector<cv::Point2d> Get_pre_point()
        {
            return pre_img_points_back;
        }

    private:
        std::vector<cv::Point2d> pre_img_points_;
        std::vector<cv::Point2d> pre_img_points_back;
        std::vector<cv::Point2d> cur_img_points_;
        cv::Point2d start_point,end_point;
        bool is_track;
    };

} // namespace lc_preprocess

#endif