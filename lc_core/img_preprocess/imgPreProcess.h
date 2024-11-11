/**
 * @file    imgPreProcess.h
 * @brief   This file defines the image preprocessing class
 * @author  Yipeng Zhao
 * @date    2024-11
 */

#ifndef IMG_PREPROCESS_H
#define IMG_PREPROCESS_H

#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/core/core.hpp>

namespace lc_core
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

private:
    std::vector<cv::Point2d> pre_img_points_;
    std::vector<cv::Point2d> cur_img_points_;
};

} // namespace lc_core

#endif