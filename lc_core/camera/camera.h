/**
 * @file   camera.h
 * @brief  This file defines camera model(pinhole).
 * @author Yipeng Zhao
 * @date   2024-07
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include <memory>

namespace lc_core
{

class Camera  
{
public:
    Camera() = default;

    /// @brief  read intrinsic parameters
    bool readIntrinsicParameters(const std::string &filename);
    
    /// @brief  Lifts a point from the image plane to its projective ray
    void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P);

    /// @brief  distortion to input point
    void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u);

    //  @brief  undistort image
    void undistortImg(const cv::Mat &distorted_img, cv::Mat &undistorted_img);

    /// @brief  project a point from 3D to 2D using internal parameters
    template <typename T>
    void spaceToPlane(const Eigen::Matrix<T, 3, 1> &P, Eigen::Matrix<T, 2, 1> &p) const
    {
        p(0) = T(fx_) * P(0) / P(2) + T(cx_);
        p(1) = T(fy_) * P(1) / P(2) + T(cy_);
    }

    /// @brief  undistort points
    template <typename T>
    void undistortPoints(const std::vector<cv::Point_<T>> &distorted_points, std::vector<cv::Point_<T>> &undistorted_points) const
    {
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx_, 0, cx_,
                                0, fy_, cy_,
                                0, 0, 1);

        cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1_, k2_, p1_, p2_, k3_);

        cv::undistortPoints(distorted_points, undistorted_points, cameraMatrix, distCoeffs);

        // coor in pixel coor
        for (auto &point : undistorted_points)
        {
            point.x = point.x * fx_ + cx_;
            point.y = point.y * fy_ + cy_;
        }
    }

    template <typename T>
    void undistortPoints(const cv::Point_<T> &distorted_point, Eigen::Matrix<T, 2, 1> &undistorted_point) const
    {
        std::vector<cv::Point_<T>> src = {distorted_point};
        std::vector<cv::Point_<T>> dst;

        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx_, 0, cx_,
                                0, fy_, cy_,
                                0, 0, 1);

        cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1_, k2_, p1_, p2_, k3_);

        cv::undistortPoints(src, dst, cameraMatrix, distCoeffs);

        undistorted_point(0) = dst[0].x * fx_ + cx_;
        undistorted_point(1) = dst[0].y * fy_ + cy_;
    }

public:
    int img_h_;
    int img_w_;
    double fx_, fy_, cx_, cy_;
    double k1_, k2_, p1_, p2_, k3_;
};

} // namespace lc_core
#endif // CAMERA_H