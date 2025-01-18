#include "camera.h"

namespace lc_core
{

bool Camera::readIntrinsicParameters(const std::string &filename)
{
    LOG(INFO) << "camera yaml file:" << filename << "\n";
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        LOG(ERROR) << "Open YAML file failed!\n";
        return false;
    }
    std::string model_type;
    if (fs["model_type"].isNone())
    {
        // set pinhole default
        model_type = "PINHOLE";
    }
    else
    {
        fs["model_type"] >> model_type;
        if (model_type.compare("PINHOLE") != 0)
        {
            LOG(ERROR) << "Wrong camera model. Just support pinhole camera now.\n";
            return false;
        }
    }

    std::string camera_name;
    fs["camera_name"] >> camera_name;
    img_w_ = static_cast<int>(fs["image_width"]);
    img_h_ = static_cast<int>(fs["image_height"]);
    cv::FileNode n = fs["distortion_parameters"];
    k1_ = static_cast<double>(n["k1"]);
    k2_ = static_cast<double>(n["k2"]);
    p1_ = static_cast<double>(n["p1"]);
    p2_ = static_cast<double>(n["p2"]);
    k3_ = static_cast<double>(n["p3"]);
    n = fs["projection_parameters"];
    fx_ = static_cast<double>(n["fx"]);
    fy_ = static_cast<double>(n["fy"]);
    cx_ = static_cast<double>(n["cx"]);
    cy_ = static_cast<double>(n["cy"]);
    return true;
}

void Camera::liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P)
{
    double m_inv_K11 = 1.0 / fx_;
    double m_inv_K13 = -cx_ / fx_;
    double m_inv_K22 = 1.0 / fy_;
    double m_inv_K23 = -cy_ / fy_;
    // double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u
    // double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv
    double mx_d, my_d, mx_u, my_u;
    mx_d = m_inv_K11 * p(0) + m_inv_K13;
    my_d = m_inv_K22 * p(1) + m_inv_K23;
    if (k1_ == 0 && k2_ == 0 && p1_ == 0 && p2_ == 0)
    {
        mx_u = mx_d;
        my_u = my_d;
    }
    int n = 8;
    Eigen::Vector2d d_u;
    distortion(Eigen::Vector2d(mx_d, my_d), d_u);
    // Approximate value
    mx_u = mx_d - d_u(0);
    my_u = my_d - d_u(1);
    for (int i = 1; i < n; ++i)
    {
        distortion(Eigen::Vector2d(mx_u, my_u), d_u);
        mx_u = mx_d - d_u(0);
        my_u = my_d - d_u(1);
    }
    P << mx_u, my_u, 1.0;
}

void Camera::distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u)
{
    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1_ * rho2_u + k2_ * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1_ * mxy_u + p2_ * (rho2_u + 2.0 * mx2_u),
        p_u(1) * rad_dist_u + 2.0 * p2_ * mxy_u + p1_ * (rho2_u + 2.0 * my2_u);
}

void Camera::undistortImg(const cv::Mat &distorted_img, cv::Mat &undistorted_img)
{
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx_, 0, cx_,
                            0, fy_, cy_,
                            0, 0, 1);

    // distortion coefficients
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1_, k2_, p1_, p2_, k3_);

    cv::undistort(distorted_img, undistorted_img, cameraMatrix, distCoeffs);
}
}