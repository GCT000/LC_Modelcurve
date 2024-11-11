#include "ex_optimization.h"
#include <glog/logging.h>

using namespace lc_core;

void ExOptimization::loadLidarPoints(const std::string& lidar_points_file)
{
    std::fstream input(lidar_points_file, std::ios::in);
    if (!input)
    {
        LOG(ERROR) << "Cannot open file: " << lidar_points_file << "\n";
        return;
    }

    std::string line;
    // load selected lidar points
    while (std::getline(input, line))
    {
        std::istringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ',')) {
            double x = std::stod(token);
            std::getline(iss, token, ',');
            double y = std::stod(token);
            std::getline(iss, token, ',');
            double z = std::stod(token);
            lidar_points_.push_back(Eigen::Vector3d(x, y, z));
        }
    }

    LOG(INFO) << "Load " << lidar_points_.size() << " lidar reference points.\n";
}

void ExOptimization::loadImgPoints(const std::string& img_ref_points_file)
{
    std::fstream file(img_ref_points_file, std::ios::in);
    if (!file.is_open())
    {
        LOG(ERROR) << "Open input points file failed!";
        return;
    }
    std::string line;

    // load selected image points
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ',')) {
            double x = std::stod(token);
            std::getline(iss, token, ',');
            double y = std::stod(token);
            img_ref_points_.push_back(cv::Point(x, y));
        }
    }

    LOG(INFO) << "Load " << img_ref_points_.size() << " image reference points.\n";
}

/// @brief ex optimization factor
struct ExFactor {
    ExFactor(const Eigen::Vector3d& lidar_point, const cv::Point& img_point, std::shared_ptr<Camera> cam)
        : lidar_point_(lidar_point), img_point_(img_point), cam_(cam) {}

    template <typename T>
    bool operator()(const T* const q, const T* const t, T* residuals) const {
        Eigen::Quaternion<T> q_c_l(q[3], q[0], q[1], q[2]);
        Eigen::Matrix<T, 3, 1> t_c_l(t[0], t[1], t[2]);
        Eigen::Matrix<T, 3, 1> p_c = q_c_l * lidar_point_.cast<T>() + t_c_l;
        Eigen::Matrix<T, 2, 1> p_img;
        cam_->spaceToPlane(p_c, p_img);
        residuals[0] = p_img(0) - T(img_point_.x);
        residuals[1] = p_img(1) - T(img_point_.y);
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& lidar_point, const cv::Point& img_point, std::shared_ptr<Camera> cam) {
        return new ceres::AutoDiffCostFunction<ExFactor, 2, 4, 3>(new ExFactor(lidar_point, img_point, cam));
    }

    Eigen::Vector3d lidar_point_;
    cv::Point img_point_;
    std::shared_ptr<Camera> cam_;
};

void ExOptimization::optimization() {
    Eigen::Quaterniond q_c_l(R_c_l_);

    double q[4] = {q_c_l.x(), q_c_l.y(), q_c_l.z(), q_c_l.w()};
    double t[3] = {t_c_l_(0), t_c_l_(1), t_c_l_(2)};

    LOG(INFO) << "Before optimization: \n";
    LOG(INFO) << "q: " << q_c_l.coeffs().transpose() << " t: " << t_c_l_.transpose() << "\n";

    ceres::Problem problem;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    // options.max_num_iterations = 8;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    problem.AddParameterBlock(q, 4, new ceres::EigenQuaternionParameterization());
    problem.AddParameterBlock(t, 3);
    problem.SetParameterBlockConstant(t);

    for (size_t i = 0; i < lidar_points_.size(); ++i) {
        ceres::CostFunction* cost_function = ExFactor::Create(lidar_points_[i], img_ref_points_[i], cam_);
        problem.AddResidualBlock(cost_function, nullptr, q, t);
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    q_c_l = Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
    R_c_l_ = q_c_l.toRotationMatrix();
    t_c_l_ << t[0], t[1], t[2];

    LOG(INFO) << "After optimization: \n";
    LOG(INFO) << "q: " << q_c_l.coeffs().transpose() << " t: " << t_c_l_.transpose() << "\n";
}