#include "ex_optimization.h"
#include <glog/logging.h>

using namespace lc_core;

ExOptimization::ExOptimization(const ExOptimization& other)
        : R_c_l_(other.R_c_l_),
          t_c_l_(other.t_c_l_),
          lidar_points_(other.lidar_points_),
          img_ref_points_(other.img_ref_points_)
{
    if (other.cam_) {
        cam_ = std::make_shared<Camera>(*other.cam_);
    }
}

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

void ExOptimization::setLidarPoints(const std::vector<Eigen::Vector3d>& lidar_points)
{
    if (lidar_points.empty()) {
        LOG(ERROR) << "Lidar points is empty!";
        return;
    }

    std::vector<Eigen::Vector3d> sorted_points = lidar_points;
    std::sort(sorted_points.begin(), sorted_points.end(), 
              [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                  return a.x() < b.x();
              });
    
    double min_x = sorted_points.front().x();

    std::vector<Eigen::Vector3d> processed_points;
    
    // divide the lidar points into bins
    double bin_size = 0.25;
    double max_x = min_x + 15 * bin_size;
    double radius = 0.05; // 5cm radius
    
    for (double x = min_x; x <= max_x; x += bin_size) {
        double x_center = x + bin_size / 2.0;
        
        // collect the points in the current interval
        std::vector<Eigen::Vector3d> bin_points;
        for (const auto& point : sorted_points) {
            if (point.x() >= x && point.x() < x + bin_size) {
                bin_points.push_back(point);
            }
        }
        
        if (!bin_points.empty()) {
            // calculate the average of y and z coordinates
            double sum_y = 0.0, sum_z = 0.0;
            int count = 0;
            
            for (const auto& point : bin_points) {
                // check if the point is in the radius
                if (std::abs(point.x() - x_center) <= radius) {
                    sum_y += point.y();
                    sum_z += point.z();
                    count++;
                }
            }
            
            if (count > 0) {
                double avg_y = sum_y / count;
                double avg_z = sum_z / count;
                processed_points.emplace_back(x_center, avg_y, avg_z);
            }
        }
    }
    
    // store the processed points
    lidar_points_ = processed_points;
    LOG(INFO) << "Processed " << lidar_points_.size() << " lidar points.\n";
}

void ExOptimization::setImgPoints(const std::vector<cv::Point2d>& projected_points, const std::vector<cv::Point2d>& ori_img_points)
{
    Matcher matcher(MatcherConfig{1});
    std::vector<cv::Point2d> img_points;
    auto [min_x_it, max_x_it] = std::minmax_element(
        projected_points.begin(), projected_points.end(),
        [](const cv::Point2d& a, const cv::Point2d& b) { return a.x < b.x; });
    double min_x = min_x_it->x, max_x = max_x_it->x;
    
    auto [min_y_it, max_y_it] = std::minmax_element(
        projected_points.begin(), projected_points.end(),
        [](const cv::Point2d& a, const cv::Point2d& b) { return a.y < b.y; });
    double min_y = min_y_it->y, max_y = max_y_it->y;
    
    // set the margin
    const double margin = 40;
    
    // filter the points in the margin
    img_points.clear();
    std::copy_if(ori_img_points.begin(), ori_img_points.end(), 
                 std::back_inserter(img_points),
                 [min_x, max_x, min_y, max_y, margin](const cv::Point2d& p) {
                     return p.x >= min_x - margin && p.x <= max_x + margin && 
                            p.y >= min_y - margin && p.y <= max_y + margin;
                 });
    
    matcher.buildKdTree(img_points);
    P2PMatchResult match_result = matcher.p2pMatch(projected_points, img_points);
    for (const auto& match : match_result) {
        img_ref_points_.emplace_back(match);
    }
    LOG(INFO) << "Processed " << img_ref_points_.size() << " image reference points.\n";
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

    // LOG(INFO) << "Before optimization: \n";
    // LOG(INFO) << "q: " << q_c_l.coeffs().transpose() << " t: " << t_c_l_.transpose() << "\n";

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

    // LOG(INFO) << "After optimization: \n";
    // LOG(INFO) << "q: " << q_c_l.coeffs().transpose() << " t: " << t_c_l_.transpose() << "\n";
}