// TODO: case of multiple curves
#include "curve_modeling.h"
#include "curve_factor.h"
#include "curve_factor_p2p.h"
#include "bSpline.hpp"
#include "gradient.hpp"

#include <fstream>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <chrono>
#include <unordered_map>

CurveModeling::CurveModeling(const std::string &yaml_file)
{
    YAML::Node yaml = YAML::LoadFile(yaml_file);

    // load camera
    loadCamera(yaml, yaml_file);

    if (yaml["line_num"]) {
        merge_ = yaml["line_num"].as<int>() > 1 ? true : false;
    }

    // load end point
    if (yaml["end_point"])
    {
        end_point_ << yaml["end_point"][0].as<double>(), yaml["end_point"][1].as<double>(), yaml["end_point"][2].as<double>();
    }
    else
    {
        LOG(ERROR) << "No end point found in yaml file.\n";
        return;
    }

    // load image
    if (yaml["image_path"])
    {
        std::string image_file = yaml["image_path"].as<std::string>();
        img_ = cv::imread(image_file, cv::IMREAD_COLOR);
    }
    else
    {
        LOG(ERROR) << "No image file found in yaml file.\n";
        return;
    }

    if (yaml["selected_points"])
    {
        std::string selected_points = yaml["selected_points"].as<std::string>();
        generateCurveImagePoints(selected_points);
    }

    // load lidar-camera extrinsic
    loadLidar2CameraExtrinsic(yaml);

    if (yaml["ex_optimization"].as<int>()) {
        ex_optimization_ = std::make_shared<ExOptimization>(R_c_l_, t_c_l_, cam_);
        ex_optimization_->loadLidarPoints(yaml["ref_lidar_points"].as<std::string>());
        ex_optimization_->loadImgPoints(yaml["ref_image_points"].as<std::string>());
        ex_optimization_->optimization();
    }

    if (yaml["matcher_type"]) {
        MatcherConfig config;
        config.type = yaml["matcher_type"].as<int>();
        matcher_ = std::make_shared<Matcher>(config);
    }
}

CurveModeling::~CurveModeling()
{
    lidar_points_.clear();
    cam_.reset();
}

void CurveModeling::loadLidarPoints(const std::string &lidar_points_file)
{

    std::fstream input(lidar_points_file, std::ios::in);
    if (!input)
    {
        LOG(ERROR) << "Cannot open file: " << lidar_points_file << "\n";
        return;
    }

    std::string line;
    while (std::getline(input, line))
    {
        std::istringstream iss(line);
        Eigen::Vector3d point;
        iss >> point(0) >> point(1) >> point(2);
        lidar_points_.push_back(point);
    }
}

void CurveModeling::loadLidarPoints(const LoadPCD &load_pcd)
{
    std::vector<Eigen::Vector3d> lidar_points = load_pcd.getPoints();
    LOG(INFO) << "Load " << lidar_points.size() << " lidar points.\n";

    if (merge_) {
        mergeLidarPoints(lidar_points);
    }
    else {
        lidar_points_ = lidar_points;
    }

    lidarP2img();
}

/// TODO: 完善合并两根线的逻辑
void CurveModeling::mergeLidarPoints(const std::vector<Eigen::Vector3d>& lidar_points) {
    if (lidar_points.empty()) {
        return;
    }
    
    double start_x = lidar_points.front().x();
    double end_x = lidar_points.back().x();

    // map: key--x, value--point
    std::unordered_map<double, std::vector<Eigen::Vector3d>> points_map;
    const double interval = 0.1;

    for (const Eigen::Vector3d& lp : lidar_points) {
        double interval_key = start_x + std::floor((lp.x() - start_x) / interval) * interval;
        // add point to map
        points_map[interval_key].emplace_back(lp);
    }

    for (const auto& [interval_key, points] : points_map) {
        Eigen::Vector3d point(0, 0, 0);
        if (points.empty()) {
            continue;
        }
        
        for (const Eigen::Vector3d& p : points) {
            point += p;
        }
        point /= points.size();
        lidar_points_.emplace_back(point);
    }

    LOG(INFO) << "Keep " << lidar_points_.size() << " lidar points after merge.\n";
}

void CurveModeling::loadCamera(const YAML::Node &yaml, const std::string &yaml_file)
{
    if (yaml["cam_calib"])
    {
        std::string camera_file;
        int pn = yaml_file.find_last_of("/");
        camera_file = yaml_file.substr(0, pn + 1) + yaml["cam_calib"].as<std::string>();
        // load camera parameters
        cam_ = std::make_shared<Camera>();
        cam_->readIntrinsicParameters(camera_file);
    }
    else
    {
        LOG(ERROR) << "No camera file found in yaml file.\n";
        return;
    }
}

void CurveModeling::loadLidar2CameraExtrinsic(const YAML::Node &yaml)
{
    if (!yaml["T_cam_lidar"])
    {
        LOG(ERROR) << "No extrinsic parameters found in yaml file.\n";
        return;
    }

    std::vector<double> vecT = yaml["T_cam_lidar"]["data"].as<std::vector<double>>();
    assert(vecT.size() == 16);

    // load extrinsic parameters
    Eigen::Matrix4d T = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(vecT.data());
    R_c_l_ = T.block<3, 3>(0, 0);
    t_c_l_ = T.block<3, 1>(0, 3);
}

void CurveModeling::lidarPreprocessing()
{
    // fit curve
    curveLidarFitting();

    // generate curve points
    generateCurvePoints();
}

[[maybe_unused]] bool CurveModeling::lineResidualTesting()
{
    auto point_error = [](const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) -> double
    {
        return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2) + pow(p1.z() - p2.z(), 2));
    };

    double sum_err = 0;
    int big_num = 0;
    std::vector<double> errors;
    for (const Eigen::Vector3d &p : lidar_points_)
    {
        Eigen::Vector3d line_point(p.x(), 0, 0);
        line_point.y() = (line_point.x() - plane_param_[0][1]) / plane_param_[0][0];
        line_point.z() = mesh_param_[0][0] * pow(line_point.x() - mesh_param_[0][1], 2) + mesh_param_[0][2];

        double error = point_error(p, line_point);
        if (error > 0.05)
        {
            big_num++;
        }
        errors.push_back(error);
    }

    double avg_err = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();

    // Eigen::Vector3d line_end_point(end_point_.x(), 0, 0);
    // line_end_point.y() = plane_param_[0][0] * line_end_point.x() + plane_param_[0][1];
    // line_end_point.z() = mesh_param_[0][0] * pow(line_end_point.x() - mesh_param_[0][1], 2) + mesh_param_[0][2];
    // double end_point_error = point_error(end_point_, line_end_point);

    // if (big_num > 20 || avg_err > 0.05 || end_point_error > 0.03)
    if (big_num > 20 || avg_err > 0.05)
    {
        LOG(WARNING) << "Average error: " << avg_err << "\n";
        LOG(WARNING) << "Big error number: " << big_num << "\n";
        // LOG(WARNING) << "End point error: " << end_point_error << "\n";
        return false;
    }

    return true;
}

[[maybe_unused]] bool CurveModeling::lineResidualTesting(const std::vector<cv::Point> &points, const Eigen::VectorXd &curve_param)
{    
    auto point_error = [](const cv::Point &p1, const cv::Point &p2) -> double
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    };

    double sum_err = 0;
    int big_error_num = 0;
    std::vector<double> errors;
    for (const cv::Point &p : points)
    {
        double y = 0;
        for (int i = 0; i <= degree_; i++)
        {
            y += curve_param(i) * pow(p.x, i);
        }

        double error = point_error(p, cv::Point(p.x, y));
        errors.push_back(error);

        if (error > 3)
        {
            big_error_num++;
        }
    }

    double avg_err = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
    LOG(WARNING) << "Average error: " << avg_err << "\n";
    LOG(WARNING) << "Big error number: " << big_error_num << "\n";

    return true;
}

void CurveModeling::curveLidarFitting()
{
    Eigen::MatrixXd A(lidar_points_.size(), 3);
    Eigen::VectorXd b(lidar_points_.size());
    for (int i = 0; i < lidar_points_.size(); ++i) {
        A(i, 0) = lidar_points_[i](0) * lidar_points_[i](0);
        A(i, 1) = lidar_points_[i](0);
        A(i, 2) = 1.0;
        b(i) = lidar_points_[i](2);
    }

    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
    mesh_param_[0][0] = x(0);
    mesh_param_[0][1] = x(1);
    mesh_param_[0][2] = x(2);

    Eigen::MatrixXd A2(lidar_points_.size(), 2);
    Eigen::VectorXd b2(lidar_points_.size());
    for (int i = 0; i < lidar_points_.size(); ++i) {
        A2(i, 0) = lidar_points_[i](1);
        A2(i, 1) = 1.0;
        b2(i) = lidar_points_[i](0);
    }
    Eigen::VectorXd x2 = A2.colPivHouseholderQr().solve(b2);
    plane_param_[0][0] = x2(0);
    plane_param_[0][1] = x2(1);

    LOG(INFO) << "a: " << mesh_param_[0][0] << " b: " << mesh_param_[0][1] << " c: " << mesh_param_[0][2];
    LOG(INFO) << "k: " << plane_param_[0][0] << " m: " << plane_param_[0][1];
    // lineResidualTesting();

#if 1
    // output points
    generateCurvePoints();
#endif
}

void CurveModeling::generateCurvePoints()
{
    ori_lidar2img_points_.clear();
    cv::Mat img = img_.clone();
    int end = end_point_.x() - 1.0;
    double sample = 0.2;
    for (double start = 20.0; start <= end; start += sample) {
        double x = start;
        x_samples_.push_back(x);
        double y = (x - plane_param_[0][1]) / plane_param_[0][0];
        double z = mesh_param_[0][0] * x * x + mesh_param_[0][1] * x + mesh_param_[0][2];

        Eigen::Vector3d p(x, y, z);
        Eigen::Vector3d p_c = R_c_l_ * p + t_c_l_;
        Eigen::Vector2d p_img;
        cam_->spaceToPlane(p_c, p_img);
        ori_lidar2img_points_.push_back(cv::Point2d(p_img(0), p_img(1)));
        cv::circle(img, cv::Point(p_img(0), p_img(1)), 3, cv::Scalar(0, 0, 255), -1);
    }
    
    cv::imwrite("curve_fitting_points.jpg", img);
    std::vector<std::pair<cv::Point2d, Gradient>> img_points_grad;
    img_points_grad = calculateGradient(ori_lidar2img_points_);
    outputPointsAndGrad("curveFunc_points_grad.txt", img_points_grad);
}

void CurveModeling::generateCurveImagePoints(const std::string &selected_points)
{
    std::fstream file(selected_points, std::ios::in);
    if (!file.is_open())
    {
        std::cerr << "Open input points file failed!" << std::endl;
        return;
    }
    std::vector<cv::Point2d> img_points;
    std::string line;

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ',')) {
            double x = std::stod(token);
            std::getline(iss, token, ',');
            double y = std::stod(token);
            img_points.emplace_back(cv::Point2d(x, y));
        }
    }
    file.close();

    // img_points_ = samplePointsBetween(img_points[0], img_points[1], 100);
    std::reverse(img_points.begin(), img_points.end());

    std::vector<cv::Point2d> temp_points;
    temp_points = calculateBSpline(img_points, 200);
    img_points_.insert(img_points_.end(), temp_points.begin(), temp_points.end());

# if 1
    cv::Mat img = img_.clone();
    for (const cv::Point2d &p : img_points_)
    {
        cv::circle(img, p, 3, cv::Scalar(0, 0, 255), -1);
    }
    cv::imwrite("curve_points.jpg", img);
#endif

    std::vector<std::pair<cv::Point2d, Gradient>> img_points_grad;
    img_points_grad = calculateGradient(img_points_);
    outputPointsAndGrad("img_points_grad.txt", img_points_grad);
}

[[maybe_unused]] void CurveModeling::curveImageDetection(bool visualize)
{
    cv::Mat img = img_.clone();
    
    std::shared_ptr<Curve> curve = std::make_shared<Curve>();
    curve->curveDetection(img);

    curve_lines_ = curve->getCurveLines();

    curve_param_ = curveImageFitting(curve_lines_[0]);

    LOG(INFO) << "Curve parameters: " << curve_param_.transpose() << "\n";

    lineResidualTesting(curve_lines_[0], curve_param_);

    if (visualize)  visualization();
}

[[maybe_unused]] Eigen::VectorXd CurveModeling::curveImageFitting(const std::vector<cv::Point> &points)
{
    int n = (int)points.size();
    Eigen::MatrixXd A(n, degree_ + 1);

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j <= degree_; j++)
        {
            A(i, j) = pow(points[i].x, j);
        }
    }

    Eigen::VectorXd y(n);
    for (int i = 0; i < n; i++)
    {
        y(i) = points[i].y;
    }

    Eigen::VectorXd result = A.householderQr().solve(y);

    return result;
}

void CurveModeling::visualization()
{
    std::ofstream output_points("output_lidar_points.txt", std::ios::out);
    cv::Mat projection2 = img_.clone();
    for (const auto& ix : x_samples_) {
        double z = mesh_param_[0][0] * ix * ix + mesh_param_[0][1] * ix + mesh_param_[0][2];
        double y = (ix - plane_param_[0][1]) / plane_param_[0][0];
        Eigen::Vector3d p(ix, y, z);
        output_points << ix << " " << y << " " << z << "\n";
        Eigen::Vector3d p_c = R_c_l_ * p + t_c_l_;
        Eigen::Vector2d p_img;
        cam_->spaceToPlane(p_c, p_img);
        cv::circle(projection2, cv::Point(p_img(0), p_img(1)), 1, cv::Scalar(0, 0, 255), -1);
    }
    cv::imwrite("projection_1.jpg", projection2);
    output_points.close();
}

void CurveModeling::project3DPointsToImage(const std::vector<Eigen::Vector3d> &points)
{
    std::vector<cv::Point2f> image_points;
    for (const Eigen::Vector3d &p : points)
    {
        Eigen::Vector3d point = R_c_l_ * p + t_c_l_;
        Eigen::Vector2d image_point;
        cam_->spaceToPlane(point, image_point);
        if (image_point.y() < 0 || image_point.y() > cam_->img_h_ || image_point.x() < 0 || image_point.x() > cam_->img_w_)
        {
            continue;
        }
        image_points.push_back(cv::Point2f(image_point.x(), image_point.y()));
    }

    // error(distance to curve equation)
    double sum_err = 0;
    for (const cv::Point2f &p : image_points)
    {
        double y = 0;
        for (int i = 0; i <= degree_; i++)
        {
            y += curve_param_(i) * pow(p.x, i);
        }

        sum_err += sqrt(pow(p.y - y, 2));
    }

    double avg_err = sum_err / image_points.size();

    LOG(INFO) << "Average error: " << avg_err << "\n";
}

void CurveModeling::optimization() {
    Matcher::MatchResult result = matcher_->match(ori_lidar2img_points_, img_points_);

    
    // Perform optimization based on the matching result
    std::visit([this](auto&& matchResult) {
        optimization3DPoints(matchResult);
    }, result);
}

void CurveModeling::optimization3DPoints(const P2LMatchResult& lines)
{
    ceres::Problem problem;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 8;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.num_threads = 8;

    for (size_t i = 0; i < lines.size(); ++i) {
        ceres::CostFunction* cost_function = CurveFactor::Create(lines[i], x_samples_[i], Trans(R_c_l_, t_c_l_), cam_);
        problem.AddResidualBlock(cost_function, nullptr, &mesh_param_[0][0], &mesh_param_[0][1], &mesh_param_[0][2], 
            &plane_param_[0][0], &plane_param_[0][1]);
    }
    // ceres::CostFunction* cost_function = CurveOptimization::Create(linep, 15.0, Tcl_, cam);
    // problem.AddResidualBlock(cost_function, nullptr, &curve.a, &curve.b, &curve.c, &curve.k, &curve.m);

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "a: " << mesh_param_[0][0] << " b: " << mesh_param_[0][1] << " c: " << mesh_param_[0][2];
    LOG(INFO) << "k: " << plane_param_[0][0] << " m: " << plane_param_[0][1];
}

void CurveModeling::optimization3DPoints(const P2PMatchResult& points)
{
#if 1 
    // visiaulization
    LOG(INFO) << "Resample points size: " << points.size() << 
        " Ori points size: " << ori_lidar2img_points_.size() << "\n";

    cv::Mat img = img_.clone();
    for (const cv::Point2d& p : ori_lidar2img_points_) {
        cv::circle(img, p, 3, cv::Scalar(0, 0, 255), -1);
    }

    for (const cv::Point2d& p : points) {
        cv::circle(img, p, 3, cv::Scalar(122, 0, 87), -1);
    }

    cv::imwrite("ori_ans_resample_points.jpg", img);
#endif
    
    ceres::Problem problem;
    ceres::Solver::Options options;
    // ceres::LossFunction *loss_function = new ceres::HuberLoss(5.0);
    ceres::LossFunction *loss_function = nullptr;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 8;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.num_threads = 8;

    for (size_t i = 0; i < points.size(); ++i) {
        ceres::CostFunction* cost_function = CurveP2PFactor::Create(points[i], x_samples_[i], Trans(R_c_l_, t_c_l_), cam_);
        problem.AddResidualBlock(cost_function, loss_function, &mesh_param_[0][0], &mesh_param_[0][1], &mesh_param_[0][2], 
            &plane_param_[0][0], &plane_param_[0][1]);
    }
    // ceres::CostFunction* cost_function = CurveOptimization::Create(linep, 15.0, Tcl_, cam);
    // problem.AddResidualBlock(cost_function, nullptr, &curve.a, &curve.b, &curve.c, &curve.k, &curve.m);

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "a: " << mesh_param_[0][0] << " b: " << mesh_param_[0][1] << " c: " << mesh_param_[0][2];
    LOG(INFO) << "k: " << plane_param_[0][0] << " m: " << plane_param_[0][1];
}

void CurveModeling::optimizationEx()
{
    cv::Mat img_1 = img_.clone();
    // projection before optimization
    for (const Eigen::Vector3d& lp : ex_optimization_->lidar_points_) {
        Eigen::Vector3d p_c = R_c_l_ * lp + t_c_l_;
        Eigen::Vector2d p_img;
        cam_->spaceToPlane(p_c, p_img);
        cv::circle(img_1, cv::Point(p_img(0), p_img(1)), 10, cv::Scalar(0, 0, 255), -1);
    }
    ex_optimization_->optimization();
    R_c_l_ = ex_optimization_->getR();
    LOG(INFO) << "\nR:\n" << R_c_l_ << "\n";
    t_c_l_ = ex_optimization_->getT();
    LOG(INFO) << "\nt:\n" << t_c_l_.transpose() << "\n";

    // projection after optimization
    for (const Eigen::Vector3d& lp : ex_optimization_->lidar_points_) {
        Eigen::Vector3d p_c = R_c_l_ * lp + t_c_l_;
        Eigen::Vector2d p_img;
        cam_->spaceToPlane(p_c, p_img);
        cv::circle(img_1, cv::Point(p_img(0), p_img(1)), 10, cv::Scalar(255, 0, 0), -1);
    }

    cv::imwrite("ex_optimization.jpg", img_1);
}

void CurveModeling::lidarP2img() {
    cv::Mat img = img_.clone();
    for (const Eigen::Vector3d& lp : lidar_points_) {
        Eigen::Vector3d p_c = R_c_l_ * lp + t_c_l_;
        Eigen::Vector2d p_img;
        cam_->spaceToPlane(p_c, p_img);
        cv::circle(img, cv::Point(p_img(0), p_img(1)), 3, cv::Scalar(0, 0, 255), -1);
    }
    cv::imwrite("lidarP2img.jpg", img);
}