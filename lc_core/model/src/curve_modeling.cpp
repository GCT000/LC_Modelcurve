// TODO: case of multiple curves
#include "curve_modeling.h"
#include "curve_factor.h"
#include "curve_factor_p2p.h"
#include "bSpline.h"

#include <fstream>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <chrono>
#include <unordered_map>

using namespace lc_core;

static int bSplineNum;
static double sample;
static std::vector<double> xSamples;
static std::string temp_path = "/home/zyp/Lidar/LC-CurveModel/temp/";

CurveModeling::CurveModeling(const std::string &yaml_file)
{
    YAML::Node yaml = YAML::LoadFile(yaml_file);

    // load camera
    loadCamera(yaml, yaml_file);

    // load end point
    if (yaml["x_interval"])
    {
        x_interval_start_ = yaml["x_interval"]["start"].as<double>();
        x_interval_end_ = yaml["x_interval"]["end"].as<double>();
    }

    // load image
    if (yaml["image_path"])
    {
        std::string image_file = yaml["image_path"].as<std::string>();
        // img_ = cv::imread(image_file, cv::IMREAD_COLOR);
        cv::Mat img = cv::imread(image_file, cv::IMREAD_COLOR);
        cam_->undistortImg(img, img_);
    }
    else
    {
        LOG(ERROR) << "No image file found in yaml file.\n";
        return;
    }

    if (yaml["b_spline_num"]) {
        bSplineNum = yaml["b_spline_num"].as<int>();
    }

    if (yaml["sample"]) {
        sample = yaml["sample"].as<double>();
        LOG(INFO) << "sample interval: " << sample << "\n";
    }

    if (yaml["selected_points"])
    {
        std::string selected_points = yaml["selected_points"].as<std::string>();
        generateCurveImagePoints(selected_points);
    }

    if (yaml["line_points"])
    {
        cv::Point2d start = cv::Point2d(yaml["line_points"]["start"][0].as<double>(), yaml["line_points"]["start"][1].as<double>());
        cv::Point2d end = cv::Point2d(yaml["line_points"]["end"][0].as<double>(), yaml["line_points"]["end"][1].as<double>());
        // generate line points
        generateLineImagePoints(start, end);
    }

    // load lidar-camera extrinsic
    loadLidar2CameraExtrinsic(yaml);

    if (yaml["ex_optimization"].as<int>()) {
        ex_optimization_ = std::make_shared<ExOptimization>(R_c_l_, t_c_l_, cam_);
        ex_optimization_->loadLidarPoints(yaml["ref_lidar_points"].as<std::string>());
        ex_optimization_->loadImgPoints(yaml["ref_image_points"].as<std::string>());
        ex_optimization_->optimization();
        optimizationEx();
    }

    if (yaml["matcher_type"]) {
        MatcherConfig config;
        config.type = yaml["matcher_type"].as<int>();
        matcher_ = std::make_shared<Matcher>(config);
    }

    if (yaml["transmission_model"]) {
        int transmission_model_type = yaml["transmission_model"].as<int>();
        if (transmission_model_type == 0) {
            transmission_model_ = std::make_shared<Parabola>();
        }
        else {
            transmission_model_ = std::make_shared<Catenary>();
        }
    }
}

CurveModeling::~CurveModeling()
{
    lidar_points_.clear();
    img_points_.clear();
    ori_lidar2img_points_.clear();
    cam_.reset();
}

Eigen::Vector2d CurveModeling::lidar2pixel(const Eigen::Vector3d& p_l) {
    Eigen::Vector3d p_c = R_c_l_ * p_l + t_c_l_;
    Eigen::Vector2d p_img;
    cam_->spaceToPlane(p_c, p_img);
    return p_img;
}

void CurveModeling::loadLidarPoints(const LoadPCD &load_pcd)
{
    std::vector<Eigen::Vector3d> lidar_points = load_pcd.getPoints();
    std::sort(lidar_points.begin(), lidar_points.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return a(0) < b(0);
    });
    
    LOG(INFO) << "Load " << lidar_points.size() << " lidar points.\n";

    lidar_points_ = std::move(lidar_points);

    lidarP2img();
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

void CurveModeling::curveLidarFitting()
{
    // fit transmission model
    transmission_model_->fitTransmissionModel(lidar_points_);
    LOG(INFO) << "Lidar points size after fitting: " << lidar_points_.size() << "\n";
}

void CurveModeling::generateCurvePoints()
{
    ori_lidar2img_points_.clear();
    cv::Mat img = img_.clone();
    for (double start = x_interval_start_; start <= x_interval_end_; start += sample) {
        xSamples.push_back(start);
        Eigen::Vector3d p = transmission_model_->generateSinglePoint(start);
        Eigen::Vector2d p_img = lidar2pixel(p);
        // cam_->undistortPoints(cv::Point2d(p_img(0), p_img(1)), undistorted_p_img);
        ori_lidar2img_points_.push_back(cv::Point2d(p_img(0), p_img(1)));
        cv::circle(img, cv::Point(p_img(0), p_img(1)), 1, cv::Scalar(0, 0, 255), -1);
    }
    cv::imwrite(temp_path + "curve_fitting_points.jpg", img);
}

void CurveModeling::generateCurveImagePoints(const std::string &selected_points)
{
    std::fstream file(selected_points, std::ios::in);
    if (!file.is_open())
    {
        LOG(ERROR) << "Open input points file failed!";
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
    temp_points = calculateBSpline(img_points, bSplineNum);
    // cam_->undistortPoints(temp_points, temp_un_points);

    // re-interpolate points
    std::vector<cv::Point2d> interpolated_points;
    for (size_t i = 0; i < temp_points.size() - 1; ++i) {
        const cv::Point2d& p1 = temp_points[i];
        const cv::Point2d& p2 = temp_points[i + 1];
        
        double distance = cv::norm(p2 - p1);
        int num_points = std::ceil(distance / 0.5);
        
        for (int j = 0; j < num_points; ++j) {
            double t = static_cast<double>(j) / num_points;
            cv::Point2d new_point = p1 + t * (p2 - p1);
            if (new_point.y >= img_points[0].y) {
                interpolated_points.push_back(new_point);
            }
        }
    }
    // add the last point
    interpolated_points.push_back(temp_points.back());
    
    img_points_.clear();
    img_points_ = std::move(interpolated_points);

# if 1
    cv::Mat img = img_.clone();
    LOG(INFO) << "Generate " << img_points_.size() << " curve points on image.\n";
    for (const cv::Point2d &p : img_points_)
    {
        cv::circle(img, p, 1, cv::Scalar(0, 0, 255), -1);
    }
    cv::imwrite(temp_path + "curve_points.jpg", img);
    outputPoints(temp_path + "curve_points.txt", img_points_);
#endif
}

void CurveModeling::generateLineImagePoints(const cv::Point2d &start, const cv::Point2d &end)
{
    std::vector<cv::Point2d> linePoints;
    linePoints.reserve(bSplineNum);

    for (int i = 0; i < bSplineNum; ++i) {
        double t = static_cast<double>(i) / (bSplineNum - 1);
        double x = start.x + t * (end.x - start.x);
        double y = start.y + t * (end.y - start.y);
        linePoints.emplace_back(x, y);
    }

    // set img_points_
    img_points_ = std::move(linePoints);

    // visualization
    cv::Mat img = img_.clone();
    for (const auto& point : img_points_) {
        cv::circle(img, point, 2, cv::Scalar(0, 255, 0), -1);
    }
    cv::imwrite(temp_path + "line_points.jpg", img);

    LOG(INFO) << "Generate " << img_points_.size() << " line points on image.\n";
}

void CurveModeling::visualization()
{
    std::ofstream output_points(temp_path + "output_lidar_points.txt", std::ios::out);
    cv::Mat projection2 = img_.clone();
    for (double ix = x_interval_start_; ix <= x_interval_end_; ix += sample) {
        Eigen::Vector3d p = transmission_model_->generateSinglePoint(ix);
        output_points << p(0) << " " << p(1) << " " << p(2) << "\n";
        Eigen::Vector2d p_img = lidar2pixel(p);
        // cam_->undistortPoints(cv::Point2d(p_img(0), p_img(1)), p_img);
        cv::circle(projection2, cv::Point(p_img(0), p_img(1)), 1, cv::Scalar(0, 0, 255), -1);
    }
    cv::imwrite(temp_path + "projection_1.jpg", projection2);
    output_points.close();
}

void CurveModeling::optimization() {
    Matcher::MatchResult result = matcher_->match(ori_lidar2img_points_, img_points_);

#if 1
    std::fstream output_points(temp_path + "match.txt", std::ios::in);
    cv::Mat match_img = img_.clone();
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
    cv::imwrite(temp_path + "match_visualization.jpg", match_img);
    output_points.close();
#endif

    OptimizationInput input(xSamples, R_c_l_, t_c_l_, cam_);
    // Perform optimization based on the matching result
    std::visit([this, &input](auto&& matchResult) {
        transmission_model_->optimizeTransmissionModel(matchResult, input);
    }, result);

    // update match and re-optimization
    updateMatchAndReOptimization(input);
}

void CurveModeling::updateMatchAndReOptimization(const OptimizationInput& input) {
    ori_lidar2img_points_.clear();

    // generate curve points using new mesh_param_ and plane_param_
    for (const double& ix : xSamples) {
        Eigen::Vector3d p = transmission_model_->generateSinglePoint(ix);
        Eigen::Vector2d p_img = lidar2pixel(p);
        ori_lidar2img_points_.emplace_back(p_img(0), p_img(1));
    }

    // update match
    P2PMatchResult points = matcher_->p2pMatch(ori_lidar2img_points_, img_points_);
#if 1
    std::fstream output_points(temp_path + "match.txt", std::ios::in);
    cv::Mat match_img = img_.clone();
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
    cv::imwrite(temp_path + "update_match_visualization.jpg", match_img);
    output_points.close();  
#endif
    // re-optimization
    transmission_model_->optimizeTransmissionModel(points, input, 2);
}

void CurveModeling::optimizationEx()
{
    cv::Mat img_1 = img_.clone();
    // projection before optimization
    for (const Eigen::Vector3d& lp : ex_optimization_->lidar_points_) {
        Eigen::Vector2d p_img = lidar2pixel(lp);
        cv::circle(img_1, cv::Point(p_img(0), p_img(1)), 10, cv::Scalar(0, 0, 255), -1);
    }
    ex_optimization_->optimization();
    R_c_l_ = ex_optimization_->getR();
    LOG(INFO) << "\nR:\n" << R_c_l_ << "\n";
    t_c_l_ = ex_optimization_->getT();
    LOG(INFO) << "\nt:\n" << t_c_l_.transpose() << "\n";

    // projection after optimization
    for (const Eigen::Vector3d& lp : ex_optimization_->lidar_points_) {
        Eigen::Vector2d p_img = lidar2pixel(lp);
        cv::circle(img_1, cv::Point(p_img(0), p_img(1)), 10, cv::Scalar(255, 0, 0), -1);
    }

    cv::imwrite(temp_path + "ex_optimization.jpg", img_1);
}

void CurveModeling::lidarP2img() {
    cv::Mat img = img_.clone();
    for (const Eigen::Vector3d& lp : lidar_points_) {
        Eigen::Vector2d p_img = lidar2pixel(lp);
        cv::circle(img, cv::Point(p_img(0), p_img(1)), 3, cv::Scalar(0, 0, 255), -1);
    }
    cv::imwrite(temp_path + "lidarP2img.jpg", img);
}
