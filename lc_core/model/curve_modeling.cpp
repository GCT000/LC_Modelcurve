// TODO: case of multiple curves
#include "curve_modeling.h"
#include "curve_factor.h"
#include "curve_factor_p2p.h"
#include "bSpline.h"
#include "evaluation.h"
#include "visualization.h"
#include <fstream>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <chrono>
#include <unordered_map>
#include <cassert>
#include <filesystem>

using namespace lc_core;

static int bSplineNum;
static double sample;
static std::vector<double> xSamples, xSamplesUsed;
static int y_optimize = 0;
#ifdef MY_DEBUG
static std::string temp_path = "/home/zyp/Lidar/LC-CurveModel/temp/";
#endif
static std::string res_path;
static bool dark = false;
Eigen::Vector3d end_point;

void savePcd2Txt(const std::string &pcd_file, const std::string &txt_file) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud);
    std::ofstream out_file(txt_file, std::ios::out);
    for (const auto &point : cloud->points) {
        out_file << point.x << " " << point.y << " " << point.z << "\n";
    }
}

CurveModeling::CurveModeling(const std::string &yaml_file)
{
    YAML::Node yaml = YAML::LoadFile(yaml_file);
    if (yaml["dark"]) {

        dark = yaml["dark"].as<int>();
    }
    std::vector<double> end_point_vec = yaml["end_point"].as<std::vector<double>>();
    end_point << end_point_vec[0], end_point_vec[1], end_point_vec[2];

    // load camera
    loadCamera(yaml, yaml_file);

    // load lidar-camera extrinsic
    loadLidar2CameraExtrinsic(yaml);

    // x sample
    if (yaml["x_interval"])
    {
        x_interval_start_ = yaml["x_interval"]["start"].as<double>();
        x_interval_end_ = yaml["x_interval"]["end"].as<double>();
    }

    // load image
    if (yaml["image_path"])
    {
        std::string image_file = yaml["image_path"].as<std::string>();
        cv::Mat img = cv::imread(image_file, cv::IMREAD_COLOR);

        cam_->undistortImg(img, img_);
    }
    else
    {
        LOG(ERROR) << "No image file found in yaml file.\n";
        return;
    }

    if (yaml["last_image_path"] && !dark)
    {
        if (std::filesystem::exists(std::filesystem::path(yaml["last_image_path"].as<std::string>())))
        {
            std::string last_image_file = yaml["last_image_path"].as<std::string>();
            cv::Mat img = cv::imread(last_image_file, cv::IMREAD_COLOR);

            cam_->undistortImg(img, last_img_);
        }
    }

    if (yaml["lidar_points_path"])
    {
        std::string lidar_points_path = yaml["lidar_points_path"].as<std::string>();
        loadLidarPoints(lidar_points_path);
    }

    if (yaml["res_path"])
    {
        res_path = yaml["res_path"].as<std::string>();
    }

    if (yaml["b_spline_num"])
    {
        bSplineNum = yaml["b_spline_num"].as<int>();
    }

    if (yaml["sample"])
    {
        sample = yaml["sample"].as<double>();
        LOG(INFO) << "sample interval: " << sample << "\n";
    }

    if (yaml["curve_point_file"] && !dark)
    {
        curve_point_file = yaml["curve_point_file"].as<std::string>();
        if (std::filesystem::exists(std::filesystem::path(curve_point_file)))
        {
            optical_flow();
        }
    }
    
    if (yaml["selected_points"] && !std::filesystem::exists(std::filesystem::path(curve_point_file)))
    {
        std::string selected_points = yaml["selected_points"].as<std::string>();
        if (!std::filesystem::exists(std::filesystem::path(selected_points)))
        {
            LOG(ERROR) << "no curve point file or selected point file\n";
            return;
        }
        if (!dark) {
            generateCurveImagePoints(selected_points);
        }
    }

    if (yaml["ex_optimization"].as<int>())
    {
        ex_optimization_ = std::make_shared<ExOptimization>(R_c_l_, t_c_l_, cam_);
        ex_optimization_->loadLidarPoints(yaml["ref_lidar_points"].as<std::string>());
        ex_optimization_->loadImgPoints(yaml["ref_image_points"].as<std::string>());
        // ex_optimization_->optimization();
        optimizationEx();
    }

    if (yaml["matcher_type"])
    {
        MatcherConfig config;
        config.type = yaml["matcher_type"].as<int>();
        matcher_ = std::make_shared<Matcher>(config);
    }

    if (yaml["transmission_model"])
    {
        int transmission_model_type = yaml["transmission_model"].as<int>();
        if (transmission_model_type == 0)
        {
            transmission_model_ = std::make_shared<Parabola>();
        }
        else
        {
            transmission_model_ = std::make_shared<Catenary>();
        }
    }

    if (yaml["y_optimize"])
    {
        y_optimize = yaml["y_optimize"].as<int>();
    }
}

CurveModeling::~CurveModeling()
{
    lidar_points_.clear();
    img_points_.clear();
    ori_lidar2img_points_.clear();
    cam_.reset();
}

Eigen::Vector2d CurveModeling::lidar2pixel(const Eigen::Vector3d &p_l)
{
    Eigen::Vector3d p_c = R_c_l_ * p_l + t_c_l_;
    Eigen::Vector2d p_img;
    cam_->spaceToPlane(p_c, p_img);
    return p_img;
}

void CurveModeling::loadLidarPoints(const std::string &lidar_points_path)
{
    lc_core::LoadPCD input_pcd;
    input_pcd(lidar_points_path);
    std::vector<Eigen::Vector3d> lidar_points;
    lidar_points = input_pcd.getPoints();
    std::sort(lidar_points.begin(), lidar_points.end(), [](const Eigen::Vector3d &a, const Eigen::Vector3d &b)
              { return a(0) < b(0); });

    LOG(INFO) << "Load " << lidar_points.size() << " lidar points.\n";

    lidar_points_ = std::move(lidar_points);

#ifdef MY_DEBUG
    lidarP2img();
#endif
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

    // generate xSamples
    xSamples = std::vector<double>(static_cast<size_t>((x_interval_end_ - x_interval_start_) / sample) + 1, x_interval_start_);
    std::generate(xSamples.begin(), xSamples.end(), [x = x_interval_start_]() mutable
                  { return x += sample; });

    // generate curve points
    updateLidar2PixelPoints();
    // output 3D points to txt file
    output3DPointsToTxt(res_path + "original_output_lidar_points.txt");
    outputPCD(res_path + "original_output_lidar_points.txt", res_path + "ori_line_points.pcd");

#ifdef MY_DEBUG
    drawPointsOnImage(img_, ori_lidar2img_points_, temp_path + "curve_fitting_points.jpg");
#endif
}

void CurveModeling::curveLidarFitting()
{
    // fit transmission model
    transmission_model_->fitTransmissionModel(lidar_points_);
    LOG(INFO) << "Lidar points size after fitting: " << lidar_points_.size() << "\n";
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
        while (std::getline(iss, token, ','))
        {
            double x = std::stod(token);
            std::getline(iss, token, ',');
            double y = std::stod(token);
            img_points.emplace_back(cv::Point2d(x, y));
        }
    }
    file.close();

    // img_points_ = samplePointsBetween(img_points[0], img_points[1], 100);
    // std::reverse(img_points.begin(), img_points.end());

    std::vector<cv::Point2d> temp_points;
    // temp_points = calculateBSpline(img_points, bSplineNum);
    temp_points = calculateCatmullRomSpline(img_points, bSplineNum);
    LOG(INFO) << "temp_points size: " << temp_points.size() << "\n";
    // cam_->undistortPoints(temp_points, temp_un_points);

    img_points_.clear();
    img_points_ = std::move(temp_points);

#ifdef MY_DEBUG
    LOG(INFO) << "Generate " << img_points_.size() << " curve points on image.\n";
    drawPointsOnImage(img_, img_points_, temp_path + "curve_points.jpg");
#endif
    outputPoints(curve_point_file, img_points_);
}

void CurveModeling::generateLineImagePoints(const std::string &selected_points)
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
        while (std::getline(iss, token, ','))
        {
            double x = std::stod(token);
            std::getline(iss, token, ',');
            double y = std::stod(token);
            img_points.emplace_back(cv::Point2d(x, y));
        }
    }
    file.close();

    // sample
    std::vector<cv::Point2d> sampled_points;
    if (!img_points.empty())
    {
        std::sort(img_points.begin(), img_points.end(),
                  [](const cv::Point2d &a, const cv::Point2d &b)
                  { return a.x < b.x; });

        double x_min = img_points.front().x;
        double x_max = img_points.back().x;

        // linear interpolation
        for (double x = x_min; x <= x_max; x += 1.0)
        { // 1 pixel interval
            // find the nearest two original points for interpolation
            auto it = std::lower_bound(img_points.begin(), img_points.end(), x,
                                       [](const cv::Point2d &p, double val)
                                       { return p.x < val; });

            if (it != img_points.begin() && it != img_points.end())
            {
                auto p2 = *it;
                auto p1 = *(--it);

                // linear interpolation
                double ratio = (x - p1.x) / (p2.x - p1.x);
                double y = p1.y + ratio * (p2.y - p1.y);

                sampled_points.emplace_back(x, y);
            }
        }
    }

    img_points_.clear();
    img_points_ = std::move(sampled_points);

#ifdef MY_DEBUG
    LOG(INFO) << "Generate " << img_points_.size() << " line points on image.\n";
    drawPointsOnImage(img_, img_points_, temp_path + "line_points.jpg");
    outputPoints(temp_path + "line_points.txt", img_points_);
#endif
}

void CurveModeling::visualization()
{
    updateLidar2PixelPoints();
    drawPointsOnImage(img_, ori_lidar2img_points_, res_path + "projection_1.jpg");
}

void CurveModeling::optimization()
{
    if (dark)
    {
        // optimize dark
        optimizationDark();
        return;
    }

    Matcher::MatchResult result = matcher_->match(ori_lidar2img_points_, img_points_);
    auto [avg_err, max_err] = calculateReprojectError(result, ori_lidar2img_points_);
    LOG(INFO) << "Original match reproject error, max: " << max_err << " , avg: " << avg_err << "\n";

#ifdef MY_DEBUG
    drawMatchResultOnImage(img_, temp_path + "match.txt", temp_path + "match_visualization.jpg");
#endif

    OptimizationInput input(xSamplesUsed, R_c_l_, t_c_l_, end_point, cam_);
    // Perform optimization based on the matching result
    std::visit([this, &input](auto &&matchResult)
               { transmission_model_->optimizeTransmissionModel(matchResult, input, y_optimize); }, result);

    // output 3D points to txt file
    output3DPointsToTxt(res_path + "middle_output_lidar_points.txt");
    outputPCD(res_path + "middle_output_lidar_points.txt", res_path + "middle_line_points.pcd");

    // update match and re-optimization
    updateMatchAndReOptimization(input);

    // output 3D points to txt file
    output3DPointsToTxt(res_path + "final_output_lidar_points.txt");
    outputPCD(res_path + "final_output_lidar_points.txt", res_path + "final_line_points.pcd");

    // save final 3D points to txt file
    savePcd2Txt(res_path + "final_line_points.pcd", res_path + "final_line_points.txt");

    updateLidar2PixelPoints();
    result = matcher_->match(ori_lidar2img_points_, img_points_);
    std::tie(avg_err, max_err) = calculateReprojectError(result, ori_lidar2img_points_);
    LOG(INFO) << "Final match reproject error, max: " << max_err << " , avg: " << avg_err << "\n";

#ifdef MY_DEBUG
    drawMatchResultOnImage(img_, temp_path + "match.txt", temp_path + "final_match_visualization.jpg");
#endif
}

void CurveModeling::optimizationDark()
{
    transmission_model_->optimizeTransmissionModelDark(end_point);

    // output 3D points to txt file
    output3DPointsToTxt(res_path + "dark_middle_lidar_points.txt");
    outputPCD(res_path + "dark_middle_lidar_points.txt", res_path + "dark_middle_line_points.pcd");
    ori_lidar2img_points_.clear();
    for (const double &x : xSamplesUsed)
    {
        Eigen::Vector3d p = transmission_model_->generateSinglePoint(x);
        Eigen::Vector2d p_img = lidar2pixel(p);
        ori_lidar2img_points_.emplace_back(p_img(0), p_img(1));
    }
}

void CurveModeling::updateMatchAndReOptimization(const OptimizationInput &input)
{
    updateLidar2PixelPoints();

    // update match
    P2PMatchResult points = matcher_->p2pMatch(ori_lidar2img_points_, img_points_);
    auto [avg_err, max_err] = calculateReprojectError(points, ori_lidar2img_points_);
    LOG(INFO) << "Update match reproject error, max: " << max_err << " , avg: " << avg_err << "\n";

#ifdef MY_DEBUG
    drawMatchResultOnImage(img_, temp_path + "match.txt", temp_path + "update_match_visualization.jpg");
#endif
    // re-optimization
    if (avg_err > 4.0)
    {
        transmission_model_->optimizeTransmissionModel(points, input, y_optimize, 2);
    }
}

void CurveModeling::optimizationEx()
{
    cv::Mat img_1 = img_.clone();
    // projection before optimization
    for (const Eigen::Vector3d &lp : ex_optimization_->lidar_points_)
    {
        Eigen::Vector2d p_img = lidar2pixel(lp);
        cv::circle(img_1, cv::Point(p_img(0), p_img(1)), 10, cv::Scalar(0, 0, 255), -1);
    }
    ex_optimization_->optimization();
    R_c_l_ = ex_optimization_->getR();
    LOG(INFO) << "\nR:\n"
              << R_c_l_ << "\n";
    t_c_l_ = ex_optimization_->getT();
    LOG(INFO) << "\nt:\n"
              << t_c_l_.transpose() << "\n";

    // projection after optimization
    for (const Eigen::Vector3d &lp : ex_optimization_->lidar_points_)
    {
        Eigen::Vector2d p_img = lidar2pixel(lp);
        cv::circle(img_1, cv::Point(p_img(0), p_img(1)), 10, cv::Scalar(255, 0, 0), -1);
    }
#ifdef MY_DEBUG
    cv::imwrite(temp_path + "ex_optimization.jpg", img_1);
#endif
}

void CurveModeling::lidarP2img()
{
    cv::Mat img = img_.clone();
    for (const Eigen::Vector3d &lp : lidar_points_)
    {
        Eigen::Vector2d p_img = lidar2pixel(lp);
        if (p_img(0) > 0 && p_img(0) < img_.cols && p_img(1) > 0 && p_img(1) < img_.rows)
        {
            cv::circle(img, cv::Point(p_img(0), p_img(1)), 3, cv::Scalar(0, 0, 255), -1);
        }
    }
    cv::imwrite(temp_path + "lidarP2img.jpg", img);
}

void CurveModeling::updateLidar2PixelPoints()
{
    ori_lidar2img_points_.clear();
    bool is_first_time = true;
    if (!xSamplesUsed.empty())
    {
        is_first_time = false;
    }

    // generate curve points using new mesh_param_ and plane_param_
    for (const double &ix : xSamples)
    {
        Eigen::Vector3d p = transmission_model_->generateSinglePoint(ix);
        Eigen::Vector2d p_img = lidar2pixel(p);
        if (p_img(0) > 0 && p_img(0) < img_.cols && p_img(1) > 0 && p_img(1) < img_.rows)
        {
            ori_lidar2img_points_.emplace_back(p_img(0), p_img(1));
            if (is_first_time)
                xSamplesUsed.push_back(ix);
        }
    }
}

void CurveModeling::output3DPointsToTxt(const std::string &filename)
{
    std::ofstream output_points(filename, std::ios::out);
    for (const double &x : xSamples)
    {
        Eigen::Vector3d p = transmission_model_->generateSinglePoint(x);
        output_points << p(0) << " " << p(1) << " " << p(2) << "\n";
    }
    output_points.close();
}

void CurveModeling::optical_flow()
{
    lc_preprocess::ImgPreProcess pp;
    pp.setPoints(curve_point_file);

    pp.track(last_img_, img_);
    // pp.visualizeTracking(img_);
    pp.reInterpolate();

    img_points_ = pp.get_cur_points();
    // pp.visualizeReInterpolated(img_);

    outputPoints(curve_point_file, img_points_);
}