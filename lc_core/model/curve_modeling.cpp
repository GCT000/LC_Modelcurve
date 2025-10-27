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
static std::vector<double> xySamples, xySamplesUsed;
static int y_optimize = 0;
#ifdef MY_DEBUG
static std::string temp_path = "/home/gct/LC-CurveModel/data/temp/";
#endif
static bool dark = false;

bool CurveModeling::if_no_end_point()
{
    return (end_point[0] == 0.0) && (end_point[1] == 0.0) && (end_point[2] == 0.0);
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<std::string>> CurveModeling::get_cal_distance_files()
{
    return std::make_pair(raw_cloud, cal_dis_output_files);
}

void CurveModeling::set_end_point(const Eigen::Vector4f &point)
{
    end_point[0] = point[0];
    end_point[1] = point[1];
    end_point[2] = point[2];
    LOG(INFO) << "Set end point : [" << end_point[0] << "、" << end_point[1] << "、" << end_point[2] << "]";
}

std::pair<std::vector<std::string>, Eigen::Vector4f> CurveModeling::get_files_point()
{
    return std::make_pair(pcd_files, end_point_wgs84);
}

void savePcd2Txt(const std::string &pcd_file, const std::string &txt_file)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud);
    std::ofstream out_file(txt_file, std::ios::out);
    for (const auto &point : cloud->points)
    {
        out_file << point.x << " " << point.y << " " << point.z << "\n";
    }
}

CurveModeling::CurveModeling(const std::string &yaml_file)
{
    first_time = false;
    YAML::Node yaml = YAML::LoadFile(yaml_file);
    if (yaml["dark"])
    {
        dark = yaml["dark"].as<int>();
    }
    if (yaml["end_point"])
    {
        bool is_zero = true;
        std::vector<double> end_point_vec = yaml["end_point"].as<std::vector<double>>();
        end_point << end_point_vec[0], end_point_vec[1], end_point_vec[2];
        for (const auto num : end_point_vec)
        {
            if (std::abs(num) > 1e-9)
            {
                is_zero = false;
                break;
            }
            else
            {
                continue;
            }
        }
        if (is_zero)
        {
            LOG(ERROR) << "No end_point . please input it";
            exit(EXIT_FAILURE);
        }
    }
    if (yaml["end_point_wgs84"])
    {
        std::vector<double> end_point_wgs84_tmp = yaml["end_point_wgs84"].as<std::vector<double>>();
        end_point_wgs84 << end_point_wgs84_tmp[0], end_point_wgs84_tmp[1], end_point_wgs84_tmp[2], 1;
    }

    if (yaml["source_file"])
    {
        std::string source_file = yaml["source_file"].as<std::string>();
        pcd_files.push_back(source_file);
    }
    if (yaml["target_file"])
    {
        std::string target_file = yaml["target_file"].as<std::string>();
        pcd_files.push_back(target_file);
    }

    if (yaml["raw_pcd_file"])
    {
        raw_pcd_file = yaml["raw_pcd_file"].as<std::string>();
        raw_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        LOG(INFO) << "Begin to load raw pcd file";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(raw_pcd_file, *raw_cloud) == -1)
        {
            LOG(ERROR) << "no rawpcd file cannot continue";
            exit(EXIT_FAILURE);
        }
    }
    if (yaml["matched_point_file"])
    {
        std::string matched_point_file = yaml["matched_point_file"].as<std::string>();
        cal_dis_output_files.push_back(matched_point_file);
    }
    if (yaml["tunnel_cloud_file"])
    {
        std::string tunnel_cloud_file = yaml["tunnel_cloud_file"].as<std::string>();
        cal_dis_output_files.push_back(tunnel_cloud_file);
    }
    if (yaml["point_txt_file"])
    {
        std::string point_txt_file = yaml["point_txt_file"].as<std::string>();
        cal_dis_output_files.push_back(point_txt_file);
        for (const auto str : cal_dis_output_files)
        {
            if (str.empty())
            {
                LOG(ERROR) << "There is no enough outputfiles";
                exit(EXIT_FAILURE);
            }
        }
    }
    if (yaml["ndt_paragram"])
    {
        ndt = yaml["ndt_paragram"].as<std::vector<float>>();
    }
    if (yaml["icp_paragram"])
    {
        icp = yaml["icp_paragram"].as<std::vector<float>>();
    }
    if (yaml["cal_distance_paragram"])
    {
        cal_distance_para = yaml["cal_distance_paragram"].as<std::vector<float>>();
        if (cal_distance_para.size() != 5)
        {
            LOG(ERROR) << "No enough cal_distance paragram   check it !!!";
            exit(EXIT_FAILURE);
        }
    }

    // load camera
    loadCamera(yaml, yaml_file);

    // load lidar-camera extrinsic
    loadLidar2CameraExtrinsic(yaml);

    // x sample
    if (yaml["xy_interval"])
    {
        xy_interval_start_ = yaml["xy_interval"]["start"].as<double>();
        xy_interval_end_ = yaml["xy_interval"]["end"].as<double>();
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

    if (yaml["curve_point_file"])
    {
        curve_point_file = yaml["curve_point_file"].as<std::string>();
        if (!dark)
        {
            if (std::filesystem::exists(std::filesystem::path(yaml["last_image_path"].as<std::string>())) &&
                std::filesystem::exists(std::filesystem::path(yaml["curve_point_file"].as<std::string>())))
            {
                if (std::filesystem::exists(std::filesystem::path(curve_point_file)))
                {
                    optical_flow();
                }
            }
            else if (!std::filesystem::exists(std::filesystem::path(yaml["last_image_path"].as<std::string>())) &&
                     std::filesystem::exists(std::filesystem::path(yaml["curve_point_file"].as<std::string>())))
            {
                LOG(ERROR) << "no last_image, can not calculate";
                exit(EXIT_FAILURE);
            }
            else
            {
                first_time = true;
                LOG(INFO) << "FIRST TIME TO CALCULATE";
            }
        }
        else
        {
            if (!std::filesystem::exists(std::filesystem::path(yaml["curve_point_file"].as<std::string>())))
            {
                LOG(ERROR) << "IN dark situation but no curve_point_file";
            }
            else
            {
                LOG(INFO) << "FRAEJCEDFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF";
                setPoints(curve_point_file);
            }
        }
    }

    if (yaml["selected_points"] && !std::filesystem::exists(std::filesystem::path(curve_point_file)))
    {
        std::string selected_points = yaml["selected_points"].as<std::string>();
        if (!std::filesystem::exists(std::filesystem::path(selected_points)))
        {
            LOG(ERROR) << "No curve point file or selected point file\n";
            return;
        }
        if (!dark)
        {
            generateCurveImagePoints(selected_points);
            LOG(INFO) << "Finish create curve points";
        }
    }

    if (yaml["ex_optimization"].as<int>())
    {
        LOG(INFO) << "Begin to optimize extrinsic parameters";
        ex_optimization_ = std::make_shared<ExOptimization>(R_c_l_, t_c_l_, cam_);
        ex_optimization_->loadLidarPoints(yaml["ref_lidar_points"].as<std::string>());
        ex_optimization_->loadImgPoints(yaml["ref_image_points"].as<std::string>());
        // ex_optimization_->optimization();
        optimizationEx();
    }

    if (yaml["rectang_size"])
    {
        std::vector<double> rectang_size_ = yaml["rectang_size"].as<std::vector<double>>();
        rectang_size = rectang_size_;
        bool flag = true;
        for (double i : rectang_size)
        {
            if (std::abs(i) > 1e-9)
            {
                flag = true;
                break;
            }
            flag = false;
        }
        if (first_time == false && (rectang_size.empty() || !flag))
        {
            LOG(ERROR) << "Not first time and There is no rectangle size information or size = 0";
            exit(EXIT_FAILURE);
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
        if (res_path.empty())
        {
            LOG(ERROR) << "No res_path. please set it!!";
            exit(EXIT_FAILURE);
        }
    }

    if (yaml["b_spline_num"])
    {
        bSplineNum = yaml["b_spline_num"].as<int>();
    }

    if (yaml["sample"])
    {
        sample = yaml["sample"].as<double>();
        LOG(INFO) << "Sample interval: " << sample << "\n";
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

void CurveModeling::setPoints(const std::string &file_name)
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
        img_points_.push_back(point);
    }
    LOG(INFO) << "In dark read " << img_points_.size() << " points from " << file_name;
}

Eigen::Vector2d CurveModeling::lidar2pixel(const Eigen::Vector3d &p_l)
{
    Eigen::Vector3d p_c = R_c_l_ * p_l + t_c_l_;
    Eigen::Vector2d p_img;
    cam_->spaceToPlane(p_c, p_img);
    return p_img;
}

void CurveModeling::getRectangle(std::vector<Eigen::Vector3d> &points_)
{
    if (!std::filesystem::exists(std::filesystem::path(raw_pcd_file)))
    {
        LOG(ERROR) << "Not the first solution and missing the original point cloud file";
        exit(EXIT_FAILURE);
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setInputCloud(raw_cloud);
        Eigen::Vector4f min_pt(rectang_size[0], rectang_size[2], rectang_size[4], 1.0f);
        Eigen::Vector4f max_pt(rectang_size[1], rectang_size[3], rectang_size[5], 1.0f);
        crop_box.setMin(min_pt);
        crop_box.setMax(max_pt);
        crop_box.filter(*filtered_cloud);

        LOG(INFO) << "Rectangle cloud size: " << filtered_cloud->size();

        points_.reserve(filtered_cloud->points.size());
        for (const auto &p : filtered_cloud->points)
        {
            points_.emplace_back(p.x, p.y, p.z);
        }
    }
}

void CurveModeling::getFilteredLine(std::vector<Eigen::Vector3d> &lidar_points, std::vector<Eigen::Vector3d> &line_points)
{
    for (const Eigen::Vector3d &lp : lidar_points)
    {
        bool is_true_point = false;
        Eigen::Vector2d p_img = lidar2pixel(lp);
        for (const auto &p : img_points_)
        {
            double x = p.x - p_img.x();
            double y = p.y - p_img.y();
            double dis = x * x + y * y;
            if (dis < 10)
            {
                is_true_point = true;
                break;
            }
        }
        if (is_true_point)
        {
            line_points.push_back(lp);
        }
    }
    LOG(INFO) << "After filtered line cloud size: " << line_points.size();
}

void CurveModeling::loadLidarPoints(const std::string &lidar_points_path)
{
    lc_core::LoadPCD input_pcd;
    input_pcd(lidar_points_path);
    std::vector<Eigen::Vector3d> lidar_points;
    std::vector<Eigen::Vector3d> line_points;
    if (first_time)
    {
        line_points = input_pcd.getPoints();
    }
    else
    {
        getRectangle(lidar_points);
        getFilteredLine(lidar_points, line_points);
        std::sort(line_points.begin(), line_points.end(), [](const Eigen::Vector3d &a, const Eigen::Vector3d &b)
                  { return a(0) < b(0); });

        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        double min_z = std::numeric_limits<double>::max();
        double max_z = std::numeric_limits<double>::lowest();
        for (const auto &point : line_points)
        {
            min_x = std::min(min_x, point.x());
            max_x = std::max(max_x, point.x());
            min_y = std::min(min_y, point.y());
            max_y = std::max(max_y, point.y());
            min_z = std::min(min_z, point.z());
            max_z = std::max(max_z, point.z());
        }
        LOG(INFO) << "X range: [" << min_x - 3 << ", " << max_x + 3 << "]";
        LOG(INFO) << "Y range: [" << min_y - 3 << ", " << max_y + 3 << "]";
        LOG(INFO) << "Z range: [" << min_z - 3 << ", " << max_z + 3 << "]";
    }

    LOG(INFO) << "Load " << line_points.size() << " lidar points.\n";

    lidar_points_ = std::move(line_points);

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

    // generate xySamples
    xySamples = std::vector<double>(static_cast<size_t>((xy_interval_end_ - xy_interval_start_) / sample) + 1, xy_interval_start_);
    std::generate(xySamples.begin(), xySamples.end(), [y = xy_interval_start_]() mutable
                  { return y += sample; });
    LOG(INFO) << "REACHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH";
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
    transmission_model_->fitTransmissionModel(lidar_points_, end_point);
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

    std::vector<cv::Point2d> temp_points;
    // temp_points = calculateBSpline(img_points, bSplineNum);
    temp_points = calculateCatmullRomSpline(img_points, bSplineNum);
    LOG(INFO) << "temp_points size: " << temp_points.size() << "\n";

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

    OptimizationInput input(xySamplesUsed, R_c_l_, t_c_l_, end_point, cam_);
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
    output3DPointsToTxt(res_path + "final_output_lidar_points.txt");
    outputPCD(res_path + "middle_output_lidar_points.txt", res_path + "final_line_points.pcd");
    ori_lidar2img_points_.clear();
    for (const double &y : xySamplesUsed)
    {
        Eigen::Vector3d p = transmission_model_->generateSinglePoint(y);
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
    if (!xySamplesUsed.empty())
    {
        is_first_time = false;
    }

    // generate curve points using new mesh_param_ and plane_param_
    for (const double &iy : xySamples)
    {
        Eigen::Vector3d p = transmission_model_->generateSinglePoint(iy);
        Eigen::Vector2d p_img = lidar2pixel(p);
        if (p_img(0) > 0 && p_img(0) < img_.cols && p_img(1) > 0 && p_img(1) < img_.rows)
        {
            ori_lidar2img_points_.emplace_back(p_img(0), p_img(1));

            if (is_first_time)
                xySamplesUsed.push_back(iy);
        }
    }
}

void CurveModeling::output3DPointsToTxt(const std::string &filename)
{
    std::ofstream output_points(filename, std::ios::out);
    if (!output_points.is_open())
    {
        LOG(ERROR) << "错误: 无法打开文件 " << filename << " 进行写入!" << std::endl;
        return;
    }
    size_t pointCount = 0;
    for (const double &y : xySamples)
    {
        Eigen::Vector3d p = transmission_model_->generateSinglePoint(y);
        output_points << p(0) << " " << p(1) << " " << p(2) << "\n";
        pointCount++;
    }
    LOG(INFO) << "3D点输出完成! 共输出 " << pointCount << " 个点到文件: " << filename << std::endl;
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
    pp.visualizeReInterpolated(img_);

    outputPoints(curve_point_file, img_points_);

    LOG(INFO) << "Finish optical_flow";
}