// TODO: case of multiple curves
#include "curve_modeling.h"
#include "bSpline.h"
#include "evaluation.h"
#include "visualization.h"
#include <fstream>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <chrono>
#include <unordered_map>
#include <array>
#include <cassert>
#include <filesystem>

using namespace lc_core;

static int bSplineNum;
static double sample;
static std::vector<double> xySamples, xySamplesUsed;
#ifdef MY_DEBUG
static std::string temp_path = "/home/gct/LC_Modelcurve/data/temp/";
#endif
static bool dark = false;


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
    is_track = true;
    YAML::Node yaml = YAML::LoadFile(yaml_file);
    transmission_model_ = std::make_shared<Catenary>();
    matcher_ = std::make_shared<Matcher>();
    if (yaml["res_path"])
    {
        res_path = yaml["res_path"].as<std::string>();
        if (res_path.empty())
        {
            LOG(ERROR) << "No res_path. please set it!!";
            exit(EXIT_FAILURE);
        }
    }
    if (yaml["dark"])
    {
        dark = yaml["dark"].as<int>();
    }
    if(!std::filesystem::exists(std::filesystem::path(yaml["curve_point_file"].as<std::string>())))
    {
        dark = 0;
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

    if (yaml["b_spline_num"])
    {
        bSplineNum = yaml["b_spline_num"].as<int>();
    }

    if (yaml["sample"])
    {
        sample = yaml["sample"].as<double>();
        LOG(INFO) << "Sample interval: " << sample << "\n";
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
        Eigen::Vector4f min_pt(rectang_size[0]-10, rectang_size[2], rectang_size[4], 1.0f);
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

float pointToLineSegmentDistance(const Eigen::Vector3f &point, 
                                 const Eigen::Vector3f &line_start, 
                                 const Eigen::Vector3f &line_end)
{
    Eigen::Vector3f v = line_end - line_start;
    Eigen::Vector3f w = point - line_start;
    
    float c1 = w.dot(v);
    if (c1 <= 0.0) return w.norm();  // 点在起点外侧，返回点到起点的距离
    
    float c2 = v.dot(v);
    if (c2 <= c1) return (point - line_end).norm();  // 点在终点外侧，返回点到终点的距离
    
    float t = c1 / c2;
    Eigen::Vector3f projection = line_start + t * v;  // 点在直线上的投影
    return (point - projection).norm();  // 返回垂直距离
}



void CurveModeling::getCylinderCloud(std::vector<Eigen::Vector3d> &points_, 
                                     float cylinder_radius)        
{
    float extend_distance = 5;
    // std::string output_pcd_path = "/home/gct/LC_Modelcurve/data/1-13guangzhou/temp/1.pcd";
    if (!std::filesystem::exists(std::filesystem::path(raw_pcd_file)))
    {
        LOG(ERROR) << "Not the first solution and missing the original point cloud file";
        exit(EXIT_FAILURE);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 1. 读取原始中轴线起止点并延伸
    Eigen::Vector3f line_start(rectang_size[0], rectang_size[3], rectang_size[5]); 
    Eigen::Vector3f line_end(rectang_size[1], rectang_size[2], rectang_size[4]); 
    Eigen::Vector3f line_direction = (line_end - line_start).normalized();
    
    // 中轴线前后延伸
    line_end = line_end + line_direction * extend_distance;
    line_start = line_start - line_direction * extend_distance;

    // 2. 手动过滤：保留圆柱内的点（替代CropCylinder）
    for (const auto &p : raw_cloud->points)
    {
        Eigen::Vector3f point(p.x, p.y, p.z);
        // 计算点到中轴线线段的垂直距离
        float distance = pointToLineSegmentDistance(point, line_start, line_end);
        // 距离小于半径 → 保留该点
        if (distance <= cylinder_radius)
        {
            filtered_cloud->points.push_back(p);
        }
    }
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    LOG(INFO) << "Cylinder cloud size: " << filtered_cloud->size();

    // // 3. 保存PCD文件
    // if (!output_pcd_path.empty()) {
    //     if (pcl::io::savePCDFileASCII(output_pcd_path, *filtered_cloud) == -1) {
    //         LOG(ERROR) << "Failed to save filtered cylinder cloud to PCD file: " << output_pcd_path;
    //     } else {
    //         LOG(INFO) << "Filtered cylinder cloud saved to: " << output_pcd_path;
    //     }
    // }

    // 转换为Eigen格式
    points_.clear();
    points_.reserve(filtered_cloud->points.size());
    for (const auto &p : filtered_cloud->points)
    {
        points_.emplace_back(p.x, p.y, p.z);
    }
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
        //getRectangle(lidar_points);
        getCylinderCloud(lidar_points,4);
        LineExtractor line_extractor;
        line_extractor.extractTwoLinesIsolated(lidar_points,
                                              line_points,    
                                              1,
                                              3.0,   
                                              0.05,   
                                              0.08,   
                                              5000,
                                              res_path);
        if (line_points.size() < 10)
        {
            std::ofstream ofs(res_path + "result.txt", std::ios::trunc);
            ofs << "0";
            LOG(ERROR) << "0  : get filter points fail";
            ofs.close();
            sleep(5);
            exit(EXIT_FAILURE);
        }
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
    // generate curve points
    updateLidar2PixelPoints();
    int size = 0;
    std::unordered_map<int, std::array<double, 6>> Qxyz;
    std::ofstream ofs(res_path + "lidar_points_Q.txt", std::ios::trunc);
    if (!ofs.is_open())
    {
        LOG(ERROR) << "错误: 无法打开文件 " << res_path + "lidar_points_Q.txt" << " 进行写入!" << std::endl;
        return;
    }
    for (double &iy : xySamples)
    {
        Eigen::Vector3d p = transmission_model_->generateSinglePoint(iy);
        std::array<double, 6> arr;
        arr[0] = cov_t[0] + pow(p.y(), 2) * cov_t[3] + pow(p.y(), 4) * cov_t[5] + 2 * p.y() * cov_t[1] + 2 * pow(p.y(), 2) * cov_t[2] + 2 * pow(p.y(), 3) * cov_t[4];
        arr[1] = 0;
        arr[2] = arr[0] * (2 * para[3] * p.x() + para[5]);
        arr[3] = 0;
        arr[4] = 0;
        arr[5] = cov_f[0] * pow(p.x(), 4) + cov_f[3] + pow(p.x(), 2) * cov_f[5] + 2 * pow(p.x(), 2) * cov_f[1] + 2 * pow(p.x(), 3) * cov_f[2] + 2 * pow(p.x(), 1) * cov_f[4] + arr[2];
        Qxyz[size++] = arr;
        if ((int)((iy - xy_interval_start_) / sample) % 2 == 0)
        {
            ofs << std::left << std::fixed << std::setprecision(10) << std::setw(20) << p.x() << std::left << std::setw(20) << p.y() << std::left << std::setw(20) << p.z() << std::left << std::setw(20) << arr[0] << std::left << std::setw(20) << arr[1] << std::left << std::setw(20) << arr[2] << std::left << std::setw(20) << arr[3] << std::left << std::setw(20) << arr[4] << std::left << std::setw(20) << arr[5] << std::endl;
        }
    }
    ofs.close();

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
    transmission_model_->fitTransmissionModel(lidar_points_, end_point, cov_t, cov_f, para);

    // LOG(INFO) << "SIGMA_XY " << "    " << cov_t[0] << "    " << cov_t[1] << "    " << cov_t[2] << "    " << cov_t[3] << "    " << cov_t[4] << "    " << cov_t[5];
    // LOG(INFO) << "SIGMA_XZ " << "    " << cov_f[0] << "    " << cov_f[1] << "    " << cov_f[2] << "    " << cov_f[3] << "    " << cov_f[4] << "    " << cov_f[5];
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
        std::ofstream ofs(res_path + "result.txt", std::ios::trunc);
        ofs << "1";
        ofs.close();
        return;
    }
    if (first_time)
    {
        transmission_model_->set_first_time();
    }

    P2PMatchResult result = matcher_->match(ori_lidar2img_points_, img_points_);
    auto [avg_err, max_err] = calculateReprojectError(result, ori_lidar2img_points_);
    LOG(INFO) << "Original match reproject error, max: " << max_err << " , avg: " << avg_err << "\n";

#ifdef MY_DEBUG
    drawMatchResultOnImage(img_, temp_path + "match.txt", temp_path + "match_visualization.jpg");
#endif

    OptimizationInput input(xySamplesUsed, R_c_l_, t_c_l_, end_point, cam_);
    // Perform optimization based on the matching result
    transmission_model_->optimizeTransmissionModel(result, input);

    // output 3D points to txt file
    output3DPointsToTxt(res_path + "middle_output_lidar_points.txt");
    outputPCD(res_path + "middle_output_lidar_points.txt", res_path + "middle_line_points.pcd");

    // update match and re-optimization
    updateMatchAndReOptimization(input);

    bool is_visual = transmission_model_->get_is_visual();

    // output 3D points to txt file
    output3DPointsToTxt(res_path + "final_output_lidar_points.txt");
    outputPCD(res_path + "final_output_lidar_points.txt", res_path + "final_line_points.pcd");
    // save final 3D points to txt file
    savePcd2Txt(res_path + "final_line_points.pcd", res_path + "final_line_points.txt");

    if (is_visual || first_time)
    {

        std::vector<std::vector<double>> test = transmission_model_->getpara();
        int size = 0;
        std::unordered_map<int, std::array<double, 6>> Rxyz;
        std::ofstream ofs(res_path + "visual_points_R.txt", std::ios::trunc);
        for (double &iy : xySamples)
        {
            Eigen::Vector3d p = transmission_model_->generateSinglePoint(iy);
            std::array<double, 6> arr;
            arr[0] = test[0][0] + pow(p.y(), 2) * test[0][3] + pow(p.y(), 4) * test[0][5] + 2 * p.y() * test[0][1] + 2 * pow(p.y(), 2) * test[0][2] + 2 * pow(p.y(), 3) * test[0][4];
            arr[1] = 0;
            arr[2] = arr[0] * (2 * test[2][3] * p.x() + test[2][5]);
            arr[3] = 0;
            arr[4] = 0;
            arr[5] = test[1][0] * pow(p.x(), 4) + test[1][3] + pow(p.x(), 2) * test[1][5] + 2 * pow(p.x(), 2) * test[1][1] + 2 * pow(p.x(), 3) * test[1][2] + 2 * pow(p.x(), 1) * test[1][4] + arr[2];
            Rxyz[size++] = arr;
            if ((int)((iy - xy_interval_start_) / sample) % 2 == 0)
            {
                ofs << std::left << std::fixed << std::setprecision(10) << std::setw(20) << p.x() << std::left << std::setw(20) << p.y() << std::left << std::setw(20) << p.z() << std::left << std::setw(20) << arr[0] << std::left << std::setw(20) << arr[1] << std::left << std::setw(20) << arr[2] << std::left << std::setw(20) << arr[3] << std::left << std::setw(20) << arr[4] << std::left << std::setw(20) << arr[5] << std::endl;
            }
        }
        updateLidar2PixelPoints();
        result = matcher_->match(ori_lidar2img_points_, img_points_);
        std::tie(avg_err, max_err) = calculateReprojectError(result, ori_lidar2img_points_);
        LOG(INFO) << "Final match reproject error, max: " << max_err << " , avg: " << avg_err << "\n";

        if (!dark)
        {
            std::ofstream ofs(res_path + "result.txt", std::ios::trunc);
            if (is_track == false && avg_err > 50)
            {
                ofs << "0";
            }
            else if ((is_track == false && avg_err <= 50))
            {
                ofs << "2";
            }
            else
            {
                ofs << "3";
            }
            ofs.close();
        }
    }
    else
    {
        std::ofstream ofs(res_path + "result.txt", std::ios::trunc);
        ofs << "1";
        ofs.close();
    }

#ifdef MY_DEBUG
    drawMatchResultOnImage(img_, temp_path + "match.txt", temp_path + "final_match_visualization.jpg");
#endif
}

void CurveModeling::optimizationDark()
{
    transmission_model_->optimizeTransmissionModelDark(end_point);

    // output 3D points to txt file
    output3DPointsToTxt(res_path + "final_output_lidar_points.txt");
    outputPCD(res_path + "final_output_lidar_points.txt", res_path + "final_line_points.pcd");
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
        transmission_model_->optimizeTransmissionModel(points, input);
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
            cv::circle(img, cv::Point(p_img(0), p_img(1)), 1, cv::Scalar(0, 0, 255), -1);
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
    if (!pp.Is_track())
    {
        img_points_ = pp.Get_pre_point();
        is_track = false;
        LOG(INFO) << "Fail to track img";
        return;
    }
    else
    {
        pp.reInterpolate();
        img_points_ = pp.get_cur_points();
        pp.visualizeReInterpolated(img_);
        outputPoints(curve_point_file, img_points_);
        is_track = true;
        LOG(INFO) << "Finish optical_flow";
    }
}