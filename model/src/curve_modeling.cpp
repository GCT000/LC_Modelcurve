// TODO: case of multiple curves
#include "curve_modeling.h"
#include "curve_factor.h"
#include "bSpline.hpp"

#include <fstream>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <chrono>

CurveModeling::CurveModeling(const std::string &yaml_file)
{
    YAML::Node yaml = YAML::LoadFile(yaml_file);

    // load lidar measurement
    std::string lidar_points_file = yaml["lidar_points_file"].as<std::string>();
    loadLidarPoints(lidar_points_file);

    // load camera
    loadCamera(yaml, yaml_file);

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

    // load image curve degree
    degree_ = yaml["curve_degree"].as<int>();

    // load lidar-camera extrinsic
    loadLidar2CameraExtrinsic(yaml);
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
    std::vector<Eigen::Vector3d> points = load_pcd.getPoints();
    lidar_points_.clear();
    lidar_points_.insert(lidar_points_.end(), points.begin(), points.end());
    LOG(INFO) << "Load " << lidar_points_.size() << " lidar points.\n";
}

void CurveModeling::loadCamera(const YAML::Node &yaml, const std::string &yaml_file)
{
    if (yaml["cam_calib"])
    {
        std::string camera_file;
        int pn = yaml_file.find_last_of("/");
        camera_file = yaml_file.substr(0, pn + 1) + yaml["cam_calib"].as<std::string>();
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

bool CurveModeling::lineResidualTesting()
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

bool CurveModeling::lineResidualTesting(const std::vector<cv::Point> &points, const Eigen::VectorXd &curve_param)
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
}

void CurveModeling::generateCurvePoints()
{
    std::sort(lidar_points_.begin(), lidar_points_.end(), [](const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) -> bool
              { return p1.x() < p2.x(); });

    int lidar_points_num = (int)lidar_points_.size();
    int num = lidar_points_num * (end_point_.x() - lidar_points_[lidar_points_num - 1].x())
            / (lidar_points_[lidar_points_num - 1].x() - lidar_points_[0].x());

    double step = (end_point_.x() - lidar_points_[lidar_points_num - 1].x()) / num;

    curve_points_.clear();
    for (int i = 1; i <= num; i++)
    {
        Eigen::Vector3d point;
        point.x() = lidar_points_[lidar_points_num - 1].x() + i * step;
        point.y() = plane_param_[0][0] * point.x() + plane_param_[0][1];
        point.z() = mesh_param_[0][0] * pow(point.x() - mesh_param_[0][1], 2) + mesh_param_[0][2];
        curve_points_.push_back(point);
    }

    outputPoints("curve_points.txt", curve_points_);
}

void CurveModeling::generateCurveImagePoints(const std::string &selected_points)
{
    std::fstream file(selected_points, std::ios::in);
    if (!file.is_open())
    {
        std::cerr << "Open input points file failed!" << std::endl;
        return;
    }
    std::vector<cv::Point> img_points;
    std::string line;

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ',')) {
            double x = 2 * std::stod(token);
            std::getline(iss, token, ',');
            double y = 2 * std::stod(token);
            img_points.emplace_back(cv::Point2d(x, y));
        }
    }
    file.close();

    img_points_ = samplePointsBetween(img_points[0], img_points[1], 100);

    std::vector<cv::Point> temp_points;
    temp_points = calculateBSpline(img_points, 200);
    img_points_.insert(img_points_.end(), temp_points.begin(), temp_points.end());
    LOG(INFO) << "Generate " << img_points_.size() << " curve points on image.\n";
}

void CurveModeling::curveImageDetection(bool visualize)
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

Eigen::VectorXd CurveModeling::curveImageFitting(const std::vector<cv::Point> &points)
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
    cv::Mat projection2 = img_.clone();
    for (double ix = 1.0; ix <= 40.0; ix += 0.1) {
        double z = mesh_param_[0][0] * ix * ix + mesh_param_[0][1] * ix + mesh_param_[0][2];
        double y = (ix - plane_param_[0][1]) / plane_param_[0][0];
        Eigen::Vector3d p(ix, y, z);
        Eigen::Vector3d p_c = R_c_l_ * p + t_c_l_;
        Eigen::Vector2d p_img;
        cam_->spaceToPlane(p_c, p_img);
        cv::circle(projection2, cv::Point(p_img(0), p_img(1)), 3, cv::Scalar(0, 0, 255), -1);
    }
    cv::imwrite("projection_1.jpg", projection2);
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

std::pair<double, double> CurveModeling::findClosestPoints(const Eigen::Vector3d &lidarPoint, const std::vector<cv::Point> &img_points)
{
    Trans Tcl(R_c_l_, t_c_l_);
    Eigen::Vector3d p_c = Tcl.R * lidarPoint + Tcl.t;
    Eigen::Vector2d p_img;
    cam_->spaceToPlane(p_c, p_img);

    auto dis = [](const Eigen::Vector2d& p1, const cv::Point& p2) {
        return std::sqrt((p1.x() - p2.x) * (p1.x() - p2.x) + (p1.y() - p2.y) * (p1.y() - p2.y));
    };

    std::vector<std::pair<double, Eigen::Vector2d>> distances;
    for (auto& p : img_points) {
        double distance = dis(p_img, p);
        // LOG(INFO) << "img: " << p_img.x() << " " << p_img.y() << ", dis: " << distance;
        distances.push_back(std::make_pair(distance, Eigen::Vector2d(p.x, p.y)));
    }

    std::sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });

    std::vector<Eigen::Vector2d> closestPoints;
    for (size_t i = 0; i < 5 && i < distances.size(); ++i) {
        closestPoints.push_back(distances[i].second);
    }

    Eigen::MatrixXd A2(closestPoints.size(), 2);
    Eigen::VectorXd b2(closestPoints.size());
    for (int i = 0; i < closestPoints.size(); ++i) {
        A2(i, 0) = closestPoints[i](0);
        A2(i, 1) = 1.0;
        b2(i) = closestPoints[i](1);
    }
    Eigen::VectorXd x2 = A2.colPivHouseholderQr().solve(b2);
    auto line = std::make_pair(x2(0), x2(1));

    return line;
}

void CurveModeling::optimization() {
    std::vector<std::pair<double, double>> lines;
    for (double x = 1.0; x <= 30.0; x += 0.1) {
        double z = mesh_param_[0][0] * x * x + mesh_param_[0][1] * x + mesh_param_[0][2];
        double y = (x - plane_param_[0][1]) / plane_param_[0][0];
        Eigen::Vector3d p(x, y, z);
        
        std::pair<double, double> line;
        line = findClosestPoints(p, img_points_);
        lines.emplace_back(line);
    }

    optimization3DPoints(lines);
}

void CurveModeling::optimization3DPoints(std::vector<std::pair<double, double>> lines)
{
    ceres::Problem problem;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    // options.max_num_iterations = 8;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.num_threads = 8;

    for (size_t i = 0; i < lines.size(); ++i) {
        ceres::CostFunction* cost_function = CurveFactor::Create(lines[i], 0.2 * i + 1.0, Trans(R_c_l_, t_c_l_), cam_);
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