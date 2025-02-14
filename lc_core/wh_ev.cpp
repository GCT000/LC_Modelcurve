#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <gtest/gtest.h>
#include "transmissionModel.h"
#include "catenary.h"
#include "parabola.h"
#include "output.h"
#include "matcher.h"
#include "evaluation.h"
#include "camera.h"
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <ceres/ceres.h>

using namespace std;
using namespace lc_core;
using namespace Eigen;

struct Error {
    double max_y_err;
    double avg_y_err;
    double rmse_y;
    double max_z_err;
    double avg_z_err;
    double rmse_z;
    double max_err;
    double avg_err;
    double rmse;
};

static std::string temp_path = "/home/zyp/Lidar/LC-CurveModel/temp/whhp";

Matrix3d Rcl = (Matrix3d() <<
    0.021222,  -0.999692, -0.0128505,
    0.0036544,  0.0129309,   -0.99991,
    0.999768,   0.021173, 0.00392775).finished();
;
Vector3d tcl = (Vector3d() <<
    -0.0425305,
    0.0922477,
    -0.117265).finished();

Matrix3d RG = (Matrix3d() <<
    0.999999523163, -0.000662012841, -0.000743341749,
    0.000662013015, 0.999999761581, 0.000000000000,
    0.000743341574, -0.000000492102, 0.999999701977).finished();

Vector3d tG = (Vector3d() <<
    -0.216916769743,
    -4.571582794189,
    -0.006015432999).finished();

Vector3d GS = (Vector3d() <<
    -532672.54,
    -3411741.57,
    0.0).finished();

void transPoints(const string& file_path)
{
    ifstream file(file_path);
    fstream output_points(temp_path + "/points.txt", std::ios::out);
    string line;
    while (getline(file, line))
    {
        Vector3d point, point_trans;
        sscanf(line.c_str(), "%lf %lf %lf", &point[0], &point[1], &point[2]);
        point_trans = point + GS;
        output_points << point_trans[0] << " " << point_trans[1] << " " << point_trans[2] << "\n";
    }
    output_points.close();
}

vector<Vector3d> loadPoints(const string& file_path)
{
    ifstream file(file_path);
    vector<Vector3d> points;
    string line;
    while (getline(file, line))
    {
        Vector3d point;
        sscanf(line.c_str(), "%lf %lf %lf", &point[0], &point[1], &point[2]);
        points.emplace_back(point);
    }
    return points;
}

cv::Mat image = cv::imread("/home/zyp/HD2/DATA/Transmisson/0912/test5/image_13.png");

Camera cam{
    .img_h_ = 2048,
    .img_w_ = 2448,
    .fx_ = 3512.8854091528533,
    .fy_ = 3511.9388239960267,
    .cx_ = 1226.5000494955293,
    .cy_ = 976.092491004889,
    .k1_ = 0.006100839571101789,
    .k2_ = 0.29178313649440873,
    .p1_ = -0.002251959981440175,
    .p2_ = -0.00020094572053250403,
    .k3_ = 0.0
};

vector<cv::Point2d> loadPoints2d(const string& file_path)
{
    ifstream file(file_path);
    vector<cv::Point2d> points;
    string line;
    while (getline(file, line))
    {
        cv::Point2d point;
        sscanf(line.c_str(), "%lf %lf", &point.x, &point.y);
        points.push_back(point);
    }
    return points;
}

Error calculateError(const vector<Vector3d>& ref_points, const vector<Vector3d>& points)
{
    Error error;
    double max_y_err = 0, avg_y_err = 0, rmse_y = 0;
    double max_z_err = 0, avg_z_err = 0, rmse_z = 0;
    double max_err = 0, avg_err = 0, rmse = 0;
    for (size_t i = 0; i < ref_points.size(); i++)
    {
        double err = (ref_points[i] - points[i]).norm();
        double y_err = abs(ref_points[i][1] - points[i][1]);
        double z_err = abs(ref_points[i][2] - points[i][2]);
        max_y_err = max_y_err > y_err ? max_y_err : y_err;
        avg_y_err += y_err;
        rmse_y += y_err * y_err;
        max_z_err = max_z_err > z_err ? max_z_err : z_err;
        avg_z_err += z_err;
        rmse_z += z_err * z_err;
        max_err = max_err > err ? max_err : err;
        avg_err += err;
        rmse += err * err;
    }
    avg_err /= ref_points.size();
    rmse = sqrt(rmse / ref_points.size());
    error.max_err = max_err;
    error.avg_err = avg_err;
    error.rmse = rmse;
    error.max_y_err = max_y_err;
    error.avg_y_err = avg_y_err / ref_points.size();
    error.rmse_y = sqrt(rmse_y / ref_points.size());
    error.max_z_err = max_z_err;
    error.avg_z_err = avg_z_err / ref_points.size();
    error.rmse_z = sqrt(rmse_z / ref_points.size());
    return error;
}

Vector2d lidar2pixel(const Vector3d& p_l, const Camera& cam) {
    Vector3d p_c = Rcl * p_l + tcl;
    Vector2d p_img;
    cam.spaceToPlane(p_c, p_img);
    return p_img;
}

void drawMatchResultOnImage(const std::string& filename, const cv::Mat& img) {
    std::fstream output_points("/home/zyp/Lidar/LC-CurveModel/temp/match.txt", std::ios::in);
    cv::Mat match_img = img.clone();
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
    cv::imwrite(filename, match_img);
    output_points.close();
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
TEST(lidar_test, test_lidar)
{
    transPoints("/home/zyp/Lidar/LC-CurveModel/temp/whhp/dji_line10_points.txt");
    vector<Vector3d> ref_points = loadPoints("/home/zyp/Lidar/LC-CurveModel/temp/whhp/points.txt");
    cout << "ref_points size: " << ref_points.size() << "\n";
    cout << "ref_points[0]: " << ref_points[0] << "\n";
    Catenary ref_catenary;
    // Parabola ref_catenary, tele_catenary;
    // I20250108 16:59:29.773806 2292966 catenary.cpp:35] c: 483.112 c1: -59.0344 c2: -486.303
    // I20250108 16:59:29.773883 2292966 catenary.cpp:36] k: -0.0472548 m: -0.327019
    ref_catenary.fitTransmissionModel(ref_points);
    vector<Vector3d> ref_points_new;
    for (double x = 5.2; x < 162.6; x += 0.2)
    {
        Vector3d ref_point = ref_catenary.generateSinglePoint(x);
        ref_points_new.push_back(ref_point);
    }
    outputPoints("/home/zyp/Lidar/LC-CurveModel/temp/whhp/ref_line10_points.txt", ref_points_new);
    vector<Vector3d> first_points, middle_points, last_points;
    // first_points = loadPoints("/home/zyp/Lidar/LC-CurveModel/temp/original_output_lidar_points.txt");
    // middle_points = loadPoints("/home/zyp/Lidar/LC-CurveModel/temp/middle_output_lidar_points.txt");
    first_points = loadPoints("/home/zyp/Lidar/LC-CurveModel/temp/whhp/original_output_lidar_points.txt");
    middle_points = loadPoints("/home/zyp/Lidar/LC-CurveModel/temp/whhp/middle_output_lidar_points.txt");
    last_points = loadPoints("/home/zyp/Lidar/LC-CurveModel/temp/whhp/final_output_lidar_points.txt");

    Error first_error = calculateError(ref_points_new, first_points);
    Error middle_error = calculateError(ref_points_new, middle_points);
    Error last_error = calculateError(ref_points_new, last_points);

    cout << "first_error:-----------------------------------\n " 
    << "max_err: " << first_error.max_err << "\n"
    << "avg_err: " << first_error.avg_err << "\n"
    << "rmse: " << first_error.rmse << "\n"
    << "max_y_err: " << first_error.max_y_err << "\n"
    << "avg_y_err: " << first_error.avg_y_err << "\n"
    << "rmse_y: " << first_error.rmse_y << "\n"
    << "max_z_err: " << first_error.max_z_err << "\n"
    << "avg_z_err: " << first_error.avg_z_err << "\n"
    << "rmse_z: " << first_error.rmse_z << "\n";
    cout << "middle_error:-----------------------------------\n " 
    << "max_err: " << middle_error.max_err << "\n"
    << "avg_err: " << middle_error.avg_err << "\n"
    << "rmse: " << middle_error.rmse << "\n"
    << "max_y_err: " << middle_error.max_y_err << "\n"
    << "avg_y_err: " << middle_error.avg_y_err << "\n"
    << "rmse_y: " << middle_error.rmse_y << "\n"
    << "max_z_err: " << middle_error.max_z_err << "\n"
    << "avg_z_err: " << middle_error.avg_z_err << "\n"
    << "rmse_z: " << middle_error.rmse_z << "\n";
    cout << "last_error:-----------------------------------\n " 
    << "max_err: " << last_error.max_err << "\n"
    << "avg_err: " << last_error.avg_err << "\n"
    << "rmse: " << last_error.rmse << "\n"
    << "max_y_err: " << last_error.max_y_err << "\n"
    << "avg_y_err: " << last_error.avg_y_err << "\n"
    << "rmse_y: " << last_error.rmse_y << "\n"
    << "max_z_err: " << last_error.max_z_err << "\n"
    << "avg_z_err: " << last_error.avg_z_err << "\n"
    << "rmse_z: " << last_error.rmse_z << "\n";
}

TEST(image_test, test_image)
{
    // curve points
    vector<cv::Point2d> curve_points = loadPoints2d("/home/zyp/Lidar/LC-CurveModel/temp/curve_points.txt");
    // ref
    vector<Vector3d> ref_points;
    ref_points = loadPoints("/home/zyp/Lidar/LC-CurveModel/temp/whhp/ref_line10_points.txt");
    vector<cv::Point2d> ref_points_image;
    for (auto& point : ref_points)
    {
        Vector2d p_img = lidar2pixel(point, cam);
        if (p_img[0] > 0 && p_img[0] < image.cols && p_img[1] > 0 && p_img[1] < image.rows) {
            ref_points_image.push_back(cv::Point2d(p_img[0], p_img[1]));
        }
    }
    // first
    vector<Vector3d> first_points;
    first_points = loadPoints("/home/zyp/Lidar/LC-CurveModel/temp/whhp/original_output_lidar_points.txt");
    vector<cv::Point2d> first_points_image;
    for (auto& point : first_points)
    {
        Vector2d p_img = lidar2pixel(point, cam);
        if (p_img[0] > 0 && p_img[0] < image.cols && p_img[1] > 0 && p_img[1] < image.rows) {
            first_points_image.push_back(cv::Point2d(p_img[0], p_img[1]));
        }
    }
    // middle
    vector<Vector3d> middle_points;
    middle_points = loadPoints("/home/zyp/Lidar/LC-CurveModel/temp/whhp/middle_output_lidar_points.txt");
    vector<cv::Point2d> middle_points_image;
    for (auto& point : middle_points)
    {
        Vector2d p_img = lidar2pixel(point, cam);
        if (p_img[0] > 0 && p_img[0] < image.cols && p_img[1] > 0 && p_img[1] < image.rows) {
            middle_points_image.push_back(cv::Point2d(p_img[0], p_img[1]));
        }
    }
    // last
    vector<Vector3d> last_points;
    last_points = loadPoints("/home/zyp/Lidar/LC-CurveModel/temp/whhp/final_output_lidar_points.txt");
    vector<cv::Point2d> last_points_image;
    for (auto& point : last_points)
    {
        Vector2d p_img = lidar2pixel(point, cam);
        if (p_img[0] > 0 && p_img[0] < image.cols && p_img[1] > 0 && p_img[1] < image.rows) {
            last_points_image.push_back(cv::Point2d(p_img[0], p_img[1]));
        }
    }

    MatcherConfig matcher_config {
        .type = 1
    };
    Matcher matcher(matcher_config);
    Matcher::MatchResult match_result = matcher.match(ref_points_image, curve_points);
    drawMatchResultOnImage("/home/zyp/Lidar/LC-CurveModel/temp/whhp/ref_match_result.png", image);
    auto [ref_avg_err, ref_max_err] = calculateReprojectError(match_result, ref_points_image);
    cout << "ref avg_err: " << ref_avg_err << "\n"
         << "ref max_err: " << ref_max_err << "\n";
    match_result = matcher.match(first_points_image, curve_points);
    drawMatchResultOnImage("/home/zyp/Lidar/LC-CurveModel/temp/whhp/first_match_result.png", image);
    auto [first_avg_err, first_max_err] = calculateReprojectError(match_result, first_points_image);
    cout << "first avg_err: " << first_avg_err << "\n"
         << "first max_err: " << first_max_err << "\n";
    match_result = matcher.match(middle_points_image, curve_points);
    drawMatchResultOnImage("/home/zyp/Lidar/LC-CurveModel/temp/whhp/middle_match_result.png", image);
    auto [middle_avg_err, middle_max_err] = calculateReprojectError(match_result, middle_points_image);
    cout << "middle avg_err: " << middle_avg_err << "\n"
         << "middle max_err: " << middle_max_err << "\n";
    match_result = matcher.match(last_points_image, curve_points);
    drawMatchResultOnImage("/home/zyp/Lidar/LC-CurveModel/temp/whhp/last_match_result.png", image);
    auto [last_avg_err, last_max_err] = calculateReprojectError(match_result, last_points_image);
    cout << "last avg_err: " << last_avg_err << "\n"
         << "last max_err: " << last_max_err << "\n";
}

TEST(exOptimization, test_exOptimization)
{
    // curve points
    vector<cv::Point2d> curve_points = loadPoints2d("/home/zyp/Lidar/LC-CurveModel/temp/curve_points.txt");
    vector<Vector3d> ref_points = loadPoints("/home/zyp/Lidar/LC-CurveModel/temp/whhp/ref_line10_points.txt");
    // ref
    vector<cv::Point2d> ref_points_image;
    cv::Mat image_copy = image.clone();
    vector<int> x;
    for (size_t i = 0; i < ref_points.size(); ++i)
    {
        Vector2d p_img = lidar2pixel(ref_points[i], cam);
        if (p_img[0] > 0 && p_img[0] < image.cols && p_img[1] > 0 && p_img[1] < image.rows) {
            ref_points_image.push_back(cv::Point2d(p_img[0], p_img[1]));
            cv::circle(image_copy, cv::Point(p_img[0], p_img[1]), 3, cv::Scalar(0, 0, 255), -1);
            x.push_back(i);
        }
    }
    cv::imwrite("/home/zyp/Lidar/LC-CurveModel/temp/whhp/ref_points_image.png", image_copy);
    MatcherConfig matcher_config {
        .type = 1
    };
    Matcher matcher(matcher_config);
    Matcher::MatchResult match_result = matcher.match(ref_points_image, curve_points);
    P2PMatchResult p2p_match_result;
    if (match_result.index() == 0) {
        p2p_match_result = std::get<P2PMatchResult>(match_result);
    }

    ceres::Problem problem;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    // options.max_num_iterations = 8;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    double q[4] = {Quaterniond(Rcl).coeffs().data()[0], 
                   Quaterniond(Rcl).coeffs().data()[1],
                   Quaterniond(Rcl).coeffs().data()[2], 
                   Quaterniond(Rcl).coeffs().data()[3]};
    double t[3] = {tcl.data()[0], tcl.data()[1], tcl.data()[2]};
    problem.AddParameterBlock(q, 4, new ceres::EigenQuaternionParameterization());
    problem.AddParameterBlock(t, 3);
    problem.SetParameterBlockConstant(t);

    shared_ptr<Camera> camPtr = std::make_shared<Camera>(cam);
    for (size_t i = 0; i < x.size(); ++i) {
        ceres::CostFunction* cost_function = ExFactor::Create(ref_points[x[i]], p2p_match_result[i], camPtr);
        problem.AddResidualBlock(cost_function, nullptr, q, t);
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    Eigen::Quaterniond q_c_l(q[3], q[0], q[1], q[2]);
    Rcl = q_c_l.toRotationMatrix();
    tcl << t[0], t[1], t[2];

    cout << "After optimization: \n";
    cout << "Rcl: \n" << Rcl << "\n";
    cout << "tcl: \n" << tcl << "\n";

    ref_points_image.clear();
    for (size_t i = 0; i < x.size(); ++i)
    {
        Vector2d p_img = lidar2pixel(ref_points[x[i]], cam);
        ref_points_image.push_back(cv::Point2d(p_img[0], p_img[1]));
    }
    matcher.match(ref_points_image, curve_points);
    drawMatchResultOnImage("/home/zyp/Lidar/LC-CurveModel/temp/whhp/ref_match_result_after_optimization.png", image);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}