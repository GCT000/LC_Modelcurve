/**
 * @file   points_generation.cpp
 * @brief  This file is used to generate points.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#include "points_simulation.hpp"
#include "output.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <yaml-cpp/yaml.h>
#include <boost/random.hpp>

DEFINE_string(yaml_file, "/home/zyp/Lidar/LC-CurveModel/simulation/config/points.yaml", "配置文件的路径");
DEFINE_string(output_file, "/home/zyp/Lidar/LC-CurveModel/simulation/data/points.txt", "输出文件的路径");
DEFINE_string(ref_points_file, "/home/zyp/Lidar/LC-CurveModel/simulation/data/ref_points.txt", "参考点文件的路径");
DEFINE_double(noise, 0.01, "高斯噪声的标准差");

void loadYaml(const std::string &file_name, int &points_num, double &y_base, Eigen::Vector3d &start_point, Eigen::Vector3d &end_point, Eigen::Vector3d &mid_point)
{
    YAML::Node config = YAML::LoadFile(file_name);

    if (config["points_num"])
    {
        points_num = config["points_num"].as<int>();
    }
    else
    {
        LOG(ERROR) << "No points_num in yaml file!";
    }

    if (config["y_base"])
    {
        y_base = config["y_base"].as<double>();
    }
    else
    {
        LOG(ERROR) << "No y_base in yaml file!";
    }

    if (config["start_point"])
    {
        start_point << config["start_point"][0].as<double>(), config["start_point"][1].as<double>(), config["start_point"][2].as<double>();
    }
    else
    {
        LOG(ERROR) << "No start_point in yaml file!";
    }

    if (config["end_point"])
    {
        end_point << config["end_point"][0].as<double>(), config["end_point"][1].as<double>(), config["end_point"][2].as<double>();
    }
    else
    {
        LOG(ERROR) << "No end_point in yaml file!";
    }

    if (config["mid_point"])
    {
        mid_point << config["mid_point"][0].as<double>(), config["mid_point"][1].as<double>(), config["mid_point"][2].as<double>();
    }
    else
    {
        LOG(ERROR) << "No mid_point in yaml file!";
    }
}

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostdout = true;
    FLAGS_stderrthreshold = google::INFO;
    
    // load variables from yaml file
    int points_num;
    double y_base;
    Eigen::Vector3d start_point, end_point, mid_point;
    loadYaml(FLAGS_yaml_file, points_num, y_base, start_point, end_point, mid_point);

    // generate points
    PointsSimulation<double> points_simulation(y_base, start_point, end_point, mid_point, points_num);

    std::vector<Eigen::Vector3d> points = points_simulation.getPoints();
    outputPoints(FLAGS_ref_points_file, points);

    // add gaussian noise
    boost::mt19937 rng; 
	rng.seed(static_cast<unsigned int>(time(0)));
	boost::normal_distribution<> nd(0, FLAGS_noise);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> var_nor(rng, nd);
    for (auto &p : points)
    {
        p.x() += var_nor();
        p.y() += var_nor();
        p.z() += var_nor();
    }

    outputPoints(FLAGS_output_file, points);

    return 0;
}