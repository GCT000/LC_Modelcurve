/**
 * @file   points_simulation.hpp
 * @brief  This file is used to output results to files.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#ifndef OUTPUT_H
#define OUTPUT_H

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <glog/logging.h>

/// @brief  output points to file
template <typename T>
void outputPoints(const std::string &file_name, const std::vector<cv::Point_<T>> &points)
{
    std::ofstream out_file(file_name, std::ios::out);
    if (!out_file.is_open())
    {
        LOG(ERROR) << "Can't open file: " << file_name;
        return;
    }

    for (const auto &point : points)
    {
        out_file << point.x << " " << point.y << std::endl;
    }

    out_file.close();
}

/// @brief  output points and gradients to file
template <typename T>
void outputPointsAndGrad(const std::string &file_name, 
    const std::vector<std::pair<cv::Point_<T>, std::pair<double, double>>> &points)
{
    std::ofstream out_file(file_name, std::ios::out);
    if (!out_file.is_open())
    {
        LOG(ERROR) << "Can't open file: " << file_name;
        return;
    }

    for (const auto &point : points)
    {
        out_file << point.first.x << " " << point.first.y << " "
                 << point.second.first << " " << point.second.second << std::endl;
    }

    out_file.close();
}

/// @brief  output points to file
void outputPoints(const std::string &file_name, const std::vector<Eigen::Vector3d> &points)
{
    std::ofstream out_file(file_name, std::ios::out);
    if (!out_file.is_open())
    {
        LOG(ERROR) << "Can't open file: " << file_name;
        return;
    }

    for (const auto &point : points)
    {
        out_file << point.x() << " " << point.y() << " " << point.z() << std::endl;
    }

    out_file.close();
}

void outputPoints(const std::string &file_name, const std::vector<cv::Point2d> &points)
{
    std::ofstream out_file(file_name, std::ios::out);
    if (!out_file.is_open())
    {
        LOG(ERROR) << "Can't open file: " << file_name;
        return;
    }

    for (const auto &point : points)
    {
        out_file << point.x << " " << point.y << std::endl;
    }

    out_file.close();
}

#endif // OUTPUT_H