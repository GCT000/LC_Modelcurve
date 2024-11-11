/**
 * @file   loadPCD.hpp
 * @brief  This file defines the class for loading PCD file.
 * @author Yipeng Zhao
 * @date   2024-08
*/
#ifndef LOADPCD_H
#define LOADPCD_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>

namespace lc_core
{

class LoadPCD
{
public:
    LoadPCD() = default;

    ~LoadPCD() = default;

    /// @brief  Load PCD file
    void operator()(const std::string &file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read file %s\n", file_name.c_str());
            return;
        }
        
        for (int i = 0; i < cloud->points.size(); i++)
        {
            Eigen::Vector3d point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            points_.push_back(point);
        }
    }

    /// @brief  Get points
    std::vector<Eigen::Vector3d> getPoints() const
    {
        return points_;
    }

private:
    std::vector<Eigen::Vector3d> points_;
};

} // namespace lc_core

#endif