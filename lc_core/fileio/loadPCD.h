/**
 * @file   loadPCD.h
 * @brief  This file defines the class for loading PCD file.
 * @author Yipeng Zhao
 * @date   2024-08
 */
#ifndef LOADPCD_H
#define LOADPCD_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>

namespace lc_core
{

    class LoadPCD
    {
    public:
        LoadPCD() = default;

        ~LoadPCD() = default;

        /// @brief  Load PCD file
        void operator()(const std::string &file_name)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1)
            {
                PCL_ERROR("Couldn't read file %s\n", file_name.c_str());
                return;
            }

            points_.reserve(cloud->points.size());
            for (const auto& p : cloud->points)
            {
                points_.emplace_back(p.x, p.y, p.z);
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