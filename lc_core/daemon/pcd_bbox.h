/**
 * @file   pcd_bbox.h
 * @brief  Axis-aligned bounding box of a PCD point cloud, used to fill
 *         rectang_size and xy_interval for the first frame.
 * @date   2026-06
 */

#ifndef PCD_BBOX_H
#define PCD_BBOX_H

#include <string>
#include <vector>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <glog/logging.h>

namespace lc_daemon {

struct BBox {
    double x_min, x_max, y_min, y_max, z_min, z_max;
    bool valid = false;
};

// Compute the AABB over ALL points in pcd_path.
inline BBox computeBBox(const std::string& pcd_path) {
    BBox b;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1 || cloud->empty()) {
        LOG(ERROR) << "[bbox] cannot load or empty pcd: " << pcd_path;
        return b;
    }
    double inf = std::numeric_limits<double>::infinity();
    b.x_min = b.y_min = b.z_min =  inf;
    b.x_max = b.y_max = b.z_max = -inf;
    for (const auto& p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.x < b.x_min) b.x_min = p.x;
        if (p.x > b.x_max) b.x_max = p.x;
        if (p.y < b.y_min) b.y_min = p.y;
        if (p.y > b.y_max) b.y_max = p.y;
        if (p.z < b.z_min) b.z_min = p.z;
        if (p.z > b.z_max) b.z_max = p.z;
    }
    b.valid = (b.x_min <= b.x_max);
    LOG(INFO) << "[bbox] " << pcd_path << "  x[" << b.x_min << "," << b.x_max
              << "] y[" << b.y_min << "," << b.y_max
              << "] z[" << b.z_min << "," << b.z_max << "]";
    return b;
}

} // namespace lc_daemon

#endif
