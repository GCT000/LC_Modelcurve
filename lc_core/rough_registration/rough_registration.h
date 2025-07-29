#ifndef LIDAR_ECEF_TRANSFORM_H
#define LIDAR_ECEF_TRANSFORM_H

#include <Eigen/Dense>
#include <iostream>
#include <utility>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <glog/logging.h>


// Used to obtain a rough conversion from lidar to ECEF
class LidarEcefTransform {
public:
    // 构造函数：接收LiDAR在ECEF中的坐标和姿态角（单位：弧度）
    LidarEcefTransform(double la, double lon, double h,
                       double roll, double pitch, double yaw);

    // 获取旋转矩阵
    Eigen::Matrix3d getRotationMatrix() const;

    // 获取平移向量
    Eigen::Vector3d getTranslationVector() const;

    // 将LiDAR坐标系中的点转换到ECEF坐标系
    Eigen::Vector3d transformPoint(const Eigen::Vector3d& lidarPoint) const;

    void pos2ecef(double lat, double lon, double height, 
               double& ecefX, double& ecefY, double& ecefZ);

    Eigen::Matrix4f get_T_ecef_l();

private:
    // 旋转矩阵
    Eigen::Matrix3d rotation;

    // 平移向量
    Eigen::Vector3d translation;

    // 绕X轴旋转的旋转矩阵
    Eigen::Matrix3d rotationX(double angle);
    
    // 绕Y轴旋转的旋转矩阵
    Eigen::Matrix3d rotationY(double angle);
    
    // 绕Z轴旋转的旋转矩阵
    Eigen::Matrix3d rotationZ(double angle);

    // 计算从ENU坐标系到ECEF坐标系的旋转矩阵
    Eigen::Matrix3d enuToEcefRotation(double lat, double lon);
};


#endif // LIDAR_ECEF_TRANSFORM_H    