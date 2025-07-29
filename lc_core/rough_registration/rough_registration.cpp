#include "rough_registration.h"
#include <cmath>

LidarEcefTransform::LidarEcefTransform(double la, double lon, double h,
                                       double roll, double pitch, double yaw)
{
    // 设置平移向量
    double ecef[3] = {0, 0, 0};
    pos2ecef(la, lon, h, ecef[0], ecef[1], ecef[2]);
    translation = Eigen::Vector3d(ecef[0], ecef[1], ecef[2]);

    LOG(INFO) << "ECEF_end_point:" << std::setprecision(10)<< ecef[0] << "   " << ecef[1] << "   " << ecef[2] << std::endl;

    // 计算从LiDAR坐标系到ENU坐标系的旋转矩阵（R_enu_lidar）
    Eigen::Matrix3d R_yaw = rotationZ(yaw);
    Eigen::Matrix3d R_pitch = rotationY(pitch);
    Eigen::Matrix3d R_roll = rotationX(roll);
    Eigen::Matrix3d R_enu_lidar = R_yaw * R_pitch * R_roll;
    //  计算从ENU坐标系到ECEF坐标系的旋转矩阵（R_ecef_enu）
    Eigen::Matrix3d R_ecef_enu = enuToEcefRotation(la, lon);

    // 计算总的旋转矩阵（R_ecef_lidar = R_ecef_enu * R_enu_lidar）
    rotation = R_ecef_enu * R_enu_lidar;

    // for debug get rotated pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/media/gct/T9/canglong/place3.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the input PCD file\n");
    }

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.rotate(rotation);
    transform.translation() = translation;

    // 应用变换到点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    pcl::io::savePCDFileASCII("/media/gct/T9/canglong/place3/place3_calib_out.pcd", *transformed_cloud);
    LOG(INFO) << "Transformed point cloud saved to output.pcd" << std::endl;
}

Eigen::Matrix4f LidarEcefTransform::get_T_ecef_l()
{
    Eigen::Matrix4f T;
    T.setIdentity();
    T.block<3, 3>(0, 0) = rotation.cast<float>();
    T.block<3, 1>(0, 3) = translation.cast<float>();
    return T;
}

Eigen::Matrix3d LidarEcefTransform::getRotationMatrix() const
{
    return rotation;
}

Eigen::Vector3d LidarEcefTransform::getTranslationVector() const
{
    return translation;
}

Eigen::Vector3d LidarEcefTransform::transformPoint(const Eigen::Vector3d &lidarPoint) const
{
    // 应用旋转和平移
    return rotation * lidarPoint + translation;
}

void LidarEcefTransform::pos2ecef(double lat, double lon, double height,
                                  double &ecefX, double &ecefY, double &ecefZ)
{
    const double a = 6378137.0;           // 长半轴（米）
    const double f = 1.0 / 298.257223563; // 扁率
    double sinp = sin(lat), cosp = cos(lat), sinl = sin(lon), cosl = cos(lon);
    double e2 = f * (2.0 - f), v = a / sqrt(1.0 - e2 * sinp * sinp);

    ecefX = (v + height) * cosp * cosl;
    ecefY = (v + height) * cosp * sinl;
    ecefZ = (v * (1.0 - e2) + height) * sinp;
}

Eigen::Matrix3d LidarEcefTransform::rotationX(double angle)
{
    Eigen::Matrix3d R;
    double c = std::cos(angle);
    double s = std::sin(angle);
    R << 1.0, 0.0, 0.0,
        0.0, c, -s,
        0.0, s, c;
    return R;
}

Eigen::Matrix3d LidarEcefTransform::rotationY(double angle)
{
    Eigen::Matrix3d R;
    double c = std::cos(angle);
    double s = std::sin(angle);
    R << c, 0.0, s,
        0.0, 1.0, 0.0,
        -s, 0.0, c;
    return R;
}

Eigen::Matrix3d LidarEcefTransform::rotationZ(double angle)
{
    Eigen::Matrix3d R;
    double c = std::cos(angle);
    double s = std::sin(angle);
    R << c, -s, 0.0,
        s, c, 0.0,
        0.0, 0.0, 1.0;
    return R;
}

Eigen::Matrix3d LidarEcefTransform::enuToEcefRotation(double lat, double lon)
{
    Eigen::Matrix3d R;
    double sin_lat = std::sin(lat);
    double cos_lat = std::cos(lat);
    double sin_lon = std::sin(lon);
    double cos_lon = std::cos(lon);

    // ENU到ECEF的旋转矩阵
    R << -sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon,
        cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon,
        0.0, cos_lat, sin_lat;

    return R;
}
