#ifndef LINE_EXTRACTOR_H
#define LINE_EXTRACTOR_H

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class LineExtractor {
public:
    LineExtractor();

    ~LineExtractor() = default;

    bool extractTwoLinesIsolated(std::vector<Eigen::Vector3d> &lidar_points_,
                                 std::vector<Eigen::Vector3d> &line_points_,
                                 int flag,
                                 double angle_threshold = 5.0,
                                 double distance_threshold = 0.02,
                                 double ransac_dist_thresh = 0.01,
                                 int ransac_max_iter = 5000,
                                 std::string path_ = "");

    PointCloudT::Ptr getLine1Cloud() const { return line1_cloud_; }

    PointCloudT::Ptr getLine2Cloud() const { return line2_cloud_; }

    double getLinesAngle() const { return line_angle_; }

private:
    double deg2rad(double deg);
    double rad2deg(double rad);

    bool fit3DLinePCL(const PointCloudT::Ptr& cloud,
                      double distance_threshold,
                      int max_iterations,
                      pcl::PointIndices::Ptr& inliers,
                      pcl::ModelCoefficients::Ptr& line_coeffs,
                      Eigen::Vector4d& line_centroid,
                      Eigen::Vector3d& line_dir);

    void filterSameLinePoints(const PointCloudT::Ptr& input_cloud,
                              const Eigen::Vector4d& line_centroid,
                              const Eigen::Vector3d& line_dir,
                              double angle_threshold,
                              double distance_threshold,
                              PointCloudT::Ptr& output_cloud);
    
    bool calculateAvgYAndZ(const PointCloudT::Ptr &cloud, double &avg_y, double &avg_z);

    PointCloudT::Ptr line1_cloud_;    
    PointCloudT::Ptr line2_cloud_;    
    double line_angle_; 
    std::string path;              
};

#endif 
