#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <glog/logging.h>
#include <boost/thread/thread.hpp>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct Ndt_transform
{
    Eigen::Matrix4f ndt_transform;
    float resolution;
    float step_size;
    float outlier_ratio;
    float max_iterations;
    Ndt_transform(const Eigen::Matrix4f &initmatrix = Eigen::Matrix4f::Identity(),
                  float resolution_ = 0.5, float step_size_ = 0.1,
                  float outlier_ratio_ = 0.55, float max_iterations_ = 100) : 
                  resolution(resolution_),
                  step_size(step_size_),
                  outlier_ratio(outlier_ratio_),
                  max_iterations(max_iterations_) {}
};

struct Icp_transform
{
    Eigen::Matrix4f icp_transform;
    float max_iterations;
    float max_correspondence_distance;
    float transformation_epsilon;
    float fitness_epsilon;

    Icp_transform(const Eigen::Matrix4f &initmatrix = Eigen::Matrix4f::Identity(),
                  float max_iterations_ = 100, float max_correspondence_distance_ = 0.5,
                  float transformation_epsilon_ = 1e-8, float fitness_epsilon_ = 1e-8) : 
                  icp_transform(initmatrix),
                  max_iterations(max_iterations_),
                  max_correspondence_distance(max_correspondence_distance_),
                  transformation_epsilon(transformation_epsilon_),
                  fitness_epsilon(fitness_epsilon_) {}
};

class Cloud_registration
{
public:
    Cloud_registration();
    /// @brief  load pcd files
    void load_file();
    /// @brief  print T
    void print4x4Matrix(const Eigen::Matrix4f &matrix);
    /// @brief  visualize
    void visualizePointClouds();

    PointCloudT::Ptr downsamplePointCloud(const PointCloudT::Ptr cloud);
    /// @brief NDT
    void performNDTRegistration();
    /// @brief ICP
    void performICPRegistration(const Eigen::Matrix4f &initial_guess);
    /// @brief use viewpoint correct
    void applyViewpointTransform(PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_record);

    void get_file_names(std::vector<std::string> file_names);
    /// @brief main
    void cal_Tlw(std::vector<std::string> file_names, Eigen::Vector4f end_point_);

    Eigen::Vector4f get_end_point();
private:
    std::string source_file, target_file;

    Eigen::Vector4f end_point;
    Eigen::Vector4f P_transformed;

    float leaf_size;
    Eigen::Matrix4f final_transform;
    Eigen::Matrix4f T_src_vp;
    Eigen::Matrix4f T_tgt_vp;
    Eigen::Matrix4f T_final;
    Eigen::Matrix4f T_final_inv;

    PointCloudT::Ptr cloud_source;
    PointCloudT::Ptr cloud_target;

    PointCloudT::Ptr cloud_src_viewpoint;
    PointCloudT::Ptr cloud_tgt_viewpoint;

    PointCloudT::Ptr cloud_source_downsampled;
    PointCloudT::Ptr cloud_target_downsampled;

    PointCloudT::Ptr cloud_source_transformed;

    PointCloudT::Ptr cloud_source_final;

    Ndt_transform ndt_transform;
    Icp_transform icp_transform;
};