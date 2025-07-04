#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void print4x4Matrix(const Eigen::Matrix4f &matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.6f %6.6f %6.6f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.6f %6.6f %6.6f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.6f %6.6f %6.6f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.6f, %6.6f, %6.6f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void visualizePointClouds(const PointCloudT::Ptr cloud1,
                          const PointCloudT::Ptr cloud2,
                          const PointCloudT::Ptr cloud1_aligned)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // Color for the original point cloud (green)
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud1_color_handler(cloud1, 0, 255, 0);
    viewer->addPointCloud(cloud1, cloud1_color_handler, "cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");

    // Color for the target point cloud (blue)
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud2_color_handler(cloud2, 0, 0, 255);
    viewer->addPointCloud(cloud2, cloud2_color_handler, "cloud2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");

    // Color for the aligned point cloud (red)
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud1_aligned_color_handler(cloud1_aligned, 255, 0, 0);
    viewer->addPointCloud(cloud1_aligned, cloud1_aligned_color_handler, "cloud1_aligned");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1_aligned");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

PointCloudT::Ptr downsamplePointCloud(const PointCloudT::Ptr cloud, float leaf_size)
{
    PointCloudT::Ptr filtered(new PointCloudT);
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*filtered);
    return filtered;
}

Eigen::Matrix4f performNDTRegistration(const PointCloudT::Ptr source_cloud,
                                       const PointCloudT::Ptr target_cloud,
                                       float ndt_resolution = 1.0,
                                       float step_size = 0.1,
                                       float outlier_ratio = 0.55,
                                       int max_iterations = 100)
{
    PointCloudT::Ptr output_cloud(new PointCloudT);

    // Initialize NDT
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(step_size);
    ndt.setResolution(ndt_resolution);
    ndt.setMaximumIterations(max_iterations);
    ndt.setInputSource(source_cloud);
    ndt.setInputTarget(target_cloud);

    // Perform alignment
    ndt.align(*output_cloud);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
              << " score: " << ndt.getFitnessScore() << std::endl;

    Eigen::Matrix4f ndt_transformation = ndt.getFinalTransformation();
    print4x4Matrix(ndt_transformation);

    return ndt_transformation;
}

Eigen::Matrix4f performICPRegistration(const PointCloudT::Ptr source_cloud,
                                       const PointCloudT::Ptr target_cloud,
                                       const Eigen::Matrix4f &initial_guess,
                                       int max_iterations = 100,
                                       float max_correspondence_distance = 0.05,
                                       float transformation_epsilon = 1e-8,
                                       float fitness_epsilon = 1e-8)
{
    PointCloudT::Ptr output_cloud(new PointCloudT);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(max_iterations);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp.setTransformationEpsilon(transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(fitness_epsilon);
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    // Use the initial guess from NDT
    icp.align(*output_cloud, initial_guess);

    std::cout << "ICP has converged:" << icp.hasConverged()
              << " score: " << icp.getFitnessScore() << std::endl;

    Eigen::Matrix4f icp_transformation = icp.getFinalTransformation();
    print4x4Matrix(icp_transformation);

    return icp_transformation;
}

// Function to apply VIEWPOINT transformation to a point cloud
void applyViewpointTransform(PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_record)
{
    if (cloud->sensor_orientation_.w() == 0 &&
        cloud->sensor_orientation_.x() == 0 &&
        cloud->sensor_orientation_.y() == 0 &&
        cloud->sensor_orientation_.z() == 0)
    {
        // Default viewpoint (no rotation)
        cloud->sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0);
    }

    Eigen::Vector3f origin = cloud->sensor_origin_.head<3>();
    Eigen::Quaternionf rotation(cloud->sensor_orientation_);

    pcl::transformPointCloud(*cloud, *cloud, origin, rotation);


    cloud_record->sensor_orientation_ = cloud->sensor_orientation_;
    cloud_record->sensor_origin_ = cloud->sensor_origin_;

    // Reset viewpoint after applying
    cloud->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 0);
    cloud->sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0);

}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " source.pcd target.pcd" << std::endl;
        return -1;
    }

    // Load point clouds
    PointCloudT::Ptr cloud_source(new PointCloudT);
    PointCloudT::Ptr cloud_target(new PointCloudT);

    PointCloudT::Ptr cloud_src_viewpoint(new PointCloudT);
    PointCloudT::Ptr cloud_tgt_viewpoint(new PointCloudT);

    PointCloudT::Ptr cloud_test(new PointCloudT);

    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud_source) == -1)
    {
        PCL_ERROR("Couldn't read source file\n");
        return -1;
    }
    std::cout << "Loaded " << cloud_source->size() << " data points from source" << std::endl;
    std::cout << "Source viewpoint: " << cloud_source->sensor_origin_.transpose()
              << " " << cloud_source->sensor_orientation_.coeffs().transpose() << std::endl;

    if (pcl::io::loadPCDFile<PointT>(argv[2], *cloud_target) == -1)
    {
        PCL_ERROR("Couldn't read target file\n");
        return -1;
    }
    std::cout << "Loaded " << cloud_target->size() << " data points from target" << std::endl;
    std::cout << "Target viewpoint: " << cloud_target->sensor_origin_.transpose()
              << " " << cloud_target->sensor_orientation_.coeffs().transpose() << std::endl;

    if (pcl::io::loadPCDFile<PointT>(argv[3], *cloud_test) == -1)
    {
        PCL_ERROR("Couldn't read test file\n");
        return -1;
    }
    std::cout << "Loaded " << cloud_test->size() << " data points from target" << std::endl;
    std::cout << "Target viewpoint: " << cloud_test->sensor_origin_.transpose()
              << " " << cloud_test->sensor_orientation_.coeffs().transpose() << std::endl;

    // Apply viewpoint transformations to both clouds
    std::cout << "\nApplying viewpoint transformations..." << std::endl;
    applyViewpointTransform(cloud_source,cloud_src_viewpoint);
    applyViewpointTransform(cloud_target,cloud_tgt_viewpoint);
    pcl::io::savePCDFileASCII("/home/gct/LC-CurveModel/bin/viewpoint_transformed_source.pcd", *cloud_source);
    pcl::io::savePCDFileASCII("/home/gct/LC-CurveModel/bin/viewpoint_transformed_target.pcd", *cloud_target);

    // Downsample both clouds for faster processing
    float leaf_size = 0.1f;
    PointCloudT::Ptr cloud_source_downsampled = downsamplePointCloud(cloud_source, leaf_size);
    PointCloudT::Ptr cloud_target_downsampled = downsamplePointCloud(cloud_target, leaf_size);

    std::cout << "Downsampled source cloud from " << cloud_source->size()
              << " to " << cloud_source_downsampled->size() << " points" << std::endl;
    std::cout << "Downsampled target cloud from " << cloud_target->size()
              << " to " << cloud_target_downsampled->size() << " points" << std::endl;

    // First perform NDT for coarse registration
    std::cout << "\nStarting NDT registration..." << std::endl;
    Eigen::Matrix4f ndt_transform = performNDTRegistration(cloud_source_downsampled,
                                                           cloud_target_downsampled,
                                                           0.5,  // NDT resolution
                                                           0.1,  // Step size
                                                           0.55, // Outlier ratio
                                                           100); // Max iterations

    // Apply NDT transform to original source cloud
    PointCloudT::Ptr cloud_source_transformed(new PointCloudT);
    pcl::transformPointCloud(*cloud_source, *cloud_source_transformed, ndt_transform);

    // Then perform ICP for fine registration
    std::cout << "\nStarting ICP registration..." << std::endl;
    Eigen::Matrix4f icp_transform = performICPRegistration(cloud_source_transformed,
                                                           cloud_target,
                                                           Eigen::Matrix4f::Identity(),
                                                           100,   // Max iterations
                                                           0.5,   // Max correspondence distance
                                                           1e-8,  // Transformation epsilon
                                                           1e-8); // Fitness epsilon

    // Combine both transformations
    Eigen::Matrix4f final_transform = icp_transform * ndt_transform;
    std::cout << "\nFinal transformation matrix:" << std::endl;
    print4x4Matrix(final_transform);

    // Apply final transformation to original source cloud
    PointCloudT::Ptr cloud_source_final(new PointCloudT);
    pcl::transformPointCloud(*cloud_source, *cloud_source_final, final_transform);

    // Save aligned cloud
    pcl::io::savePCDFileASCII("output_aligned.pcd", *cloud_source_final);

    // Visualize results
    visualizePointClouds(cloud_target, cloud_source, cloud_source_final);

    Eigen::Matrix4f T_src_vp = Eigen::Matrix4f::Identity();
    T_src_vp.block<3, 1>(0, 3) = cloud_src_viewpoint->sensor_origin_.head<3>();
    T_src_vp.block<3, 3>(0, 0) = cloud_src_viewpoint->sensor_orientation_.toRotationMatrix();
    print4x4Matrix(T_src_vp);
    Eigen::Matrix4f T_tgt_vp = Eigen::Matrix4f::Identity();
    T_tgt_vp.block<3, 1>(0, 3) = cloud_tgt_viewpoint->sensor_origin_.head<3>();
    T_tgt_vp.block<3, 3>(0, 0) = cloud_tgt_viewpoint->sensor_orientation_.toRotationMatrix();
    print4x4Matrix(T_tgt_vp);


    Eigen::Matrix4f T_final = T_tgt_vp.inverse() * final_transform * T_src_vp;
    std::cout << "\nFinal transformation (Original Source -> Original Target):" << std::endl;
    print4x4Matrix(T_final);

    PointCloudT::Ptr cloud_source_orig_to_tgt(new PointCloudT);
    pcl::transformPointCloud(*cloud_test, *cloud_source_orig_to_tgt, T_final);

    pcl::io::savePCDFileASCII("final_aligned_original_coords.pcd", *cloud_source_orig_to_tgt);

    Eigen::Matrix4f T_final_inv = T_final.inverse();

    // 定义原始点
    Eigen::Vector4f P_orig(22.213433, -63.339874, 48.940902, 1.0);

    // 变换坐标
    Eigen::Vector4f P_transformed = T_final_inv * P_orig;

    // 输出结果（前三个分量）
    std::cout << "Transformed point: " 
              << P_transformed[0] << ", " 
              << P_transformed[1] << ", " 
              << P_transformed[2] << std::endl;

    return 0;
}