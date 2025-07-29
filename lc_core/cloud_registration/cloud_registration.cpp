#include "cloud_registration.h"

Cloud_registration::Cloud_registration(Data_paragram data_para_, Eigen::Matrix4f T_ecef_l_)
{
    data_para = data_para_;
    leaf_size = 0.03;
    end_point = {0, 0, 0, 0};
    T_ecef_l = T_ecef_l_;
    final_transform = Eigen::Matrix4f::Identity();
    T_src_vp = Eigen::Matrix4f::Identity();
    T_tgt_vp = Eigen::Matrix4f::Identity();
    T_final = Eigen::Matrix4f::Identity();
    T_final_inv = Eigen::Matrix4f::Identity();
    cloud_source = boost::make_shared<PointCloudT>();
    cloud_target = boost::make_shared<PointCloudT>();
    cloud_src_viewpoint = boost::make_shared<PointCloudT>();
    cloud_tgt_viewpoint = boost::make_shared<PointCloudT>();
    cloud_source_downsampled = boost::make_shared<PointCloudT>();
    cloud_target_downsampled = boost::make_shared<PointCloudT>();
    cloud_source_transformed = boost::make_shared<PointCloudT>();
    cloud_source_final = boost::make_shared<PointCloudT>();
    ndt_transform = Ndt_transform(Eigen::Matrix4f::Identity(), data_para.ndt_resolution, data_para.ndt_step_size, data_para.ndt_outlier_ratio, data_para.ndt_max_iterations);
    icp_transform = Icp_transform(Eigen::Matrix4f::Identity(), data_para.icp_max_iterations, data_para.icp_max_correspondence_distance, data_para.icp_transformation_epsilon, data_para.icp_fitness_epsilon);
}

void Cloud_registration::get_file_names(std::vector<std::string> file_names)
{
    if (file_names.size() < 2)
    {
        LOG(INFO) << "no enough files";
    }
    source_file = file_names[0];
    target_file = file_names[1];
}

Eigen::Vector4f Cloud_registration::get_end_point()
{
    return P_transformed;
}

void Cloud_registration::load_file()
{
    if (pcl::io::loadPCDFile<PointT>(source_file, *cloud_source) == -1)
    {
        PCL_ERROR("Couldn't read source file\n");
    }
    LOG(INFO) << "Loaded " << cloud_source->size() << " data points from source";
    LOG(INFO) << "Source viewpoint: " << cloud_source->sensor_origin_.transpose()
              << " " << cloud_source->sensor_orientation_.coeffs().transpose();

    if (pcl::io::loadPCDFile<PointT>(target_file, *cloud_target) == -1)
    {
        PCL_ERROR("Couldn't read target file\n");
    }
    LOG(INFO) << "Loaded " << cloud_target->size() << " data points from target";
    LOG(INFO) << "Target viewpoint: " << cloud_target->sensor_origin_.transpose()
              << " " << cloud_target->sensor_orientation_.coeffs().transpose();
}

void Cloud_registration::print4x4Matrix(const Eigen::Matrix4f &matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.6f %6.6f %6.6f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.6f %6.6f %6.6f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.6f %6.6f %6.6f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.6f, %6.6f, %6.6f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void Cloud_registration::visualizePointClouds()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // Color for the original point cloud (green)
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_target_color_handler(cloud_target, 0, 255, 0);
    viewer->addPointCloud(cloud_target, cloud_target_color_handler, "cloud_target");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_target");

    // Color for the target point cloud (blue)
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_source_color_handler(cloud_source, 0, 0, 255);
    viewer->addPointCloud(cloud_source, cloud_source_color_handler, "cloud_source");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_source");

    // Color for the aligned point cloud (red)
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_source_final_color_handler(cloud_source_final, 255, 0, 0);
    viewer->addPointCloud(cloud_source_final, cloud_source_final_color_handler, "cloud_source_final");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_source_final");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

PointCloudT::Ptr Cloud_registration::downsamplePointCloud(const PointCloudT::Ptr cloud)
{
    PointCloudT::Ptr filtered(new PointCloudT);
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*filtered);
    return filtered;
}

void Cloud_registration::performNDTRegistration()
{
    PointCloudT::Ptr output_cloud(new PointCloudT);

    // Initialize NDT
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(ndt_transform.step_size);
    ndt.setResolution(ndt_transform.resolution);
    ndt.setMaximumIterations(ndt_transform.max_iterations);
    ndt.setInputSource(cloud_source_downsampled);
    ndt.setInputTarget(cloud_target_downsampled);

    // Perform alignment
    ndt.align(*output_cloud);

    LOG(INFO) << "Normal Distributions Transform has converged:" << ndt.hasConverged()
              << " score: " << ndt.getFitnessScore();

    ndt_transform.ndt_transform = ndt.getFinalTransformation();
    print4x4Matrix(ndt_transform.ndt_transform);
}

void Cloud_registration::performICPRegistration(const Eigen::Matrix4f &initial_guess)
{
    PointCloudT::Ptr output_cloud(new PointCloudT);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(icp_transform.max_iterations);
    icp.setMaxCorrespondenceDistance(icp_transform.max_correspondence_distance);
    icp.setTransformationEpsilon(icp_transform.transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(icp_transform.fitness_epsilon);
    icp.setUseReciprocalCorrespondences(true);
    icp.setInputSource(cloud_source_transformed);
    icp.setInputTarget(cloud_target);

    // Use the initial guess from NDT
    icp.align(*output_cloud, initial_guess);

    LOG(INFO) << "ICP has converged:" << icp.hasConverged()
              << " score: " << icp.getFitnessScore();

    icp_transform.icp_transform = icp.getFinalTransformation();
    print4x4Matrix(icp_transform.icp_transform);
}

// Function to apply VIEWPOINT transformation to a point cloud
void Cloud_registration::applyViewpointTransform(PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_record)
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

void Cloud_registration::cal_Tlw(std::vector<std::string> file_names, Eigen::Vector4f end_point_)
{
    get_file_names(file_names);
    load_file();
    end_point = end_point_;
    // Apply viewpoint transformations to both clouds
    LOG(INFO) << "Applying viewpoint transformations...";
    applyViewpointTransform(cloud_source, cloud_src_viewpoint);
    applyViewpointTransform(cloud_target, cloud_tgt_viewpoint);

    // Downsample both clouds for faster processing
    cloud_source_downsampled = downsamplePointCloud(cloud_source);
    cloud_target_downsampled = downsamplePointCloud(cloud_target);

    LOG(INFO) << "Downsampled source cloud from " << cloud_source->size()
              << " to " << cloud_source_downsampled->size() << " points";
    LOG(INFO) << "Downsampled target cloud from " << cloud_target->size()
              << " to " << cloud_target_downsampled->size() << " points";

    // First perform NDT for coarse registration
    LOG(INFO) << "Starting NDT registration...";
    performNDTRegistration(); // Max iterations

    // Apply NDT transform to original source cloud
    pcl::transformPointCloud(*cloud_source, *cloud_source_transformed, ndt_transform.ndt_transform);

    // Then perform ICP for fine registration
    LOG(INFO) << "\nStarting ICP registration...";
    performICPRegistration(ndt_transform.ndt_transform);

    //icp_transform.icp_transform = Eigen::Matrix4f::Identity();

    // Combine both transformations
    final_transform = icp_transform.icp_transform;
    LOG(INFO) << "\nFinal transformation matrix:";
    print4x4Matrix(final_transform);

    // Apply final transformation to original source cloud
    pcl::transformPointCloud(*cloud_source, *cloud_source_final, final_transform);

    // Save aligned cloud
    pcl::io::savePCDFileASCII("output_aligned.pcd", *cloud_source_final);

    // Visualize results
    visualizePointClouds();

    // get TS TT
    T_src_vp = Eigen::Matrix4f::Identity();
    T_src_vp.block<3, 1>(0, 3) = cloud_src_viewpoint->sensor_origin_.head<3>();
    T_src_vp.block<3, 3>(0, 0) = cloud_src_viewpoint->sensor_orientation_.toRotationMatrix();

    T_tgt_vp = Eigen::Matrix4f::Identity();
    T_tgt_vp.block<3, 1>(0, 3) = cloud_tgt_viewpoint->sensor_origin_.head<3>();
    T_tgt_vp.block<3, 3>(0, 0) = cloud_tgt_viewpoint->sensor_orientation_.toRotationMatrix();

    // L_2_W
    T_final = T_tgt_vp.inverse() * final_transform * T_src_vp * T_ecef_l;
    LOG(INFO) << "Final transformation (Original Source -> Original Target):";
    print4x4Matrix(T_final);

    // W_2_L
    T_final_inv = T_final.inverse();
    // Eigen::Vector4f P_orig(22.213433, -63.339874, 48.940902, 1.0);

    P_transformed = T_final_inv * end_point;

    LOG(INFO) << "Transformed point: "
              << P_transformed[0] << ", "
              << P_transformed[1] << ", "
              << P_transformed[2];
}