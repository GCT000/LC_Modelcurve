#include "cal_distance.h"

void Cal_distance::load_pcd_file()
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_line_pcd, *cloud_result_line) == -1)
    {
        PCL_ERROR("Couldn't read result_line\n");
    }
    LOG(INFO) << "Loaded " << cloud_result_line->size() << " data points from " << file_line_pcd;
}

void Cal_distance::excu_line_point()
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_result_line);

    for (const auto &point : *cloud_raw)
    {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (kdtree.radiusSearch(point, excu_line_threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0)
        {
            cloud_raw_filtered->push_back(point);
        }
    }
    LOG(INFO) << "After excluding finalline points from rawpcd, remaining points in cloud_raw: " << cloud_raw_filtered->size();
}

void Cal_distance::cloud_tunnel_filter()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_result_line);

    cloud_filtered->clear();
    cloud_filtered->reserve(cloud_raw_filtered->size());

    for (const auto &point : *cloud_raw_filtered)
    {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            float distance = std::sqrt(pointNKNSquaredDistance[0]);

            if (distance <= tunnel_radius)
            {
                cloud_filtered->push_back(point);
            }
        }
    }

    cloud_filtered->width = cloud_filtered->size();
    cloud_filtered->height = 1;

    LOG(INFO) << "Points within tunnel: " << cloud_filtered->size();

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
    radius_filter.setInputCloud(cloud_filtered);
    radius_filter.setRadiusSearch(radius_filter_paragram.first);
    radius_filter.setMinNeighborsInRadius(radius_filter_paragram.second);
    radius_filter.filter(*cloud_final);

    LOG(INFO) << "Complete tunnel filter";
}

void Cal_distance::get_clostest_points()
{
    pcl::KdTreeFLANN<pcl::PointXYZ> cloud1_kdtree;
    cloud1_kdtree.setInputCloud(cloud_result_line);

    std::vector<bool> processed_points(cloud_final->size(), false);

    pcl::KdTreeFLANN<pcl::PointXYZ> final_cloud_kdtree;
    final_cloud_kdtree.setInputCloud(cloud_final);

    for (size_t i = 0; i < cloud_final->size(); ++i)
    {
        if (processed_points[i])
        {
            continue;
        }
        const auto &point = cloud_final->points[i];
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (cloud1_kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            float distance = std::sqrt(pointNKNSquaredDistance[0]);
            if (distance < match_distance_threshold)
            {
                pcl::PointXYZ matched_point = cloud_result_line->points[pointIdxNKNSearch[0]];
                matched_points.push_back(std::make_pair(point, matched_point));
                std::vector<int> radius_indices;
                std::vector<float> radius_distances;
                if (final_cloud_kdtree.radiusSearch(point, excu_raw_radius_threshold, radius_indices, radius_distances) > 0)
                {
                    for (int idx : radius_indices)
                    {
                        processed_points[idx] = true;
                    }
                }
            }
        }
    }
    LOG(INFO) << "Found " << matched_points.size() << " point pairs.";
}

void Cal_distance::visual()
{
    if (output_files.size() < 2)
    {
        LOG(ERROR) << "No enough output files";
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_matches(new pcl::PointCloud<pcl::PointXYZRGB>);

    // KDTree
    pcl::KdTreeFLANN<pcl::PointXYZ> cloud1_kdtree_for_region;
    cloud1_kdtree_for_region.setInputCloud(cloud_result_line);

    pcl::KdTreeFLANN<pcl::PointXYZ> cloud_final_kdtree_for_region;
    cloud_final_kdtree_for_region.setInputCloud(cloud_final);

    // mark points
    for (const auto &pair : matched_points)
    {
        pcl::PointXYZRGB p1;
        p1.x = pair.first.x;
        p1.y = pair.first.y;
        p1.z = pair.first.z;
        p1.r = 255;
        p1.g = 0;
        p1.b = 0; // red cloud_final

        pcl::PointXYZRGB p2;
        p2.x = pair.second.x;
        p2.y = pair.second.y;
        p2.z = pair.second.z;
        p2.r = 0;
        p2.g = 0;
        p2.b = 255; // blue cloud_line

        colored_matches->push_back(p1);
        colored_matches->push_back(p2);

        // mark raw_cloud nearby points
        std::vector<int> final_region_indices;
        std::vector<float> final_region_distances;

        if (cloud_final_kdtree_for_region.radiusSearch(pair.first, visual_region_radius, final_region_indices, final_region_distances) > 0)
        {
            for (int idx : final_region_indices)
            {
                pcl::PointXYZRGB region_point;
                region_point.x = cloud_final->points[idx].x;
                region_point.y = cloud_final->points[idx].y;
                region_point.z = cloud_final->points[idx].z;
                region_point.r = 255; // little red
                region_point.g = 128;
                region_point.b = 128;

                if (std::abs(region_point.x - p1.x) > 1e-6 ||
                    std::abs(region_point.y - p1.y) > 1e-6 ||
                    std::abs(region_point.z - p1.z) > 1e-6)
                {
                    colored_matches->push_back(region_point);
                }
            }
        }

        // mark line_cloud nearby points
        std::vector<int> cloud1_region_indices;
        std::vector<float> cloud1_region_distances;

        if (cloud1_kdtree_for_region.radiusSearch(pair.second, visual_region_radius, cloud1_region_indices, cloud1_region_distances) > 0)
        {
            for (int idx : cloud1_region_indices)
            {
                pcl::PointXYZRGB region_point;
                region_point.x = cloud_result_line->points[idx].x;
                region_point.y = cloud_result_line->points[idx].y;
                region_point.z = cloud_result_line->points[idx].z;
                region_point.r = 128; // little blue
                region_point.g = 128;
                region_point.b = 255;

                if (std::abs(region_point.x - p2.x) > 1e-6 ||
                    std::abs(region_point.y - p2.y) > 1e-6 ||
                    std::abs(region_point.z - p2.z) > 1e-6)
                {
                    colored_matches->push_back(region_point);
                }
            }
        }
    }
    // save cloud
    pcl::io::savePCDFileASCII(output_files[0], *colored_matches);
    LOG(INFO) << "Matched points saved to matched_points.pcd";

    // cloud_final to white for compare
    pcl::PointCloud<pcl::PointXYZ>::Ptr white_cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &point : *cloud_final)
    {
        pcl::PointXYZ white_point;
        white_point.x = point.x;
        white_point.y = point.y;
        white_point.z = point.z;
        white_cloud_final->push_back(white_point);
    }

    pcl::PCDWriter writer;
    // 使用二进制格式保存（更高效），如果需要ASCII格式可以将第二个参数改为true
    if (writer.writeBinary(output_files[1], *white_cloud_final) == -1)
    {
        LOG(ERROR) << "Failed to open pcd file for writing: " << output_files[1];
    }
    else
    {
        LOG(INFO) << "Filtered cloud saved to pcd file: " << output_files[1]
                  << " with " << white_cloud_final->size() << " points.";
    }
}

void Cal_distance::save_match_points_txt()
{
    std::ofstream file(output_files[2]);
    if (!file.is_open())
    {
        LOG(ERROR) << "Fail to open : " << output_files[2];
        exit(EXIT_FAILURE);
    }

    for (const auto &pair : matched_points)
    {
        file << "first point: " << pair.first.x << " " << pair.first.y << " " << pair.first.z << "   second point: "
             << pair.second.x << " " << pair.second.y << " " << pair.second.z << "\n";
    }

    file.close();
    LOG(INFO) << "Successfully saved " << matched_points.size() << " point pairs to the file " << output_files[2];
}

void Cal_distance::calculate_distance()
{
    load_pcd_file();
    excu_line_point();
    cloud_tunnel_filter();
    get_clostest_points();
    visual();
    save_match_points_txt();
}
