#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <glog/logging.h>

#include <iostream>
#include <limits>
#include <vector>
#include <utility>
#include <fstream>
#include <chrono>

struct Cal_dist_paragram
{
    std::string file_line_pcd;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw;
    float excu_line_threshold;
    float tunnel_radius;
    float match_distance_threshold;
    float excu_raw_radius_threshold;
    float visual_region_radius;

    std::pair<float, float> radius_filter_paragram;
    std::pair<float, float> ex_line_tower_X;
    std::vector<std::string> output_files;

    Cal_dist_paragram(std::string file_line_pcd = "",
                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(),
                      std::vector<std::string> output_files = {"","",""},
                      std::vector<float>radius_para = {0,0,0,0,0},
                      std::pair<float, float> ex_line_tower_X = std::make_pair(0, 0),
                      std::pair<float, float> radius_filter_paragram = {0.1, 5}) : 
                      file_line_pcd(file_line_pcd),
                      cloud_raw(cloud_raw),
                      output_files(output_files),
                      excu_line_threshold(radius_para[0]),
                      tunnel_radius(radius_para[1]),
                      match_distance_threshold(radius_para[2]),
                      excu_raw_radius_threshold(radius_para[3]),
                      visual_region_radius(radius_para[4]),
                      radius_filter_paragram(radius_filter_paragram),
                      ex_line_tower_X(ex_line_tower_X)
                      {
                        LOG(INFO) << "Begin to cal_distance";
                        bool is_file_line_empty = file_line_pcd.empty();
                        bool is_cloud_empty = (cloud_raw == nullptr) || (cloud_raw->empty());

                        bool is_output_files_empty = false;
                        if (!output_files.empty()) {
                            for (const auto& file : output_files) {
                                if (file.empty()) {
                                    is_output_files_empty = true;
                                    break;
                                }
                            }
                        }
                        if (is_file_line_empty)
                        {
                            LOG(ERROR) << "When calculating the distance, there is no input file or no output file path." << file_line_pcd;
                            exit(EXIT_FAILURE);
                        }
                      }
};

class Cal_distance
{
public:
    Cal_distance(Cal_dist_paragram cal_dist_paragram) : 
        file_line_pcd(cal_dist_paragram.file_line_pcd),
        output_files(cal_dist_paragram.output_files),
        radius_filter_paragram(cal_dist_paragram.radius_filter_paragram),
        excu_line_threshold(cal_dist_paragram.excu_line_threshold),
        tunnel_radius(cal_dist_paragram.tunnel_radius),
        match_distance_threshold(cal_dist_paragram.match_distance_threshold),
        excu_raw_radius_threshold(cal_dist_paragram.excu_raw_radius_threshold),
        visual_region_radius(cal_dist_paragram.visual_region_radius),
        ex_line_tower_X(cal_dist_paragram.ex_line_tower_X),
        cloud_result_line(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_raw(cal_dist_paragram.cloud_raw),
        cloud_final(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_raw_filtered(new pcl::PointCloud<pcl::PointXYZ>){
        if (!std::all_of(output_files.begin(), output_files.end(), [](const std::string& s) {
            return !s.empty();
        }))
        {
            LOG(ERROR) << "some output files are empty strings";
        }
        }


    void load_pcd_file();

    void cloud_tunnel_filter();

    void get_clostest_points();

    void visual();

    void calculate_distance();

    void save_match_points_txt();

private:
    float excu_line_threshold;
    float tunnel_radius;
    float match_distance_threshold;
    float excu_raw_radius_threshold;
    float visual_region_radius;

    std::string file_line_pcd;
    std::pair<float, float> radius_filter_paragram;
    std::pair<float, float> ex_line_tower_X;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result_line;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_filtered;

    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> matched_points;

    std::vector<std::string> output_files;
};