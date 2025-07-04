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

struct Cal_dist_paragram
{
    std::string file_line_pcd;
    std::string file_raw_pcd;
    float excu_line_threshold;
    float tunnel_radius;
    float match_distance_threshold;
    float excu_raw_radius_threshold;
    float visual_region_radius;

    std::pair<float, float> radius_filter_paragram;
    std::vector<std::string> output_files;

    Cal_dist_paragram(std::string file_line_pcd = "",
                      std::string file_raw_pcd = "",
                      std::vector<std::string> output_files = {"",""},
                      float excu_line_threshold = 0.15,
                      float tunnel_radius = 0.8,
                      float match_distance_threshold = 0.5,
                      float excu_raw_radius_threshold = 0.2,
                      float visual_region_radius =0.15,
                      std::pair<float, float> radius_filter_paragram = {0.1, 5}) : 
                      file_line_pcd(file_line_pcd),
                      file_raw_pcd(file_raw_pcd),
                      output_files(output_files),
                      excu_line_threshold(excu_line_threshold),
                      tunnel_radius(tunnel_radius),
                      match_distance_threshold(match_distance_threshold),
                      excu_raw_radius_threshold(excu_raw_radius_threshold),
                      visual_region_radius(visual_region_radius),
                      radius_filter_paragram(radius_filter_paragram){}
};

class Cal_distance
{
public:
    Cal_distance(Cal_dist_paragram cal_dist_paragram) : 
        file_line_pcd(cal_dist_paragram.file_line_pcd),
        file_raw_pcd(cal_dist_paragram.file_raw_pcd),
        output_files(cal_dist_paragram.output_files),
        radius_filter_paragram(cal_dist_paragram.radius_filter_paragram),
        excu_line_threshold(cal_dist_paragram.excu_line_threshold),
        tunnel_radius(cal_dist_paragram.tunnel_radius),
        match_distance_threshold(cal_dist_paragram.match_distance_threshold),
        excu_raw_radius_threshold(cal_dist_paragram.excu_raw_radius_threshold),
        visual_region_radius(cal_dist_paragram.visual_region_radius),
        cloud_result_line(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_raw(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_final(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_raw_filtered(new pcl::PointCloud<pcl::PointXYZ>){}


    void load_pcd_file();

    void excu_line_point();

    void cloud_tunnel_filter();

    void get_clostest_points();

    void visual();

    void calculate_distance();

private:
    float excu_line_threshold;
    float tunnel_radius;
    float match_distance_threshold;
    float excu_raw_radius_threshold;
    float visual_region_radius;

    std::string file_line_pcd;
    std::string file_raw_pcd;
    std::pair<float, float> radius_filter_paragram;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result_line;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_filtered;

    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> matched_points;
    Cal_dist_paragram cal_dist_paragram;

    std::vector<std::string> output_files;
};