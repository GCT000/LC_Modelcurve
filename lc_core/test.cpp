/**
 * @file   test.cpp
 * @brief  Test.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#include "curve_modeling.h"
#include "cloud_registration.h"
#include "cal_distance.h"
#include "loadPCD.h"
#include "bSpline.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <memory>
#include <sys/stat.h>
#include <errno.h>

// DEFINE_string(input_pcd, "/home/zyp/HD2/DATA/Transmisson/0912/test5/filter.pcd", "输入的点云");
// DEFINE_string(input_pcd, "/ssd/DATA/Transmisson/whu/0103/extracted03/filtered.pcd", "输入的点云");
DEFINE_string(yaml, "/home/gct/LC-CurveModel/config/whu/model1.yaml", "yaml文件");
DEFINE_string(dir, "/home/gct/LC-CurveModel/data/temp/log", "日志文件夹");
DEFINE_bool(visualize, true, "是否可视化");

/// @brief 测试加载pcd文件是否正常
TEST(loadPcdFile, loadPcd)
{
    std::string pcd_file = "test_pcd_file.pcd";
    lc_core::LoadPCD input_pcd;
    input_pcd(pcd_file);
    LOG(INFO) << "input points num: " << input_pcd.getPoints().size();
    EXPECT_EQ(input_pcd.getPoints().empty(), false);
}

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    // check log dir
    struct stat info;
    if (stat(FLAGS_dir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR))
    {
        int status = mkdir(FLAGS_dir.c_str(), 0777);
        if (status != 0)
        {
            LOG(ERROR) << "Cannot create log directory: " << FLAGS_dir << ", error code: " << strerror(errno);
        }
        else
        {
            LOG(INFO) << "Successfully create log directory: " << FLAGS_dir;
        }
    }

    google::InitGoogleLogging(argv[0]);
    // testing::InitGoogleTest(&argc, argv);
    // RUN_ALL_TESTS();
    FLAGS_colorlogtostderr = true;
    FLAGS_minloglevel = google::INFO;
    FLAGS_alsologtostderr = true;
    FLAGS_logtostderr = false;

    // set log dir
    FLAGS_log_dir = FLAGS_dir;
    google::SetLogFilenameExtension(".log");
    google::FlushLogFiles(google::INFO);

    lc_core::CurveModeling curve_modeling(FLAGS_yaml);


    // get end_point if 
    if (curve_modeling.if_no_end_point())
    {
        Cloud_registration cloud_registration;
        LOG(INFO) << "HERE";
        cloud_registration.cal_Tlw(curve_modeling.get_files_point().first, curve_modeling.get_files_point().second);
        curve_modeling.set_end_point(cloud_registration.get_end_point());
    }

    // process lidar points
    curve_modeling.lidarPreprocessing();
    curve_modeling.optimization();
    if (FLAGS_visualize)
    {
        curve_modeling.visualization();
    }
    
    // calculate distances 
    Cal_dist_paragram cal_dist_paragram("/home/gct/LC-CurveModel/data/wangan_0630/tempfinal_line_points.pcd",curve_modeling.get_cal_distance_files().first,curve_modeling.get_cal_distance_files().second);
    Cal_distance cal_distance(cal_dist_paragram);
    cal_distance.calculate_distance();

    return 0;
}
