/**
 * @file   test.cpp
 * @brief  Test.
 * @author Chantian Gao
 * @date   2026-06
 */

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <memory>
#include <sys/stat.h>
#include <errno.h>

#include "curve_modeling.h"
#include "cal_distance.h"
#include "loadPCD.h"
#include "bSpline.h"


DEFINE_string(yaml, "/home/gct/LC_Modelcurve/config/whu/model1.yaml", "yaml文件");
DEFINE_string(dir, "/home/gct/LC_Modelcurve/data/temp/log", "日志文件夹");
DEFINE_bool(visualize, true, "是否可视化");


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
    FLAGS_colorlogtostderr = true;
    FLAGS_minloglevel = google::INFO;
    FLAGS_alsologtostderr = true;
    FLAGS_logtostderr = false;

    // set log dir
    FLAGS_log_dir = FLAGS_dir;
    google::SetLogFilenameExtension(".log");
    google::FlushLogFiles(google::INFO);

    lc_core::CurveModeling curve_modeling(FLAGS_yaml);

    //process lidar points
    curve_modeling.lidarPreprocessing();
    curve_modeling.optimization();
    if (FLAGS_visualize)
    {
        curve_modeling.visualization();
    }

    //calculate distances
    Cal_dist_paragram cal_dist_paragram(curve_modeling.get_res_path()+ "final_line_points.pcd",curve_modeling.get_cal_distance_files().first,curve_modeling.get_cal_distance_files().second,curve_modeling.get_cal_distance_para(), curve_modeling.ex_line_tower_X());
    Cal_distance cal_distance(cal_dist_paragram);
    cal_distance.calculate_distance();

    return 0;
}
