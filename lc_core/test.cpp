/**
 * @file   test.cpp
 * @brief  Test.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <memory>
#include <sys/stat.h>
#include <errno.h>

#include "curve_modeling.h"
#include "cloud_registration.h"
#include "rough_registration.h"
#include "cal_distance.h"
#include "loadPCD.h"
#include "bSpline.h"

extern "C"
{
#include "monitor.h"
#include "rtklib.h"
}

// DEFINE_string(input_pcd, "/home/zyp/HD2/DATA/Transmisson/0912/test5/filter.pcd", "输入的点云");
// DEFINE_string(input_pcd, "/ssd/DATA/Transmisson/whu/0103/extracted03/filtered.pcd", "输入的点云");
DEFINE_string(yaml, "/home/gct/LC-CurveModel/config/whu/model1.yaml", "yaml文件");
DEFINE_string(dir, "/home/gct/LC-CurveModel/data/temp/log", "日志文件夹");
DEFINE_bool(visualize, true, "是否可视化");

void test_gnss(std::vector<double> &rover, std::vector<double> &angle)
{
    mInfo moniInfo1;
    gtime_t ts, te;
    double es[6] = {2025, 7, 10, 9, 18, 37}, ee[6] = {2025, 7, 10, 9, 24, 37};
    ts = epoch2time(es);
    te = epoch2time(ee);
    gtime_t tn = te;
    char tsstr[40], testr[40];
    time2str(ts, tsstr, 0);
    time2str(tn, testr, 0);
    sprintf(moniInfo1.configStr, "3@2@%s@%s@@0@rover@base@/media/gct/T9/canglong/place3gnss/@.obs@/media/gct/T9/canglong/BRDM1910.rnx@45@/home/gct/LC-CurveModel/data/test.pos@0@0@0.5@@@", tsstr, testr);

    startMonitor(&moniInfo1);

    printf("solBuf: %s\n", moniInfo1.solBuf);
    rover.push_back(moniInfo1.blh[0]);
    rover.push_back(moniInfo1.blh[1]);
    rover.push_back(moniInfo1.blh[2]);

    double yaw = std::atan2(moniInfo1.enu[0], moniInfo1.enu[1]);
    if (yaw < 0)
    {
        yaw += 2 * M_PI;
    }
    angle.push_back(yaw);
    std::cout.precision(15);
    LOG(INFO) << "blh:   " << std::setprecision(10) << moniInfo1.blh[0] << "    " << moniInfo1.blh[1] << "        " << moniInfo1.blh[2] << std::endl;
    LOG(INFO) << "enu:   " << std::setprecision(10) << moniInfo1.enu[0] << "    " << moniInfo1.enu[1] << "        " << moniInfo1.enu[2] << std::endl;
    LOG(INFO) << "yaw:   " << std::setprecision(10) << yaw << std::endl;
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

    std::vector<double> rover;
    std::vector<double> angle;

    lc_core::CurveModeling curve_modeling(FLAGS_yaml);

    // get end_point if no input
    if (curve_modeling.if_no_end_point())
    {
        LOG(INFO) << "Cannot get end point. Begin to estimate it";
        test_gnss(rover, angle);
        LidarEcefTransform lidarEcefTransform(rover[0], rover[1], rover[2], 0, -0.23, 2.5 * M_PI - angle[0]);
        Eigen::Matrix4f T_ecef_l = lidarEcefTransform.get_T_ecef_l();

        Data_paragram data_para(curve_modeling.get_ndt_icp_para().first, curve_modeling.get_ndt_icp_para().second);
        Cloud_registration cloud_registration(data_para, T_ecef_l);
        cloud_registration.cal_Tlw(curve_modeling.get_files_point().first, curve_modeling.get_files_point().second);
        curve_modeling.set_end_point(cloud_registration.get_end_point());
    }

    //process lidar points
    curve_modeling.lidarPreprocessing();
    curve_modeling.optimization();
    if (FLAGS_visualize)
    {
        curve_modeling.visualization();
    }

    //calculate distances
    
    Cal_dist_paragram cal_dist_paragram(curve_modeling.get_res_path()+ "final_line_points.pcd",curve_modeling.get_cal_distance_files().first,curve_modeling.get_cal_distance_files().second,curve_modeling.get_cal_distance_para(), curve_modeling.ex_line_tower_X());
    //Cal_dist_paragram cal_dist_paragram("/home/gct/LC-CurveModel/lc_tools/filtered_2.pcd",curve_modeling.get_cal_distance_files().first,curve_modeling.get_cal_distance_files().second,curve_modeling.get_cal_distance_para(), curve_modeling.ex_line_tower_X());
    Cal_distance cal_distance(cal_dist_paragram);
    cal_distance.calculate_distance();

    return 0;
}
