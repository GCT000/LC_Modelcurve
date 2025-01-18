#include "fileio/output.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(lidar_points, "/home/zyp/Lidar/LC-CurveModel/temp/final_output_lidar_points.txt", "要添加的lidar点");
// DEFINE_string(input_pcd, "/home/zyp/HD2/DATA/Transmisson/0912/test5/extracted_points.pcd", "输入的点云");
DEFINE_string(output_pcd, "line_1.pcd", "输出的点云");

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    lc_core::outputPCD(FLAGS_lidar_points, FLAGS_output_pcd);
    return 0;
}
