#include "imgPreProcess.h"
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>

DEFINE_string(points_file, "/home/zyp/Lidar/LC-CurveModel/build/curve_points.txt", "pre-curve points file");
DEFINE_string(pre_img, "/home/zyp/HD2/DATA/Transmisson/0912/test5/image_13.png", "pre-image path");
DEFINE_string(cur_img, "/home/zyp/HD2/DATA/Transmisson/0912/test5/image_20.png", "current image path");

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_minloglevel = google::INFO;
    FLAGS_logtostderr = true;

    ImgPreProcess pp;
    pp.setPoints(FLAGS_points_file);
    
    cv::Mat pre_img = cv::imread(FLAGS_pre_img);
    cv::Mat cur_img = cv::imread(FLAGS_cur_img);
    pp.track(pre_img, cur_img);
    pp.visualizeTracking(cur_img);
    pp.reInterpolate();
    pp.visualizeReInterpolated(cur_img);

    return 0;
}