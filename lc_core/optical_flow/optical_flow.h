#ifndef OPTICAL_FLOW
#define OPTICAL_FLOW
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <yaml-cpp/yaml.h>

#include "imgPreProcess.h"
#include "camera.h"
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace lc_core
{
    class Optical_flow
    {
    public:
        Optical_flow();
        Optical_flow(const std::string &yaml_file);
        bool do_optical_flow();

        void loadCamera(const YAML::Node &yaml, const std::string &yaml_file);

        void loadLidar2CameraExtrinsic(const YAML::Node &yaml);

        bool read_img();

    private:
        std::string curve_points_file;
        std::string img_path;
        std::string last_img_path;
        std::string result_flag_path;
        std::shared_ptr<Camera> cam_;
        cv::Mat img, last_img;
        std::vector<cv::Point2d> img_points_;
        bool is_track;
        Eigen::Matrix3d R_c_l_;
        Eigen::Vector3d t_c_l_;
    };
}
#endif