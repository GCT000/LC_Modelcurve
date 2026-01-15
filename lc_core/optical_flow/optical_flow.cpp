#include "optical_flow.h"

DEFINE_string(yaml, "/home/gct/LC-CurveModel/config/whu/optical_flow.yaml", "yaml文件");

using namespace lc_core;
Optical_flow::Optical_flow() {};

Optical_flow::Optical_flow(const std::string &yaml_file)
{
    YAML::Node yaml = YAML::LoadFile(yaml_file);
    if (yaml["img_path"])
    {
        img_path = yaml["img_path"].as<std::string>();
        if (img_path.empty())
        {
            LOG(ERROR) << "No img_path. please set it!!";
            exit(EXIT_FAILURE);
        }
    }
    if (yaml["last_img_path"])
    {
        last_img_path = yaml["last_img_path"].as<std::string>();
        if (last_img_path.empty())
        {
            LOG(ERROR) << "No last_img_path. please set it!!";
            exit(EXIT_FAILURE);
        }
    }
    if (yaml["curve_points_file"])
    {
        curve_points_file = yaml["curve_points_file"].as<std::string>();
        if (curve_points_file.empty())
        {
            LOG(ERROR) << "No curve_points_file. please set it!!";
            exit(EXIT_FAILURE);
        }
    }
    if (yaml["result_flag_path"])
    {
        result_flag_path = yaml["result_flag_path"].as<std::string>();
        if (result_flag_path.empty())
        {
            LOG(ERROR) << "No result_flag_path. please set it!!";
            exit(EXIT_FAILURE);
        }
    }

    loadCamera(yaml, yaml_file);

    // load lidar-camera extrinsic
    loadLidar2CameraExtrinsic(yaml);
}
void Optical_flow::loadCamera(const YAML::Node &yaml, const std::string &yaml_file)
{
    if (yaml["cam_calib"])
    {
        std::string camera_file;
        int pn = yaml_file.find_last_of("/");
        camera_file = yaml_file.substr(0, pn + 1) + yaml["cam_calib"].as<std::string>();
        // load camera parameters
        cam_ = std::make_shared<Camera>();
        cam_->readIntrinsicParameters(camera_file);
    }
    else
    {
        LOG(ERROR) << "No camera file found in yaml file.\n";
        return;
    }
}

void Optical_flow::loadLidar2CameraExtrinsic(const YAML::Node &yaml)
{
    if (!yaml["T_cam_lidar"])
    {
        LOG(ERROR) << "No extrinsic parameters found in yaml file.\n";
        return;
    }

    std::vector<double> vecT = yaml["T_cam_lidar"]["data"].as<std::vector<double>>();
    assert(vecT.size() == 16);

    // load extrinsic parameters
    Eigen::Matrix4d T = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(vecT.data());
    R_c_l_ = T.block<3, 3>(0, 0);
    t_c_l_ = T.block<3, 1>(0, 3);
}


bool Optical_flow::read_img()
{
    cv::Mat img_color = cv::imread(img_path, cv::IMREAD_COLOR);
    cv::Mat last_img_color = cv::imread(last_img_path, cv::IMREAD_COLOR);

    cv::Mat img_color_;
    cv::Mat last_img_color_;

    if (img_color.empty() || last_img_color.empty())
    {
        LOG(ERROR) << "error to load img or last img, path: " << img_path << ", " << last_img_path;
        return false;
    }

    cam_->undistortImg(img_color, img_color_);
    cam_->undistortImg(last_img_color, last_img_color_);

    cv::cvtColor(img_color_, img, cv::COLOR_BGR2GRAY);
    cv::cvtColor(last_img_color_, last_img, cv::COLOR_BGR2GRAY);
    return true;
}

void outputPoints(const std::string &file_name, const std::vector<cv::Point2d> &points)
{
    std::ofstream out_file(file_name, std::ios::out);
    if (!out_file.is_open())
    {
        LOG(ERROR) << "Can't open file: " << file_name;
        LOG(ERROR) << "Can't open file: " << file_name;
        return;
    }

    for (const auto &point : points)
    {
        out_file << point.x << " " << point.y << std::endl;
    }

    out_file.close();
}

bool Optical_flow::do_optical_flow()
{
    read_img();
    lc_preprocess::ImgPreProcess img_process;
    img_process.setPoints(curve_points_file);

    img_process.track(last_img, img);
    std::ofstream ofs(result_flag_path, std::ios::trunc);
    // pp.visualizeTracking(img_);
    if (!img_process.Is_track())
    {
        img_points_ = img_process.Get_pre_point();
        LOG(INFO) << "read last img points : " << img_points_.size();
        is_track = false;
        LOG(INFO) << "Fail to track img";
        ofs << 0;
        return false;
    }
    else
    {
        img_process.reInterpolate();
        img_points_ = img_process.get_cur_points();
        img_process.visualizeReInterpolated(img);

        outputPoints(curve_points_file, img_points_);
        is_track = true;
        LOG(INFO) << "Finish optical_flow";
        ofs << 3;
        return true;
    }
    ofs.close();
}

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true; 
    FLAGS_minloglevel = 0; 
    // 2. 创建工具类实例
    Optical_flow optical_flow(FLAGS_yaml);
    bool flag = optical_flow.do_optical_flow();
    return 0;
}
