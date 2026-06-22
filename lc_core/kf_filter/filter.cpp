#include "filter.h"





DEFINE_string(yaml, "/home/gct/LC_Modelcurve/config/whu/filter.yaml", "yaml文件");
// 构造函数（默认）
FusionPCDTool::FusionPCDTool() {}

// 带参数构造函数
FusionPCDTool::FusionPCDTool(const std::string &yaml_file)
{
    YAML::Node yaml = YAML::LoadFile(yaml_file);
    if (yaml["input_path_R"])
    {
        input_path_R_ = yaml["input_path_R"].as<std::string>();
        if (input_path_R_.empty())
        {
            LOG(ERROR) << "No input_path_R_. please set it!!";
            exit(EXIT_FAILURE);
        }
    }
    if (yaml["input_path_Q"])
    {
        input_path_Q_ = yaml["input_path_Q"].as<std::string>();
        if (input_path_Q_.empty())
        {
            LOG(ERROR) << "No input_path_Q_. please set it!!";
            exit(EXIT_FAILURE);
        }
    }
    if (yaml["input_path_Q_raw"])
    {
        input_path_Q_raw_ = yaml["input_path_Q_raw"].as<std::string>();
        if (input_path_Q_raw_.empty())
        {
            LOG(ERROR) << "No input_path_Q_raw_. please set it!!";
            exit(EXIT_FAILURE);
        }
    }
    if (yaml["output_path"])
    {
        output_path_ = yaml["output_path"].as<std::string>();
        if (output_path_.empty())
        {
            LOG(ERROR) << "No output_path_. please set it!!";
            exit(EXIT_FAILURE);
        }
    }
    if (yaml["pcd_path"])
    {
        pcd_path = yaml["pcd_path"].as<std::string>();
        if (pcd_path.empty())
        {
            LOG(ERROR) << "No pcd_path. please set it!!";
            exit(EXIT_FAILURE);
        }
    }
}

// 读取单个文件数据
void FusionPCDTool::readSingleFile(std::ifstream &ifs, std::vector<Vector2d> &z, VectorXd &y, std::vector<Matrix2d> &R)
{
    if (!ifs.is_open())
    {
        throw std::runtime_error("文件流未打开");
    }

    std::ofstream ofs("/home/gct/LC_Modelcurve/data/filter_test/tes.txt", std::ios::trunc);

    std::string line;
    int line_num = 0;
    y.resize(0);
    while (std::getline(ifs, line))
    {
        line_num++;
        if (line.empty() || line[0] == '#')
            continue;

        std::istringstream iss(line);
        std::vector<double> data;
        std::string token;
        while (iss >> token)
        {
            try
            {
                data.push_back(std::stod(token));
            }
            catch (const std::invalid_argument &e)
            {
                LOG(ERROR) << "第" << line_num << "行存在非数字数据，跳过：" << token << std::endl;
                data.clear();
                break;
            }
        }
        if (data.size() != DATA_COLUMN_COUNT)
        {
            LOG(ERROR) << "第" << line_num << "行列数不符（期望9列，实际" << data.size() << "列），跳过" << std::endl;
            continue;
        }
        y.conservativeResize(y.size() + 1);

        Vector2d temp_z;
        Matrix2d temp_R;

        temp_z(0) = data[0];
        temp_z(1) = data[2];
        y(y.size() - 1) = data[1];
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                temp_R(i, j) = 0.0;
                temp_R(j, i) = 0.0;
            }
        }

        const double cov_xx = std::max(data[3], -data[3]);
        const double cov_xz = data[5];
        const double cov_zz = std::max(data[8], -data[8]);

        temp_R(0, 0) = cov_xx;
        temp_R(1, 0) = cov_xz;
        temp_R(0, 1) = cov_xz;
        temp_R(1, 1) = cov_zz;

        z.push_back(temp_z);
        R.push_back(temp_R);
    }

    ifs.close();
    for (size_t i = 0; i < z.size(); i++)
    {
        ofs << "data: " << z[i](0) << "  " << z[i](1) << std::endl;
    }
    for (size_t i = 0; i < R.size(); i++)
    {
        ofs << "R   : " << R[i](0, 0) << "  " << R[i](1, 1) << std::endl;
        ofs << "R   : " << R[i](0, 1) << "  " << R[i](1, 0) << std::endl;
    }
    LOG(INFO) << "[INFO] 文件读取完成: 点数量=" << y.size()<< std::endl;
    LOG(INFO) << "[INFO] 文件读取完成: Z尺寸=" << z.size()<< std::endl;
    LOG(INFO) << "[INFO] 文件读取完成: R尺寸=" << R.size()<< std::endl;
}

bool FusionPCDTool::readInputData()
{
    try
    {
        std::ifstream ifsr(input_path_R_);
        std::ifstream ifsq(input_path_Q_);
        std::ifstream ifsq_raw(input_path_Q_raw_);
        if (!ifsr.is_open())
            throw std::runtime_error("无法打开文件：" + input_path_R_);
        if (!ifsq.is_open())
            throw std::runtime_error("无法打开文件：" + input_path_Q_);
        if (!ifsq_raw.is_open())
            throw std::runtime_error("无法打开文件：" + input_path_Q_raw_);
        readSingleFile(ifsr, zr_, yr_, Rr_);
        readSingleFile(ifsq, zq_, yq_, Rq_);
        readSingleFile(ifsq_raw, zq_raw_, yq_raw_, Rq_raw_);

        const int point_num = yr_.size();
        if (zr_.size() != point_num || zq_.size() != point_num || zq_raw_.size() != point_num)
        {
            throw std::runtime_error("观测向量长度与点数量不匹配");
        }
    }
    catch (const std::exception &e)
    {
        LOG(ERROR) << "数据读取失败：" << e.what() << std::endl;
        return false;
    }

    return true;
}

// 2x2矩阵SVD求逆
bool FusionPCDTool::matrixInverse2x2(const Matrix2d &mat, Matrix2d &inv_mat)
{
    Eigen::JacobiSVD<Matrix2d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // 检查最小奇异值（确保矩阵可逆）
    if (svd.singularValues()(1) < EPS)
    {
        LOG(ERROR) << "矩阵不可逆，最小奇异值：" << svd.singularValues()(1) << std::endl;
        return false;
    }
    // SVD求逆公式：A⁻¹ = VΣ⁻¹Uᵀ
    inv_mat = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();
    return true;
}

// WLS静态融合
bool FusionPCDTool::fuse2dStatic(const Vector2d &z1, const Matrix2d &R1,
                                 const Vector2d &z2, const Matrix2d &R2,
                                 Vector2d &x_fuse, Matrix2d &P_fuse)
{
    Matrix2d R1_inv, R2_inv;
    if (!matrixInverse2x2(R1, R1_inv))
        return false;
    if (!matrixInverse2x2(R2, R2_inv))
        return false;

    // WLS核心计算
    const Matrix2d P_fuse_inv = R1_inv + R2_inv;
    if (!matrixInverse2x2(P_fuse_inv, P_fuse))
        return false;
    x_fuse = P_fuse * (R1_inv * z1 + R2_inv * z2);

    // std::cout << "x_fuse: " << x_fuse(0) << "  " << x_fuse(1) << std::endl;
    // std::cout << "R_fuse: " << P_fuse(0, 0) << "  " << P_fuse(1, 1) << std::endl;
    return true;
}

// 执行融合流程
bool FusionPCDTool::runFusion()
{
    const int point_num = yr_.size();
    if (point_num == 0)
    {
        LOG(ERROR) << "[ERROR] 无有效观测点，融合终止" << std::endl;
        return false;
    }

    // 初始化融合结果容器
    z_fuse_all_.resize(0);
    P_fuse_all_.resize(0);

    // 逐点融合
    for (int i = 0; i < point_num; ++i)
    {
        // 第一步：WLS静态融合
        Vector2d z_fuse;
        Matrix2d R_fuse;
        if (!fuse2dStatic(zr_[i], Rr_[i], zq_[i], Rq_[i], z_fuse, R_fuse))
        {
            LOG(ERROR) << "[WARNING] 第" << i << "个点WLS融合失败，使用第一组数据作为默认值" << std::endl;
            z_fuse = zr_[i];
            R_fuse = Rr_[i];
        }

        // 第二步：卡尔曼滤波更新（融合后与原始数据融合）
        const MatrixXd K_single = Rq_raw_[i] * (Rq_raw_[i] + R_fuse).inverse();
        //std::cout << "K_single: " << K_single(0, 0) << "  " << K_single(1, 1) << K_single(0, 1) << "  " << K_single(1, 0) << std::endl;
        const Vector2d z_single = zq_raw_[i] + K_single * (z_fuse - zq_raw_[i]);
        const Matrix2d P_END = (Matrix2d::Identity() - K_single) * Rq_raw_[i] + K_single * R_fuse;

        // 调试输出（y值接近阈值时）
        if (std::abs(yr_(i) - Y_DEBUG_THRESHOLD) < Y_DEBUG_TOL)
        {
            LOG(INFO) << "\n[DEBUG] 第" << i << "个点（y≈" << Y_DEBUG_THRESHOLD << "）：" << std::endl;
            LOG(INFO) << "R_L:\n"
                      << Rq_[i] << std::endl;
            LOG(INFO) << "R_V:\n"
                      << Rr_[i] << std::endl;
            LOG(INFO) << "R_raw:\n"
                      << Rq_raw_[i] << std::endl;
            LOG(INFO) << "R_fuse:\n"
                      << R_fuse << std::endl;
            LOG(INFO) << "z_fuse:\n"
                      << z_fuse << std::endl;
            LOG(INFO) << "z_single:\n"
                      << z_single << std::endl;
            LOG(INFO) << "P_END:\n"
                      << P_END << std::endl;
        }

        // 保存到全局融合结果
        double t1 = zr_[i](1) - zq_raw_[i](1);
        double t2 = z_single(1) - zq_raw_[i](1);
        if (abs(t1 - t2) > 0.3)
        {
            LOG(INFO) << "reach here" << i;
            z_fuse_all_.push_back(zq_raw_[i]);
            P_fuse_all_.push_back(Rq_raw_[i]);
        }
        else
        {
            z_fuse_all_.push_back(z_single);
            P_fuse_all_.push_back(P_END);
        }
    }

    LOG(INFO) << "所有点融合完成！" << std::endl;
    return true;
}

// 导出融合结果到文本文件
bool FusionPCDTool::exportFusionResult()
{
    std::ofstream ofs(output_path_, std::ios::trunc);
    if (!ofs.is_open())
    {
        LOG(ERROR) << "无法打开输出文件：" << output_path_ << std::endl;
        return false;
    }

    const int point_num = yr_.size();
    for (int i = 0; i < point_num; ++i)
    {
        const Matrix2d P_END = P_fuse_all_[i];
        // 格式化输出（与原始格式保持一致）
        ofs << std::left << std::fixed << std::setprecision(OUTPUT_PRECISION)
            << std::setw(15) << z_fuse_all_[i](0)
            << std::setw(15) << yr_(i)
            << std::setw(15) << z_fuse_all_[i](1)
            << std::setw(15) << P_END(0, 0)
            << std::setw(15) << 0.0
            << std::setw(15) << P_END(0, 1)
            << std::setw(15) << 0.0
            << std::setw(15) << 0.0
            << std::setw(15) << P_END(1, 1)
            << std::endl;
    }

    ofs.close();
    LOG(INFO) << "融合结果已导出到：" << output_path_ << std::endl;
    return true;
}

// Eigen向量转PCD文件
bool FusionPCDTool::eigenVectorToPCD(const std::vector<Vector2d> &zr, const VectorXd &yr, const std::string &pcd_file_path)
{
    const int point_num = zr.size();
    if (yr.size() != point_num)
    {
        LOG(ERROR) << "y向量长度与点数量不匹配（点数量：" << point_num << "，y长度：" << yr.size() << "）" << std::endl;
        return false;
    }

    // 初始化点云
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    cloud->width = point_num;
    cloud->height = 1;
    cloud->is_dense = true;
    cloud->resize(point_num);

    // 填充点云并查找z最小值点
    int min_z_index = 0;
    double min_z_value = zr[0](1);
    for (int i = 0; i < point_num; ++i)
    {
        const double x = zr[i](0);
        const double y = yr(i);
        const double z = zr[i](1);

        cloud->points[i].x = x;
        cloud->points[i].y = y;
        cloud->points[i].z = z;

        // 更新z最小值
        if (z < min_z_value)
        {
            min_z_value = z;
            min_z_index = i;
        }
    }

    // 保存PCD文件
    if (pcl::io::savePCDFileASCII(pcd_file_path, *cloud) == -1)
    {
        LOG(ERROR) << "无法写入PCD文件：" << pcd_file_path << std::endl;
        return false;
    }

    // 输出统计信息
    LOG(INFO) << "\n========================================" << std::endl;
    LOG(INFO) << "PCD文件生成成功！" << std::endl;
    LOG(INFO) << "输出路径：" << pcd_file_path << std::endl;
    LOG(INFO) << "点云数量：" << point_num << std::endl;
    LOG(INFO) << "----------------------------------------" << std::endl;
    LOG(INFO) << "z值最小点信息：" << std::endl;
    LOG(INFO) << "  索引：" << min_z_index << std::endl;
    LOG(INFO) << "  坐标：(" << std::fixed << std::setprecision(6)
              << cloud->points[min_z_index].x << ", "
              << cloud->points[min_z_index].y << ", "
              << cloud->points[min_z_index].z << ")" << std::endl;
    LOG(INFO) << "  最小z值：" << min_z_value << std::endl;
    LOG(INFO) << "========================================\n"
              << std::endl;

    return true;
}

// 生成所有PCD文件
bool FusionPCDTool::generatePCDFiles()
{
    LOG(INFO) << "开始生成PCD文件..." << std::endl;
    if (!eigenVectorToPCD(z_fuse_all_, yr_, pcd_path))
        return false;
    LOG(INFO) << "所有PCD文件生成完成！" << std::endl;
    return true;
}

// 主函数（程序入口）
int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    // 2. 创建工具类实例
    FusionPCDTool fusion_tool(FLAGS_yaml);

    // 3. 执行完整流程
    if (!fusion_tool.readInputData())
        return EXIT_FAILURE;
    if (!fusion_tool.runFusion())
        return EXIT_FAILURE;
    if (!fusion_tool.exportFusionResult())
        return EXIT_FAILURE;
    if (!fusion_tool.generatePCDFiles())
        return EXIT_FAILURE;

    LOG(INFO) << "\n[INFO] 所有流程执行完成！" << std::endl;
    return EXIT_SUCCESS;
}
