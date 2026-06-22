#include "filter.h"

/**
 * @brief 静态融合两组二维坐标观测值（加权最小二乘 WLS）
 * @param z1 第一组观测值 (x,z)
 * @param R1 第一组2x2协方差阵
 * @param z2 第二组观测值 (x,z)
 * @param R2 第二组2x2协方差阵
 * @param x_fuse 输出：融合后的二维坐标
 * @param P_fuse 输出：融合后的2x2协方差阵
 * @return 是否融合成功（协方差阵不可逆时返回false）
 */
bool fuse2dStatic(const Vector2d &z1, const Matrix2d &R1,
                  const Vector2d &z2, const Matrix2d &R2,
                  Vector2d &x_fuse, Matrix2d &P_fuse)
{
    // SVD分解求逆（兼容低版本Eigen）
    Eigen::JacobiSVD<Matrix2d> svd1(R1, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Matrix2d> svd2(R2, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // 检查奇异值（协方差需正定）
    if (svd1.singularValues()(0) < 1e-9 || svd2.singularValues()(0) < 1e-9)
    {
        std::cerr << "Error: 协方差阵不可逆！" << std::endl;
        return false;
    }

    // 手动计算逆矩阵（SVD标准方法）
    Matrix2d R1_inv = svd1.matrixV() * svd1.singularValues().asDiagonal().inverse() * svd1.matrixU().transpose();
    Matrix2d R2_inv = svd2.matrixV() * svd2.singularValues().asDiagonal().inverse() * svd2.matrixU().transpose();

    // 计算融合后协方差和估计值
    Matrix2d P_fuse_inv = R1_inv + R2_inv;
    Eigen::JacobiSVD<Matrix2d> svd_p(P_fuse_inv, Eigen::ComputeFullU | Eigen::ComputeFullV);
    P_fuse = svd_p.matrixV() * svd_p.singularValues().asDiagonal().inverse() * svd_p.matrixU().transpose();

    x_fuse = P_fuse * (R1_inv * z1 + R2_inv * z2);

    return true;
}

void make_z_R(std::ifstream &ifs, Eigen::VectorXd &z, Eigen::VectorXd &y, Eigen::MatrixXd &R)
{
    std::string line;
    z.resize(0);
    y.resize(0);
    R.resize(0, 0);
    while (getline(ifs, line))
    {
        if (line.empty())
        {
            continue;
        }
        std::istringstream iss(line);
        std::string token;
        std::vector<double> data;
        while (iss >> token)
        {
            data.push_back(std::stod(token));
        }
        if (data.size() == 9)
        {
            z.conservativeResize(z.size() + 2);
            y.conservativeResize(y.size() + 1);
            R.conservativeResize(R.rows() + 2, R.cols() + 2);
            z(z.size() - 2) = data[0];
            z(z.size() - 1) = data[2];
            y(y.size() - 1) = data[1];
            for (size_t i = R.rows()-2; i < R.rows(); i++)
            {
                for (size_t j = 0; j < R.rows()-2; j++)
                {
                    R(i, j) = 0;
                    R(j, i) = 0;
                }
            }
            R(R.rows() - 2, R.cols() - 2) = data[3];
            R(R.rows() - 1, R.cols() - 2) = 0;
            R(R.rows() - 2, R.cols() - 1) = 0;
            R(R.rows() - 1, R.cols() - 1) = data[8];
        }
    }
    ifs.close();
}


bool eigenVectorToPCD(const Eigen::VectorXd& zr, 
                      const Eigen::VectorXd& yr, 
                      const std::string& pcd_file_path)
{
    // 1. 检查zr长度是否为偶数（x/z成对）
    if (zr.size() % 2 != 0)
    {
        std::cerr << "错误：zr向量长度必须为偶数（x/z成对存储）！当前长度：" << zr.size() << std::endl;
        return false;
    }

    // 2. 计算点云数量并检查yr长度是否匹配
    const int point_num = zr.size() / 2;
    if (yr.size() != point_num)
    {
        std::cerr << "错误：yr向量长度与zr中的点数量不匹配！" << std::endl;
        std::cerr << "zr中的点数量：" << point_num << " | yr长度：" << yr.size() << std::endl;
        return false;
    }

    // 3. 创建点云对象（XYZ类型）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->width = point_num;    // 点云宽度（点数量）
    cloud->height = 1;           // 无序点云（一维）
    cloud->is_dense = true;      // 无无效点
    cloud->resize(point_num);    // 分配内存

    // ========== 新增：初始化z最小值相关变量 ==========
    int min_z_index = 0;                // 记录z最小值点的索引
    double min_z_value = zr(1);         // 初始化为第一个点的z值（zr[1]）
    pcl::PointXYZ min_z_point;          // 存储z最小值点的完整坐标

    // 4. 遍历向量，填充点云数据 + 查找z最小值点
    for (int i = 0; i < point_num; ++i)
    {
        // zr：2*i -> x坐标，2*i+1 -> z坐标；yr：i -> y坐标
        double x = zr(2 * i);
        double y = yr(i);
        double z = zr(2 * i + 1);

        // 填充点云
        cloud->points[i].x = x;
        cloud->points[i].y = y;
        cloud->points[i].z = z;

        // ========== 新增：更新z最小值点 ==========
        if (z < min_z_value)
        {
            min_z_value = z;    // 更新最小值
            min_z_index = i;    // 更新最小值点索引
            min_z_point.x = x;  // 记录最小值点的x
            min_z_point.y = y;  // 记录最小值点的y
            min_z_point.z = z;  // 记录最小值点的z
        }
    }

    // 5. 写入PCD文件（ASCII格式，便于查看）
    if (pcl::io::savePCDFileASCII(pcd_file_path, *cloud) == -1)
    {
        std::cerr << "错误：无法写入PCD文件到路径：" << pcd_file_path << std::endl;
        return false;
    }

    // 6. 输出成功信息 + 新增：输出z最小值点信息
    std::cout << "========================================" << std::endl;
    std::cout << "成功生成PCD文件！" << std::endl;
    std::cout << "点云数量：" << point_num << std::endl;
    std::cout << "输出路径：" << pcd_file_path << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "z值最小的点信息：" << std::endl;
    std::cout << "  点索引：" << min_z_index << std::endl;
    std::cout << "  坐标 (x, y, z)：(" 
              << min_z_point.x << ", " 
              << min_z_point.y << ", " 
              << min_z_point.z << ")" << std::endl;
    std::cout << "  最小z值：" << min_z_value << std::endl;
    std::cout << "========================================" << std::endl;

    return true;
}


int main()
{
    std::ifstream ifsr("/home/gct/LC_Modelcurve/data/filter_test/1747R.txt");
    std::ifstream ifsq("/home/gct/LC_Modelcurve/data/filter_test/1747Q.txt");
    std::ifstream ifsq_raw("/home/gct/LC_Modelcurve/data/filter_test/1742R.txt");
    if (!ifsr.is_open() || !ifsq.is_open() || !ifsq_raw.is_open())
    {
        std::cout << "error to open R file";
        return 0;
    }
    Eigen::VectorXd zr;
    Eigen::MatrixXd Rr;
    Eigen::VectorXd zq;
    Eigen::MatrixXd Rq;
    Eigen::VectorXd zq_raw;
    Eigen::MatrixXd Rq_raw;
    Eigen::VectorXd yr;
    Eigen::VectorXd yq;
    Eigen::VectorXd yq_raw;
    make_z_R(ifsr, zr, yr, Rr);
    make_z_R(ifsq, zq, yq, Rq);
    make_z_R(ifsq_raw, zq_raw, yq_raw, Rq_raw);
    if (zr.size() != zq.size() || zr.size() != Rr.cols() || Rr.cols() != Rr.rows() || Rq.cols() != Rq.rows())
    {
        std::cout << "error size" << std::endl;
    }

    Vector2d z1, z2, z_fuse;
    Matrix2d R1, R2, R_fuse;
    Eigen::VectorXd z;
    Eigen::MatrixXd R;
    z.resize(0);
    R.resize(0, 0);
    for (size_t i = 0; i < zr.size() / 2; i++)
    {
        z1.x() = zr(2*i);
        z1.y() = zr(2*i + 1);
        z2.x() = zq(2*i);
        z2.y() = zq(2*i + 1);
        R1(0, 0) = Rr(2*i, 2*i);
        R1(0, 1) = Rr(2*i, 2*i + 1);
        R1(1, 0) = Rr(2*i + 1, 2*i);
        R1(1, 1) = Rr(2*i + 1, 2*i + 1);
        R2(0, 0) = Rq(2*i, 2*i);
        R2(0, 1) = Rq(2*i, 2*i + 1);
        R2(1, 0) = Rq(2*i + 1, 2*i);
        R2(1, 1) = Rq(2*i + 1, 2*i + 1);
        fuse2dStatic(z1, R1, z2, R2, z_fuse, R_fuse);

        z.conservativeResize(z.size() + 2);
        R.conservativeResize(R.rows() + 2, R.cols() + 2);
        z(z.size() - 2) = z_fuse.x();
        z(z.size() - 1) = z_fuse.y();
        R(R.rows() - 2, R.cols() - 2) = R_fuse(0,0);
        R(R.rows() - 1, R.cols() - 2) = R_fuse(1,0);
        R(R.rows() - 2, R.cols() - 1) = R_fuse(0,1);
        R(R.rows() - 1, R.cols() - 1) = R_fuse(1,1);
         //   std::cout << "R: " << R_fuse <<std::endl;
    }
    std::cout << "Z: " << zq_raw - z <<std::endl;
    //std::cout << "Zq_raw: " << zq_raw <<std::endl;

    // std::cout << "R_size: " << R.cols() <<std::endl;
    // std::cout << "z_size: " << z.size() <<std::endl;
    // std::cout << "raw_r_size: " << Rq_raw.cols() <<std::endl;
    // std::cout << "raw_z_szie: " << zq_raw.size() <<std::endl;

    //update
    Eigen::MatrixXd K = Rq_raw * ((Rq_raw + R).inverse());
    Eigen::VectorXd z_end = zq_raw + K * (z - zq_raw);
    // std::cout << "Z: " << z - zq_raw <<std::endl;
    std::cout << "R: " << Rq_raw <<std::endl;
    // std::cout << "Z: " << yr <<std::endl;
    // std::cout << "Z: " << yq <<std::endl;
    // std::cout << "Z: " << yq_raw <<std::endl;

    std::string d1747L_path = "/home/gct/LC_Modelcurve/data/filter_test/1747L.pcd";
    std::string d1747V_path = "/home/gct/LC_Modelcurve/data/filter_test/1747V.pcd";
    std::string d1747VL_path = "/home/gct/LC_Modelcurve/data/filter_test/1747VL.pcd";
    if (!eigenVectorToPCD(zq, yq ,d1747L_path))
    {
        return -1;
    }
        if (!eigenVectorToPCD(zr, yr ,d1747V_path))
    {
        return -1;
    }
        if (!eigenVectorToPCD(z_end, yr ,d1747VL_path))
    {
        return -1;
    }








    return 0;
}
