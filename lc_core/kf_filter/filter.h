#ifndef FILTER_H
#define FILTER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <yaml-cpp/yaml.h>

// 类型别名定义（统一命名，增强可读性）
using Eigen::Vector2d;
using Eigen::Matrix2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using pcl::PointCloud;
using pcl::PointXYZ;

/**
 * @brief 二维观测数据融合与点云生成工具类
 * 功能：读取观测数据、WLS静态融合、卡尔曼滤波更新、PCD文件导出
 */
class FusionPCDTool {
public:
    // 构造函数（默认）
    FusionPCDTool();

    // 带参数构造函数（直接传入文件路径）
    FusionPCDTool(const std::string &yaml_file);

    // 析构函数
    ~FusionPCDTool() = default;

    /**
     * @brief 读取所有输入文件的观测数据和协方差矩阵
     * @return 是否读取成功
     * @throw std::runtime_error 读取失败时抛出异常
     */
    bool readInputData();

    /**
     * @brief 执行数据融合（WLS静态融合 + 卡尔曼滤波更新）
     * @return 是否融合成功
     */
    bool runFusion();

    /**
     * @brief 导出融合结果到文本文件
     * @return 是否导出成功
     */
    bool exportFusionResult();

    /**
     * @brief 生成PCD点云文件（原始数据+融合数据）
     * @return 是否生成成功
     */
    bool generatePCDFiles();

private:
    // 私有常量（魔法数字集中管理）
    static constexpr double EPS = 1e-9;                // 奇异值阈值（矩阵可逆判断）
    static constexpr double MIN_COV_VALUE = 1e-6;      // 协方差最小有效值
    static constexpr int DATA_COLUMN_COUNT = 9;        // 输入文件每行有效列数
    static constexpr int OUTPUT_PRECISION = 10;        // 输出数据精度
    static constexpr double Y_DEBUG_THRESHOLD = -2.55; // 调试输出的y值阈值
    static constexpr double Y_DEBUG_TOL = 0.01;        // y值调试容差

    // 输入输出路径
    std::string input_path_R_;    // 第一组观测数据路径（R相关）
    std::string input_path_Q_;    // 第二组观测数据路径（Q相关）
    std::string input_path_Q_raw_;// 原始观测数据路径（Q_raw相关）
    std::string output_path_;     // 融合结果输出路径
    std::string pcd_path;

    // 输入数据存储
    std::vector<Vector2d> zr_;   // 第一组观测向量 (x0,z0,x1,z1,...)
    std::vector<Vector2d> zq_;   // 第二组观测向量 (x0,z0,x1,z1,...)
    std::vector<Vector2d> zq_raw_;// 原始观测向量 (x0,z0,x1,z1,...)
    VectorXd yr_;   // 第一组y坐标向量
    VectorXd yq_;   // 第二组y坐标向量
    VectorXd yq_raw_;// 原始y坐标向量
    std::vector<Matrix2d> Rr_;   // 第一组协方差矩阵（2N×2N）
    std::vector<Matrix2d> Rq_;   // 第二组协方差矩阵（2N×2N）
    std::vector<Matrix2d> Rq_raw_;// 原始协方差矩阵（2N×2N）

    // 融合结果存储
    std::vector<Vector2d> z_fuse_all_;  // 所有点的融合后观测向量
    std::vector<Matrix2d> P_fuse_all_;  // 所有点的融合后协方差矩阵

    /**
     * @brief 从单个文件读取观测数据和协方差矩阵
     * @param ifs 输入文件流（已打开）
     * @param z 输出：观测向量
     * @param y 输出：y坐标向量
     * @param R 输出：协方差矩阵
     */
    void readSingleFile(std::ifstream& ifs, std::vector<Vector2d>& z, VectorXd& y, std::vector<Matrix2d>& R);

    /**
     * @brief SVD分解求2x2矩阵的逆（兼容低版本Eigen）
     * @param mat 输入2x2矩阵
     * @param inv_mat 输出：逆矩阵
     * @return 是否求逆成功
     */
    bool matrixInverse2x2(const Matrix2d& mat, Matrix2d& inv_mat);

    /**
     * @brief 静态融合两组二维观测值（加权最小二乘 WLS）
     * @param z1 第一组观测值
     * @param R1 第一组协方差矩阵
     * @param z2 第二组观测值
     * @param R2 第二组协方差矩阵
     * @param x_fuse 输出：融合后观测值
     * @param P_fuse 输出：融合后协方差矩阵
     * @return 是否融合成功
     */
    bool fuse2dStatic(const Vector2d& z1, const Matrix2d& R1,
                      const Vector2d& z2, const Matrix2d& R2,
                      Vector2d& x_fuse, Matrix2d& P_fuse);

    /**
     * @brief 将Eigen向量转换为PCD点云文件
     * @param zr 观测向量 (x0,z0,x1,z1,...)
     * @param yr y坐标向量
     * @param pcd_file_path 输出路径
     * @return 是否转换成功
     */
    bool eigenVectorToPCD(const std::vector<Vector2d>& zr, const VectorXd& yr, const std::string& pcd_file_path);
};

#endif // FILTER_H
