#include "catenary.h"
#include "glog/logging.h"
#include "ceres/ceres.h"
#include "catenary_init_factor.h"
#include "catenary_p2p_factor.h"
#include "catenary_ep_factor.h"

using namespace lc_core;

void Catenary::fitTransmissionModel(std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &end_point, std::vector<double> &cov_t, std::vector<double> &cov_f, std::vector<double> &para)
{
    is_visual = false;
    first_time = false;
    ceres::Problem problem_xy;
    ceres::Solver::Options options_xy;
    options_xy.linear_solver_type = ceres::DENSE_QR;
    options_xy.minimizer_progress_to_stdout = true;
    cov_t.resize(6);
    cov_f.resize(6);
    para.resize(0);
    // options.max_num_iterations = 10;
    options_xy.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    for (size_t i = 0; i < points.size(); ++i)
    {
        ceres::CostFunction *cost_function = new CatenaryInitFactor_xy(points[i].x(), points[i].y());
        problem_xy.AddResidualBlock(cost_function, nullptr, &T1, &T2, &T3);
    }
    ceres::CostFunction *cost_functionxy = new CatenaryEpFactorxy(end_point(0), end_point(1), end_point(2));
    problem_xy.AddResidualBlock(cost_functionxy, nullptr, &T1, &T2, &T3);
    ceres::Solver::Summary summary_xy;
    ceres::Solve(options_xy, &problem_xy, &summary_xy);
    // cal Qxy
    double residual_sumxy = 0;
    for (size_t i = 0; i < points.size(); i++)
    {
        double tmp_residual = T1 + T2 / 10 * points[i].y() + T3 / 1000 * points[i].y() * points[i].y() - points[i].x();
        residual_sumxy += pow(tmp_residual, 2);
    }
    double end_residual_xy = T1 + T2 / 10 * end_point(1) + T3 / 1000 * end_point(1) * end_point(1) - end_point(0);
    residual_sumxy += 1e4 * pow(end_residual_xy, 2);
    double sigma_xy = residual_sumxy / (points.size() + 1 - 3);
    ceres::Covariance::Options cov_options;
    ceres::Covariance cov1(cov_options);
    std::vector<std::pair<const double *, const double *>> cov_blocks1 = {
        {&T1, &T1}, {&T1, &T2}, {&T1, &T3}, {&T2, &T2}, {&T2, &T3}, {&T3, &T3}};
    CHECK(cov1.Compute(cov_blocks1, &problem_xy));
    // double cov_T1_T1, cov_T1_T2, cov_T1_T3, cov_T2_T2, cov_T2_T3, cov_T3_T3;
    cov1.GetCovarianceBlock(&T1, &T1, &cov_t[0]);
    cov_t[0] *= sigma_xy;
    cov1.GetCovarianceBlock(&T1, &T2, &cov_t[1]);
    cov_t[1] *= sigma_xy;
    cov1.GetCovarianceBlock(&T1, &T3, &cov_t[2]);
    cov_t[2] *= sigma_xy;
    cov1.GetCovarianceBlock(&T2, &T2, &cov_t[3]);
    cov_t[3] *= sigma_xy;
    cov1.GetCovarianceBlock(&T2, &T3, &cov_t[4]);
    cov_t[4] *= sigma_xy;
    cov1.GetCovarianceBlock(&T3, &T3, &cov_t[5]);
    cov_t[5] *= sigma_xy;

    ceres::Problem problem_xz;
    ceres::Solver::Options options_xz;
    options_xz.linear_solver_type = ceres::DENSE_QR;
    options_xz.minimizer_progress_to_stdout = true;
    // options.max_num_iterations = 10;
    options_xz.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    for (size_t i = 0; i < points.size(); ++i)
    {
        ceres::CostFunction *cost_function = new CatenaryInitFactor_xz(points[i].x(), points[i].z());
        problem_xz.AddResidualBlock(cost_function, nullptr, &F1, &F2, &F3);
    }
    ceres::CostFunction *cost_functionxz = new CatenaryEpFactorxz(end_point(0), end_point(1), end_point(2));
    problem_xz.AddResidualBlock(cost_functionxz, nullptr, &F1, &F2, &F3);
    ceres::Solver::Summary summary_xz;
    ceres::Solve(options_xz, &problem_xz, &summary_xz);

    // cal Qxz
    double residual_sumxz = 0;
    for (size_t i = 0; i < points.size(); i++)
    {
        double tmp_residual = F1 / 10000 * points[i].x() * points[i].x() + F2 + F3 / 10 * points[i].x() - points[i].z();
        residual_sumxz += pow(tmp_residual, 2);
    }
    double end_residual_xz = F1 / 10000 * end_point(0) * end_point(0) + F2 + F3 / 10 * end_point(0) - end_point(2);
    residual_sumxz += 1e4 * pow(end_residual_xz, 2);
    double sigma_xz = residual_sumxz / (points.size() + 1 - 3);
    ceres::Covariance::Options cov_options_xz;
    ceres::Covariance cov2(cov_options_xz);
    std::vector<std::pair<const double *, const double *>> cov_blocks2 = {
        {&F1, &F1}, {&F1, &F2}, {&F1, &F3}, {&F2, &F2}, {&F2, &F3}, {&F3, &F3}};
    CHECK(cov2.Compute(cov_blocks2, &problem_xz));
    // double cov_F1_F1, cov_F1_F2, cov_F1_F3, cov_F2_F2, cov_F2_F3, cov_F3_F3;
    cov2.GetCovarianceBlock(&F1, &F1, &cov_f[0]);
    cov_f[0] *= sigma_xz;
    cov2.GetCovarianceBlock(&F1, &F2, &cov_f[1]);
    cov_f[1] *= sigma_xz;
    cov2.GetCovarianceBlock(&F1, &F3, &cov_f[2]);
    cov_f[2] *= sigma_xz;
    cov2.GetCovarianceBlock(&F2, &F2, &cov_f[3]);
    cov_f[3] *= sigma_xz;
    cov2.GetCovarianceBlock(&F2, &F3, &cov_f[4]);
    cov_f[4] *= sigma_xz;
    cov2.GetCovarianceBlock(&F3, &F3, &cov_f[5]);
    cov_f[5] *= sigma_xz;

    T2 = T2 / 10;
    T3 = T3 / 1000;
    F1 = F1 / 10000;
    F3 = F3 / 10;

    cov_t[1] /= 10;
    cov_t[2] /= 1000;
    cov_t[4] /= 10000;
    cov_t[3] /= 100;
    cov_t[5] /= 1000000;
    cov_f[0] /= 100000000;
    cov_f[1] /= 10000;
    cov_f[2] /= 100000;
    cov_f[4] /= 10;
    cov_f[5] /= 100;
    para.push_back(T1);
    para.push_back(T2);
    para.push_back(T3);
    para.push_back(F1);
    para.push_back(F2);
    para.push_back(F3);
    para_.push_back(T1);
    para_.push_back(T2);
    para_.push_back(T3);
    para_.push_back(F1);
    para_.push_back(F2);
    para_.push_back(F3);

    LOG(INFO) << "T1: " << T1 << " T2: " << T2 << "T3: " << T3;
    LOG(INFO) << "F1: " << F1 << " F2: " << F2 << "F3: " << F3;
}

Eigen::Vector3d Catenary::generateSinglePoint(const double &y)
{
    double x = T1 + T2 * y + T3 * y * y;
    double z = F1 * x * x + F2 + F3 * x;
    return Eigen::Vector3d(x, y, z);
}


void Catenary::optimizeTransmissionModel(const P2PMatchResult &points, const OptimizationInput &input)
{
    ceres::Problem problem;
    ceres::Solver::Options options;
    // ceres::LossFunction *loss_function = new ceres::HuberLoss(5.0);
    ceres::LossFunction *loss_function = nullptr;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 8;
    options.trust_region_strategy_type = ceres::DOGLEG;
    // options.num_threads = 8;

    for (size_t i = 0; i < points.size(); i++)
    {
        ceres::CostFunction *cost_function = new CatenaryP2PFactorA(points[i], input.xySamples[i], Trans(input.R, input.t), input.cam);
        problem.AddResidualBlock(cost_function, NULL, &T1, &T2, &T3, &F1, &F2, &F3);
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Eigen::Vector3d p = generateSinglePoint(input.end_point(1));
    double deta_x = (p(0) - input.end_point(0)) / input.end_point(0);
    if ((deta_x > 0.1 || deta_x < -0.02)&& !first_time)
    {
        is_visual = false;
        T1 = para_[0];
        T2 = para_[1];
        T3 = para_[2];
        F1 = para_[3];
        F2 = para_[4];
        F3 = para_[5];
    }
    else
    {
        is_visual = true;

        double residual = 0;
        double index_residual = 0;
        double sqrt_info = 0;

        for (size_t i = 0; i < input.xySamples.size(); i++)
        {
            Eigen::Matrix<double, 3, 1> pLidar;
            double x = T1 + T2 * input.xySamples[i] + T3 * input.xySamples[i] * input.xySamples[i];
            pLidar << x, input.xySamples[i], F1 * x * x + F2 + F3 * x;
            Eigen::Matrix<double, 3, 1> pCam = input.R.cast<double>() * pLidar + input.t.cast<double>();
            Eigen::Matrix<double, 2, 1> pImg;
            input.cam->spaceToPlane(pCam, pImg);
            double dist = ceres::sqrt((pImg(0) - points[i].x) * (pImg(0) - points[i].x) + (pImg(1) - points[i].y) * (pImg(1) - points[i].y));
            residual += pow(dist, 2);
        }
        double sigma_sq = residual / (points.size() - 6);
        ceres::Covariance::Options cov_options;
        ceres::Covariance covariance(cov_options);

        // 定义需要计算协方差的参数块（6个参数）
        std::vector<const double *> param_blocks = {&T1, &T2, &T3, &F1, &F2, &F3};

        // 计算协方差（核心：基于优化后的Problem）
        CHECK(covariance.Compute(param_blocks, &problem))
            << "6参数协方差计算失败！请检查linear_solver_type是否为DENSE_QR/DENSE_NORMAL_CHOLESKY";

        Eigen::Matrix<double, 6, 6> Sigma_6d;
        // 提取所有协方差块并缩放（乘以σ²得到真实协方差）
        covariance.GetCovarianceBlock(&T1, &T1, &Sigma_6d(0, 0));
        Sigma_6d(0, 0) *= sigma_sq;
        covariance.GetCovarianceBlock(&T1, &T2, &Sigma_6d(0, 1));
        Sigma_6d(0, 1) *= sigma_sq;
        covariance.GetCovarianceBlock(&T1, &T3, &Sigma_6d(0, 2));
        Sigma_6d(0, 2) *= sigma_sq;
        covariance.GetCovarianceBlock(&T1, &F1, &Sigma_6d(0, 3));
        Sigma_6d(0, 3) *= sigma_sq;
        covariance.GetCovarianceBlock(&T1, &F2, &Sigma_6d(0, 4));
        Sigma_6d(0, 4) *= sigma_sq;
        covariance.GetCovarianceBlock(&T1, &F3, &Sigma_6d(0, 5));
        Sigma_6d(0, 5) *= sigma_sq;

        covariance.GetCovarianceBlock(&T2, &T2, &Sigma_6d(1, 1));
        Sigma_6d(1, 1) *= sigma_sq;
        covariance.GetCovarianceBlock(&T2, &T3, &Sigma_6d(1, 2));
        Sigma_6d(1, 2) *= sigma_sq;
        covariance.GetCovarianceBlock(&T2, &F1, &Sigma_6d(1, 3));
        Sigma_6d(1, 3) *= sigma_sq;
        covariance.GetCovarianceBlock(&T2, &F2, &Sigma_6d(1, 4));
        Sigma_6d(1, 4) *= sigma_sq;
        covariance.GetCovarianceBlock(&T2, &F3, &Sigma_6d(1, 5));
        Sigma_6d(1, 5) *= sigma_sq;

        covariance.GetCovarianceBlock(&T3, &T3, &Sigma_6d(2, 2));
        Sigma_6d(2, 2) *= sigma_sq;
        covariance.GetCovarianceBlock(&T3, &F1, &Sigma_6d(2, 3));
        Sigma_6d(2, 3) *= sigma_sq;
        covariance.GetCovarianceBlock(&T3, &F2, &Sigma_6d(2, 4));
        Sigma_6d(2, 4) *= sigma_sq;
        covariance.GetCovarianceBlock(&T3, &F3, &Sigma_6d(2, 5));
        Sigma_6d(2, 5) *= sigma_sq;

        covariance.GetCovarianceBlock(&F1, &F1, &Sigma_6d(3, 3));
        Sigma_6d(3, 3) *= sigma_sq;
        covariance.GetCovarianceBlock(&F1, &F2, &Sigma_6d(3, 4));
        Sigma_6d(3, 4) *= sigma_sq;
        covariance.GetCovarianceBlock(&F1, &F3, &Sigma_6d(3, 5));
        Sigma_6d(3, 5) *= sigma_sq;

        covariance.GetCovarianceBlock(&F2, &F2, &Sigma_6d(4, 4));
        Sigma_6d(4, 4) *= sigma_sq;
        covariance.GetCovarianceBlock(&F2, &F3, &Sigma_6d(4, 5));
        Sigma_6d(4, 5) *= sigma_sq;

        covariance.GetCovarianceBlock(&F3, &F3, &Sigma_6d(5, 5));
        Sigma_6d(5, 5) *= sigma_sq;

        // 对称填充下三角（协方差矩阵是对称的）
        for (int i = 1; i < 6; i++)
        {
            for (int j = 0; j < i; j++)
            {
                Sigma_6d(i, j) = Sigma_6d(j, i);
            }
        }

        // ========== 第五步：输出协方差结果（可选） ==========
        LOG(INFO) << "6参数协方差矩阵：";
        std::vector<std::string> param_names = {"T1", "T2", "T3", "F1", "F2", "F3"};
        for (int i = 0; i < 6; i++)
        {
            std::string row_str = "";
            for (int j = 0; j < 6; j++)
            {
                row_str += param_names[i] + "-" + param_names[j] + ": " + std::to_string(Sigma_6d(i, j)) + "\t";
            }
            LOG(INFO) << row_str;
        }

        // ========== 扩展：提取单个参数的方差（用于快速查看） ==========
        double var_T1 = Sigma_6d(0, 0); // T1的方差
        double var_T2 = Sigma_6d(1, 1); // T2的方差
        double var_T3 = Sigma_6d(2, 2); // T3的方差
        double var_F1 = Sigma_6d(3, 3); // F1的方差
        double var_F2 = Sigma_6d(4, 4); // F2的方差
        double var_F3 = Sigma_6d(5, 5); // F3的方差
        LOG(INFO) << "各参数方差：T1=" << var_T1 << ", T2=" << var_T2 << ", T3=" << var_T3
                  << ", F1=" << var_F1 << ", F2=" << var_F2 << ", F3=" << var_F3;

        LOG(INFO) << "After optimization: ";
        LOG(INFO) << "T1: " << T1 << " T2: " << T2 << "T3: " << T3;
        LOG(INFO) << "F1: " << F1 << " F2: " << F2 << "F3: " << F3;
        std::vector<double> cov_t = {Sigma_6d(0, 0), Sigma_6d(0, 1), Sigma_6d(0, 2), Sigma_6d(1, 1), Sigma_6d(1, 2), Sigma_6d(2, 2)};
        std::vector<double> cov_f = {Sigma_6d(3, 3), Sigma_6d(3, 4), Sigma_6d(3, 5), Sigma_6d(4, 4), Sigma_6d(4, 5), Sigma_6d(5, 5)};
        std::vector<double> para = {T1, T2, T3, F1, F2, F3};
        var_para.push_back(cov_t);
        var_para.push_back(cov_f);
        var_para.push_back(para);
    }
}

void Catenary::optimizeTransmissionModelDark(const Eigen::Vector3d &end_point)
{
}

std::vector<std::vector<double>> Catenary::getpara()
{
    if (var_para.size() < 3)
    {
        LOG(ERROR) << "There is no enough visual para";
        exit(EXIT_FAILURE);
    }
    else
    {
        return var_para;
    }
}

bool Catenary::get_is_visual()
{
    return is_visual;
}

bool Catenary::set_first_time()
{
    first_time = true;
    return true;
}