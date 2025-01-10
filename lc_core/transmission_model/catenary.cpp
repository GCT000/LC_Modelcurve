#include "catenary.h"
#include "glog/logging.h"
#include "ceres/ceres.h"
#include "catenary_init_factor.h"
#include "catenary_p2l_factor.h"
#include "catenary_p2p_factor.h"

using namespace lc_core;

void Catenary::fitTransmissionModel(std::vector<Eigen::Vector3d> &points)
{
    // ransac fit line
    auto [k, m] = ransacFitLine(points);
    k_ = k;
    m_ = m;

    // 2-degree tylor
    ceres::Problem problem;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    // options.max_num_iterations = 10;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    
    for (size_t i = 0; i < points.size(); ++i) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CatenaryInitFactor, 1, 1, 1, 1>(
                new CatenaryInitFactor(points[i].x(), points[i].z())),
            nullptr, &c_, &c1_, &c2_);
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    LOG(INFO) << "c: " << c_ << " c1: " << c1_ << " c2: " << c2_;
    LOG(INFO) << "k: " << k_ << " m: " << m_;
}

Eigen::Vector3d Catenary::generateSinglePoint(const double &x)
{
    double y = k_ * x + m_;
    double z = c_ * cosh((x + c1_) / c_) + c2_ ;
    return Eigen::Vector3d(x, y, z);
}

void Catenary::optimizeTransmissionModel(const P2LMatchResult& lines, const OptimizationInput& input, int y_optimize)
{
    ceres::Problem problem;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 10;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.num_threads = 8;
    
    problem.AddParameterBlock(&k_, 1);
    problem.AddParameterBlock(&m_, 1);

    if (!y_optimize) {
        problem.SetParameterBlockConstant(&k_);
        problem.SetParameterBlockConstant(&m_);
    }
    
    for (size_t i = 0; i < lines.size(); i++) {
        ceres::CostFunction* cost_function = CatenaryP2LFactor::Create(lines[i], input.xSamples[i], Trans(input.R, input.t), input.cam);
        problem.AddResidualBlock(cost_function, nullptr, &c_, &c1_, &c2_, &k_, &m_);
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "c: " << c_ << " c1: " << c1_ << " c2: " << c2_;
    LOG(INFO) << "k: " << k_ << " m: " << m_;
}

void Catenary::optimizeTransmissionModel(const P2PMatchResult& points, const OptimizationInput& input, int y_optimize, int time)
{
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(5.0);
    // ceres::LossFunction *loss_function = nullptr;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 8;
    options.trust_region_strategy_type = ceres::DOGLEG;
    // options.num_threads = 8;

    problem.AddParameterBlock(&k_, 1);
    problem.AddParameterBlock(&m_, 1);

    if (!y_optimize || time == 2) {
        problem.SetParameterBlockConstant(&k_);
        problem.SetParameterBlockConstant(&m_);
    }

    for (size_t i = 0; i < points.size(); i ++) {
        ceres::CostFunction* cost_function = CatenaryP2PFactor::Create(points[i], input.xSamples[i], Trans(input.R, input.t), input.cam, static_cast<WeightType>(time));
        problem.AddResidualBlock(cost_function, loss_function, &c_, &c1_, &c2_, &k_, &m_);
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "c: " << c_ << " c1: " << c1_ << " c2: " << c2_;
    LOG(INFO) << "k: " << k_ << " m: " << m_;
}