#include "catenary.h"
#include "glog/logging.h"
#include "ceres/ceres.h"
#include "catenary_init_factor.h"
#include "catenary_p2l_factor.h"
#include "catenary_p2p_factor.h"
#include "catenary_ep_factor.h"

using namespace lc_core;

void Catenary::fitTransmissionModel(std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &end_point)
{
    ceres::Problem problem_xy;
    ceres::Solver::Options options_xy;
    options_xy.linear_solver_type = ceres::DENSE_QR;
    options_xy.minimizer_progress_to_stdout = true;
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

    T2 = T2 /10;
    T3 = T3 /1000;
    F1 = F1 /10000;
    F3 = F3 /10;

    LOG(INFO) << "T1: " << T1 << " T2: " << T2 << "T3: " << T3;
    LOG(INFO) << "F1: " << F1 << " F2: " << F2 << "F3: " << F3;
}

Eigen::Vector3d Catenary::generateSinglePoint(const double &y)
{
    double x = T1 + T2 * y + T3 *y *y;
    double z = F1 * x*x + F2 + F3 *x;
    return Eigen::Vector3d(x, y, z);
}

void Catenary::optimizeTransmissionModel(const P2LMatchResult &lines, const OptimizationInput &input, int y_optimize)
{
    ceres::Problem problem;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 10;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.num_threads = 8;

#if 0
    for (size_t i = 0; i < lines.size(); i++)
    {
        ceres::CostFunction *cost_function = CatenaryP2LFactor::Create(lines[i], input.xSamples[i], Trans(input.R, input.t), input.cam);
        problem.AddResidualBlock(cost_function, nullptr, &c_, &c1_, &c2_, &k_, &m_);
    }
    ceres::CostFunction *cost_function = CatenaryEpFactor::Create(input.end_point(0), input.end_point(1), input.end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &c_, &c1_, &c2_, &k_, &m_);

#else
    for (size_t i = 0; i < lines.size(); i++)
    {
        ceres::CostFunction *cost_function = new CatenaryP2LFactorA(lines[i], input.xySamples[i], Trans(input.R, input.t), input.cam);
        problem.AddResidualBlock(cost_function, nullptr, &F1, &F2, &F3);
    }
    ceres::CostFunction *cost_function = new CatenaryEpFactorA(input.end_point(0), input.end_point(1), input.end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &T1, &T2, &T3);
#endif

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "T1: " << T1 << " T2: " << T2 << "T3: " << T3;
    LOG(INFO) << "F1: " << F1 << " F2: " << F2 << "F3: " << F3;
}

void Catenary::optimizeTransmissionModel(const P2PMatchResult &points, const OptimizationInput &input, int y_optimize, int time)
{
    ceres::Problem problem;
    ceres::Solver::Options options;
    // ceres::LossFunction *loss_function = new ceres::HuberLoss(5.0);
    ceres::LossFunction *loss_function = nullptr;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = time == 1 ? 8 : 5;
    options.trust_region_strategy_type = ceres::DOGLEG;
    // options.num_threads = 8;

#if 0
    for (size_t i = 0; i < points.size(); i++)
    {
        ceres::CostFunction *cost_function = CatenaryP2PFactor::Create(points[i], input.xSamples[i], Trans(input.R, input.t), input.cam, static_cast<WeightType>(time));
        problem.AddResidualBlock(cost_function, loss_function, &c_, &c1_, &c2_, &k_, &m_);
    }
    ceres::CostFunction *cost_function = CatenaryEpFactor::Create(input.end_point(0), input.end_point(1), input.end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &c_, &c1_, &c2_, &k_, &m_);
#else
    for (size_t i = 0; i < points.size(); i++)
    {
        ceres::CostFunction *cost_function = new CatenaryP2PFactorA(points[i], input.xySamples[i], Trans(input.R, input.t), input.cam, static_cast<WeightType>(time));
        problem.AddResidualBlock(cost_function, NULL, &T1, &T2, &T3, &F1, &F2, &F3);
    }
    // ceres::CostFunction *cost_function = new CatenaryEpFactorA(input.end_point(0), input.end_point(1), input.end_point(2));
    // problem.AddResidualBlock(cost_function, nullptr, &T1, &T2, &T3, &F1, &F2, &F3);
#endif

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "T1: " << T1 << " T2: " << T2 << "T3: " << T3;
    LOG(INFO) << "F1: " << F1 << " F2: " << F2 << "F3: " << F3;
}

void Catenary::optimizeTransmissionModelDark(const Eigen::Vector3d &end_point)
{
    ceres::Problem problem;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 10;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.num_threads = 8;

    // problem.AddParameterBlock(&c_, 1);
    // problem.AddParameterBlock(&c1_, 1);
    // problem.AddParameterBlock(&c2_, 1);
    // problem.AddParameterBlock(&k_, 1);
    // problem.AddParameterBlock(&m_, 1);

    // problem.SetParameterBlockConstant(&c_);
#if 0
    ceres::CostFunction *cost_function = CatenaryEpFactor::Create(end_point(0), end_point(1), end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &c_, &c1_, &c2_, &k_, &m_);
#else
    ceres::CostFunction *cost_function = new CatenaryEpFactorA(end_point(0), end_point(1), end_point(2));
    problem.AddResidualBlock(cost_function, nullptr,&T1, &T2, &T3, &F1, &F2, &F3);
#endif

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    // LOG(INFO) << "c: " << c_ << " c1: " << c1_ << " c2: " << c2_;
    // LOG(INFO) << "k: " << k_ << " m: " << m_;
}