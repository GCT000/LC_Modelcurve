#include "straight_line.h"
#include "glog/logging.h"
#include "ceres/ceres.h"
#include "curve_factor.h"
#include "curve_factor_p2p.h"
#include "parabola_ep_factor.h"

using namespace lc_core;

void Parabola::fitTransmissionModel(std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &end_point, std::vector<double> &cov_t, std::vector<double> &cov_f, std::vector<double> &para)
{
    auto [k, m] = ransacFitLine(points);
    k_ = k;
    m_ = m;

    Eigen::MatrixXd A(points.size(), 3);
    Eigen::VectorXd b(points.size());
    for (int i = 0; i < points.size(); ++i)
    {
        A(i, 0) = points[i](0) * points[i](0);
        A(i, 1) = points[i](0);
        A(i, 2) = 1.0;
        b(i) = points[i](2);
    }

    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
    a_ = x(0);
    b_ = x(1);
    c_ = x(2);

    LOG(INFO) << "a: " << a_ << " b: " << b_ << " c: " << c_;
    LOG(INFO) << "k: " << k_ << " m: " << m_;
}

Eigen::Vector3d Parabola::generateSinglePoint(const double &x)
{
    double y = k_ * x + m_;
    double z = a_ * x * x + b_ * x + c_;
    return Eigen::Vector3d(x, y, z);
}

void Parabola::optimizeTransmissionModel(const P2LMatchResult &lines, const OptimizationInput &input, int y_optimize)
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

    if (!y_optimize)
    {
        problem.SetParameterBlockConstant(&k_);
        problem.SetParameterBlockConstant(&m_);
    }
#if 0
    for (size_t i = 0; i < lines.size(); i++)
    {
        ceres::CostFunction *cost_function = CurveFactor::Create(lines[i], input.xySamples[i], Trans(input.R, input.t), input.cam);
        problem.AddResidualBlock(cost_function, nullptr, &a_, &b_, &c_, &k_, &m_);
    }
    ceres::CostFunction *cost_function = ParabolaEpFactor::Create(input.end_point(0), input.end_point(1), input.end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &a_, &b_, &c_, &k_, &m_);
#else
    for (size_t i = 0; i < lines.size(); i++)
    {
        ceres::CostFunction *cost_function = new CurveFactorA(lines[i], input.xySamples[i], Trans(input.R, input.t), input.cam);
        problem.AddResidualBlock(cost_function, nullptr, &k_, &m_, &a_, &b_, &c_);
    }
    ceres::CostFunction *cost_function = new ParabolaEpFactorA(input.end_point(0), input.end_point(1), input.end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &k_, &m_, &a_, &b_, &c_);
#endif

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "a: " << a_ << " b: " << b_ << " c: " << c_;
    LOG(INFO) << "k: " << k_ << " m: " << m_;
}

void Parabola::optimizeTransmissionModel(const P2PMatchResult &points, const OptimizationInput &input, int y_optimize, int time)
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

    problem.AddParameterBlock(&k_, 1);
    problem.AddParameterBlock(&m_, 1);

    if (!y_optimize || time == 2)
    {
        problem.SetParameterBlockConstant(&k_);
        problem.SetParameterBlockConstant(&m_);
    }
#if 0

    for (size_t i = 0; i < points.size(); i++)
    {
        ceres::CostFunction *cost_function = CurveP2PFactor::Create(points[i], input.xySamples[i], Trans(input.R, input.t), input.cam, static_cast<WeightType>(time));
        problem.AddResidualBlock(cost_function, loss_function, &a_, &b_, &c_, &k_, &m_);
    }

    ceres::CostFunction *cost_function = ParabolaEpFactor::Create(input.end_point(0), input.end_point(1), input.end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &a_, &b_, &c_, &k_, &m_);
#else
    for (size_t i = 0; i < points.size(); i++)
    {
        ceres::CostFunction *cost_function = new CurveP2PFactorA(points[i], input.xySamples[i], Trans(input.R, input.t), input.cam, static_cast<WeightType>(time));
        problem.AddResidualBlock(cost_function, nullptr, &k_, &m_, &a_, &b_, &c_);
    }
            ceres::CostFunction *cost_function = new ParabolaEpFactorA(input.end_point(0), input.end_point(1), input.end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &k_, &m_, &a_, &b_, &c_);

#endif


    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "a: " << a_ << " b: " << b_ << " c: " << c_;
    LOG(INFO) << "k: " << k_ << " m: " << m_;
}

void Parabola::optimizeTransmissionModelDark(const Eigen::Vector3d& end_point)
{
    ceres::Problem problem;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 10;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    problem.AddParameterBlock(&a_, 1);
    problem.AddParameterBlock(&b_, 1);
    problem.AddParameterBlock(&c_, 1);
    problem.AddParameterBlock(&k_, 1);
    problem.AddParameterBlock(&m_, 1);

    problem.SetParameterBlockConstant(&a_);
#if 0
    ceres::CostFunction *cost_function = ParabolaEpFactor::Create(end_point(0), end_point(1), end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &a_, &b_, &c_, &k_, &m_);
#else
    ceres::CostFunction *cost_function = new ParabolaEpFactorA(end_point(0), end_point(1), end_point(2));
    problem.AddResidualBlock(cost_function, nullptr,&k_, &m_, &a_, &b_, &c_);
#endif

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "a: " << a_ << " b: " << b_ << " c: " << c_;
    LOG(INFO) << "k: " << k_ << " m: " << m_;
}