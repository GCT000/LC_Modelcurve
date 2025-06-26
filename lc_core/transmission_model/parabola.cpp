#include "parabola.h"
#include "glog/logging.h"
#include "ceres/ceres.h"
#include "curve_factor.h"
#include "curve_factor_p2p.h"
#include "parabola_ep_factor.h"

using namespace lc_core;

void Parabola::fitTransmissionModel(std::vector<Eigen::Vector3d> &points)
{
    Eigen::MatrixXd A1(points.size(), 3);
    Eigen::VectorXd b1(points.size());
    for (int i = 0; i < points.size(); ++i)
    {
        A1(i, 0) = points[i](1) * points[i](1);
        A1(i, 1) = points[i](1);
        A1(i, 2) = 1.0;
        b1(i) = points[i](0);
    }

    Eigen::VectorXd x1 = A1.colPivHouseholderQr().solve(b1);
    a1_ = x1(0);
    b1_ = x1(1);
    c1_ = x1(2);


    Eigen::MatrixXd A2(points.size(), 3);
    Eigen::VectorXd b2(points.size());
    for (int i = 0; i < points.size(); ++i)
    {
        A2(i, 0) = points[i](1) * points[i](1);
        A2(i, 1) = points[i](1);
        A2(i, 2) = 1.0;
        b2(i) = points[i](2);
    }

    Eigen::VectorXd x2 = A2.colPivHouseholderQr().solve(b2);
    a2_ = x2(0);
    b2_ = x2(1);
    c2_ = x2(2);

    LOG(INFO) << "a1: " << a1_ << " b1: " << b1_ << " c1: " << c1_;
    LOG(INFO) << "a2: " << a2_ << " b2: " << b2_ << " c2: " << c2_;
}

Eigen::Vector3d Parabola::generateSinglePoint(const double &y)
{
    double x = a1_ * y * y + b1_ * y + c1_;
    double z = a2_ * y * y + b2_ * y + c2_;
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
    // options.num_threads = 8;_function, nullptr, &a_, &b_, &c_, &k_, &m_);

#if 0
    for (size_t i = 0; i < lines.size(); i++)
    {
        ceres::CostFunction *cost_function = CurveFactor::Create(lines[i], input.xSamples[i], Trans(input.R, input.t), input.cam);
        problem.AddResidualBlock(cost_function, nullptr, &a_, &b_, &c_, &k_, &m_);
    }
    ceres::CostFunction *cost_function = ParabolaEpFactor::Create(input.end_point(0), input.end_point(1), input.end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &a_, &b_, &c_, &k_, &m_);
#else
    for (size_t i = 0; i < lines.size(); i++)
    {
        ceres::CostFunction *cost_function = new CurveFactorA(lines[i], input.ySamples[i], Trans(input.R, input.t), input.cam);
        problem.AddResidualBlock(cost_function, nullptr, &a1_, &b1_, &c1_, &b2_, &c2_);
    }
    ceres::CostFunction *cost_function = new ParabolaEpFactorA(input.end_point(0), input.end_point(1), input.end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &a1_, &b1_, &c1_, &b2_, &c2_);
#endif

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "a: " << a1_ << " b: " << b1_ << " c: " << c1_;
    LOG(INFO) << "k: " << a2_ << " m: " << b2_;
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

    // problem.AddParameterBlock(&k_, 1);
    // problem.AddParameterBlock(&m_, 1);

    // if (!y_optimize || time == 2)
    // {
    //     problem.SetParameterBlockConstant(&k_);
    //     problem.SetParameterBlockConstant(&m_);
    // }
#if 0

    for (size_t i = 0; i < points.size(); i++)
    {
        ceres::CostFunction *cost_function = CurveP2PFactor::Create(points[i], input.xSamples[i], Trans(input.R, input.t), input.cam, static_cast<WeightType>(time));
        problem.AddResidualBlock(cost_function, loss_function, &a_, &b_, &c_, &k_, &m_);
    }

    ceres::CostFunction *cost_function = ParabolaEpFactor::Create(input.end_point(0), input.end_point(1), input.end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &a_, &b_, &c_, &k_, &m_);
#else
    for (size_t i = 0; i < points.size(); i++)
    {
        ceres::CostFunction *cost_function = new CurveP2PFactorA(points[i], input.ySamples[i], Trans(input.R, input.t), input.cam, static_cast<WeightType>(time));
        problem.AddResidualBlock(cost_function, nullptr, &a1_, &b1_, &c1_, &a2_, &b2_, &c2_);
    }
            ceres::CostFunction *cost_function = new ParabolaEpFactorA(input.end_point(0), input.end_point(1), input.end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &a1_, &b1_, &c1_, &a2_, &b2_, &c2_);

#endif


    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "a1: " << a1_ << " b1: " << b1_ << " c1: " << c1_;
    LOG(INFO) << "a2: " << a2_ << " b2: " << b2_ << " c2: " << c2_;
}

void Parabola::optimizeTransmissionModelDark(const Eigen::Vector3d& end_point)
{
    ceres::Problem problem;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 10;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    problem.AddParameterBlock(&a1_, 1);
    problem.AddParameterBlock(&b1_, 1);
    problem.AddParameterBlock(&c1_, 1);
    problem.AddParameterBlock(&a2_, 1);
    problem.AddParameterBlock(&b2_, 1);

    problem.SetParameterBlockConstant(&a1_);
#if 0
    ceres::CostFunction *cost_function = ParabolaEpFactor::Create(end_point(0), end_point(1), end_point(2));
    problem.AddResidualBlock(cost_function, nullptr, &a_, &b_, &c_, &k_, &m_);
#else
    ceres::CostFunction *cost_function = new ParabolaEpFactorA(end_point(0), end_point(1), end_point(2));
    problem.AddResidualBlock(cost_function, nullptr,&a1_, &b1_, &c1_, &a2_, &b2_);
#endif

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "a1: " << a1_ << " b1: " << b1_ << " c1: " << c1_;
    LOG(INFO) << "a2: " << a2_ << " b2: " << b2_ << " c2: " << c2_;
}