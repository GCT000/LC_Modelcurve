#include "parabola.h"
#include "glog/logging.h"
#include "ceres/ceres.h"
#include "curve_factor.h"
#include "curve_factor_p2p.h"

void Parabola::fitTransmissionModel(const std::vector<Eigen::Vector3d> &points)
{
    Eigen::MatrixXd A(points.size(), 3);
    Eigen::VectorXd b(points.size());
    for (int i = 0; i < points.size(); ++i) {
        A(i, 0) = points[i](0) * points[i](0);
        A(i, 1) = points[i](0);
        A(i, 2) = 1.0;
        b(i) = points[i](2);
    }

    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
    a_ = x(0);
    b_ = x(1);
    c_ = x(2);

    Eigen::MatrixXd A2(points.size(), 2);
    Eigen::VectorXd b2(points.size());
    for (int i = 0; i < points.size(); ++i) {
        A2(i, 0) = points[i](0);
        A2(i, 1) = 1.0;
        b2(i) = points[i](1);
    }
    Eigen::VectorXd x2 = A2.colPivHouseholderQr().solve(b2);
    k_ = x2(0);
    m_ = x2(1);

    LOG(INFO) << "a: " << a_ << " b: " << b_ << " c: " << c_;
    LOG(INFO) << "k: " << k_ << " m: " << m_;
}

Eigen::Vector3d Parabola::generateSinglePoint(const double &x)
{
    double y = k_ * x + m_;
    double z = a_ * x * x + b_ * x + c_;
    return Eigen::Vector3d(x, y, z);
}

void Parabola::optimizeTransmissionModel(const P2LMatchResult& lines, const OptimizationInput& input)
{
    ceres::Problem problem;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 10;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.num_threads = 8;
    
    for (size_t i = 0; i < lines.size(); i++) {
        ceres::CostFunction* cost_function = CurveFactor::Create(lines[i], input.xSamples[i], Trans(input.R, input.t), input.cam);
        problem.AddResidualBlock(cost_function, nullptr, &a_, &b_, &c_, &k_, &m_);
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "a: " << a_ << " b: " << b_ << " c: " << c_;
    LOG(INFO) << "k: " << k_ << " m: " << m_;
}

void Parabola::optimizeTransmissionModel(const P2PMatchResult& points, const OptimizationInput& input, int time)
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

    // if (time > 1) {
    //     problem.SetParameterBlockConstant(&k_);
    //     problem.SetParameterBlockConstant(&m_);
    // }

    for (size_t i = 0; i < points.size(); i++) {
        ceres::CostFunction* cost_function = CurveP2PFactor::Create(points[i], input.xSamples[i], Trans(input.R, input.t), input.cam, static_cast<WeightType>(time));
        problem.AddResidualBlock(cost_function, loss_function, &a_, &b_, &c_, &k_, &m_);
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "After optimization: ";
    LOG(INFO) << "a: " << a_ << " b: " << b_ << " c: " << c_;
    LOG(INFO) << "k: " << k_ << " m: " << m_;
}