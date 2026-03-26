#ifndef GENERATE_CURVE
#define GENERATE_CURVE

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ceres/ceres.h>
#include <vector>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
class Generate_curve : public ceres::SizedCostFunction<1, 1, 1, 1,1,1>
{
public:
    Generate_curve(double x, double y, double weight = 1.0) : x_(x), y_(y), weight_(weight) {}
    virtual ~Generate_curve() {};
    // 计算残差：残差 = 预测值 - 实际值
    virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const
    {
        // parameters[0][0] = a0, parameters[1][0] = a1, ..., parameters[5][0] = a5
        const double a0 = parameters[0][0];
        const double a1 = parameters[1][0];
        const double a2 = parameters[2][0];
        const double a3 = parameters[3][0];
        const double a4 = parameters[4][0];
        // const double a5 = parameters[5][0];

        // 6次多项式预测值
        double y_pred = a0 +
                        a1 * x_ +
                        a2 * x_ * x_ + 
                        a3 * x_ *x_ *x_ + 
                        a4 * x_ *x_ *x_ *x_;

        // 残差计算（使用平方残差可提高拟合稳定性）
        residual[0] = weight_* (y_pred - y_);

        if (jacobians)
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::ColMajor>> jacobians_v2a0(jacobians[0]);
                Eigen::Matrix<double, 1, 1> v2a0;
                v2a0 << weight_*1;
                jacobians_v2a0 = v2a0;
            }

            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::ColMajor>> jacobians_v2a1(jacobians[1]);
                Eigen::Matrix<double, 1, 1> v2a1;
                v2a1 << weight_*x_;
                jacobians_v2a1 = v2a1;
            }

            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::ColMajor>> jacobians_v2a2(jacobians[2]);
                Eigen::Matrix<double, 1, 1> v2a2;
                v2a2 << weight_*x_*x_;
                jacobians_v2a2 = v2a2;
            }
            if (jacobians[3])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::ColMajor>> jacobians_v2a3(jacobians[3]);
                Eigen::Matrix<double, 1, 1> v2a3;
                v2a3 << weight_*x_*x_*x_;
                jacobians_v2a3 = v2a3;
            }
            if (jacobians[4])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::ColMajor>> jacobians_v2a4(jacobians[4]);
                Eigen::Matrix<double, 1, 1> v2a4;
                v2a4 << weight_*x_*x_*x_ * x_;
                jacobians_v2a4 = v2a4;
            }
        }

        return true;
    }

private:
    const double x_; // 输入x（像素x坐标）
    const double y_; // 观测y（像素y坐标）
    const double weight_;
};

#endif