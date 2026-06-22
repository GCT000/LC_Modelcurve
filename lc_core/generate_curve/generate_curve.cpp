#include "generate_curve.h"

bool ReadPixelPoints(const std::string &filename, std::vector<cv::Point2d> &points)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "错误：无法打开文件 " << filename << std::endl;
        return false;
    }

    double x, y;
    // 按行读取两个数字（x坐标和y坐标）
    while (file >> x >> y)
    {
        points.emplace_back(x, y);
        std::cout << "www   " << x << "  " << y << std::endl;
    }

    file.close();

    if (points.empty())
    {
        std::cerr << "警告：文件中未读取到任何坐标数据" << std::endl;
        return false;
    }

    std::cout << "成功读取 " << points.size() << " 个像素坐标" << std::endl;
    return true;
}

std::vector<cv::Point2d> GenerateNewCurve(const std::vector<cv::Point2d> &original_points,
                                          double a0, double a1, double a2,double a3,double a4,
                                          double step = 0.5)
{
    std::vector<cv::Point2d> new_curve;

    // 确定原始数据的x范围
    double x_min = original_points[0].x;
    double x_max = original_points[0].x;
    for (const auto &p : original_points)
    {
        x_min = std::min(x_min, p.x);
        x_max = std::max(x_max, p.x);
    }

    // 以0.5为步幅遍历x，计算对应的y值
    for (double x = x_min; x <= x_max; x += step)
    {
        double y = a0 + a1 * x + a2 * x * x + a3 *x* x*x+ a4 *x* x*x*x;
        new_curve.emplace_back(x, y);
    }

    std::cout << "生成新曲线：x范围 [" << x_min << ", " << x_max << "]，步幅 " << step
              << "，共 " << new_curve.size() << " 个点" << std::endl;
    return new_curve;
}

void DrawCurves(const std::vector<cv::Point2d> &original_points,
                const std::vector<cv::Point2d> &new_curve,
                const std::string &save_path = "/home/gct/LC_Modelcurve/data/1-13guangzhou/curve_fitting_result.png")
{
    // 1. 计算坐标范围，确定画布大小（添加边距避免点超出画布）
    double x_min = original_points[0].x, x_max = original_points[0].x;
    double y_min = original_points[0].y, y_max = original_points[0].y;
    for (const auto &p : original_points)
    {
        x_min = std::min(x_min, p.x);
        y_min = std::min(y_min, p.y);
        x_max = std::max(x_max, p.x);
        y_max = std::max(y_max, p.y);
    }
    for (const auto &p : new_curve)
    {
        x_min = std::min(x_min, p.x);
        y_min = std::min(y_min, p.y);
        x_max = std::max(x_max, p.x);
        y_max = std::max(y_max, p.y);
    }

    // 添加边距（50像素），避免点贴边
    const int margin = 50;
    x_min -= margin;
    x_max += margin;
    y_min -= margin;
    y_max += margin;

    // 2. 坐标映射：将实际坐标转换为画布像素坐标（解决y轴方向反转问题）
    int canvas_width = 800;  // 画布宽度
    int canvas_height = 600; // 画布高度
    auto map_x = [&](double x)
    { return (x - x_min) / (x_max - x_min) * canvas_width; };
    auto map_y = [&](double y)
    { return canvas_height - (y - y_min) / (y_max - y_min) * canvas_height; };

    // 3. 创建画布（白色背景）
    cv::Mat canvas(canvas_height, canvas_width, CV_8UC3, cv::Scalar(255, 255, 255));

    // 4. 绘制原始点（蓝色圆点，大小3）
    for (const auto &p : original_points)
    {
        cv::Point pix_p(map_x(p.x), map_y(p.y));
        cv::circle(canvas, pix_p, 3, cv::Scalar(255, 0, 0), -1); // 蓝色实心圆
    }

    // 5. 绘制新生成的曲线（红色实线，线宽2）
    for (size_t i = 1; i < new_curve.size(); ++i)
    {
        cv::Point p1(map_x(new_curve[i - 1].x), map_y(new_curve[i - 1].y));
        cv::Point p2(map_x(new_curve[i].x), map_y(new_curve[i].y));
        cv::line(canvas, p1, p2, cv::Scalar(0, 0, 255), 2); // 红色实线
    }

    // 6. 添加文字标注
    cv::putText(canvas, "Original Points (Blue)", cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
    cv::putText(canvas, "Fitted Curve (Red, step=0.5)", cv::Point(20, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

    // 7. 保存图片
    if (!cv::imwrite(save_path, canvas))
    {
        std::cerr << "错误：无法保存图片到 " << save_path << std::endl;
    }
    else
    {
        std::cout << "成功保存拟合结果图片：" << save_path << std::endl;
    }

    // 8. 显示图片（可选）
    cv::imshow("Curve Fitting Result", canvas);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

int main(int argc, char **argv)
{
    // 1. 读取像素坐标数据
    std::vector<cv::Point2d> pixel_points;
    std::string filename = "/home/gct/LC_Modelcurve/data/temp/curve_points.txt"; // 文件路径（根据实际情况修改）

    if (!ReadPixelPoints(filename, pixel_points))
    {
        return -1;
    }

    ceres::Problem problem;

    // 多项式参数初始值（a0~a5），可根据数据范围调整初始值
    double a0 = 0.0, a1 = 0.0, a2 = 0.0,a3 = 0.0,a4 = 0.0;
    const int total_points = pixel_points.size();
    double last_point_weight = 1.0;

    //std::ofstream ofs("/home/gct/LC_Modelcurve/data/1-13guangzhou/generate_curve.txt", std::ios::trunc);
    // 3. 为每个数据点添加代价函数
    for (int i = 0; i < total_points; ++i)
    {
        const auto &point = pixel_points[i];
        double x = point.x;
        double y = point.y;
        if (((double)(i) / total_points) < 0.1)
        {
            ceres::CostFunction *cost_function = new Generate_curve(x, y, last_point_weight * 1);
            problem.AddResidualBlock(
                cost_function,
                nullptr,      // 权重损失函数
                &a0, &a1, &a2, &a3, &a4 // 待优化的参数（根据你的多项式次数调整）
            );
        }
        else if (((double)(i) / total_points) > 0.7 && i != total_points - 1)
        {
            ceres::CostFunction *cost_function = new Generate_curve(x, y, last_point_weight * 1);
            problem.AddResidualBlock(
                cost_function,
                nullptr,      // 权重损失函数
                &a0, &a1, &a2, &a3, &a4 // 待优化的参数（根据你的多项式次数调整）
            );
        }
        else if (((double)(i) / total_points) > 0.1 && ((double)(i) / total_points) < 0.4 )
        {
            ceres::CostFunction *cost_function = new Generate_curve(x, y, last_point_weight * 1);
            problem.AddResidualBlock(
                cost_function,
                nullptr,      // 权重损失函数
                &a0, &a1, &a2, &a3, &a4 // 待优化的参数（根据你的多项式次数调整）
            );
        }

        // 判断是否是最后一个点：设置缩放权重
        else if (i == total_points - 1)
        {
            ceres::CostFunction *cost_function = new Generate_curve(x, y, last_point_weight * 100);
            problem.AddResidualBlock(
                cost_function,
                nullptr,      // 权重损失函数
                &a0, &a1, &a2, &a3, &a4 // 待优化的参数（根据你的多项式次数调整）
            );
        }
        else
        {
            // 创建代价函数对象
            ceres::CostFunction *cost_function = new Generate_curve(x, y, last_point_weight);
            problem.AddResidualBlock(
                cost_function,
                nullptr,      // 权重损失函数
                &a0, &a1, &a2, &a3, &a4 // 待优化的参数（根据你的多项式次数调整）
            );
        }

        // 添加到问题中
    }

    // 4. 设置优化选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; // 稠密QR分解（适合小规模问题）
    options.minimizer_progress_to_stdout = true;  // 输出优化过程信息
    options.max_num_iterations = 1000;            // 最大迭代次数
    options.function_tolerance = 1e-10;           // 函数收敛阈值
    options.gradient_tolerance = 1e-10;           // 梯度收敛阈值

    // 5. 执行优化
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 6. 输出优化结果
    std::cout << "\n优化总结：" << std::endl;
    std::cout << summary.BriefReport() << std::endl;

    std::cout << "\n拟合得到的多项式参数：" << std::endl;
    std::cout << "a0 = " << a0 << std::endl;
    std::cout << "a1 = " << a1 << std::endl;
    std::cout << "a2 = " << a2 << std::endl;
    std::cout << "a3 = " << a3 << std::endl;
    std::cout << "a4 = " << a4 << std::endl;

    std::vector<cv::Point2d> new_curve = GenerateNewCurve(pixel_points, a0, a1, a2,a3,a4, 0.5);

    // 8. 新增：绘制原始曲线和新曲线到同一张图片
    DrawCurves(pixel_points, new_curve);
}