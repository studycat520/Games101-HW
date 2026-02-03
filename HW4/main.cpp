#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // Base case: 如果只剩下一个点，返回这个点，这就是最终结果
    if (control_points.size() == 1) 
    {
        return control_points[0];
    }

    // Recursive step: 创建下一层级的控制点序列
    std::vector<cv::Point2f> next_level_points;

    // 遍历当前层级的每两个相邻点
    for (int i = 0; i < control_points.size() - 1; ++i) 
    {
        // 核心公式：P_new = (1-t) * P_i + t * P_{i+1}
        // OpenCV 的 Point2f 支持直接的向量加法和标量乘法
        auto p0 = control_points[i];
        auto p1 = control_points[i + 1];
        
        // 线性插值计算新点
        cv::Point2f point = (1 - t) * p0 + t * p1;
        
        next_level_points.push_back(point);
    }

    // 将新的点集传入下一层递归
    return recursive_bezier(next_level_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // 遍历时间 t，从 0 到 1
    // 步长 0.001 决定了曲线的平滑程度（点的密度）
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        // 调用递归算法计算当前 t 对应的曲线点坐标
        auto point = recursive_bezier(control_points, t);

        // 绘制点
        // 注意：OpenCV 的坐标访问顺序是 (y, x) 即 (row, col)
        // 最好加一个边界检查，防止计算误差导致越界崩溃
        if (point.x >= 0 && point.x < window.cols && point.y >= 0 && point.y < window.rows)
        {
            // 作业要求：绘制为绿色
            // window 是 CV_8UC3 类型。
            // 通常 OpenCV 是 BGR 顺序，但在 main 函数开头有一句 cvtColor(..., RGB)
            // 不过 imshow 通常预期 BGR。
            // 让我们观察 naive_bezier：它设置了 index [2] 为 255 (也就是红色)。
            // 那么绿色应该是 index [1]。
            window.at<cv::Vec3b>(point.y, point.x)[1] = 255; 
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
