#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    // caution：cos和sin接收的是弧度，作业输入的是角度
    // 1. 将角度转换为弧度

    float radian = rotation_angle / 180.0f * MY_PI;

    // 2. 填充绕 Z 轴旋转矩阵
    model << std::cos(radian), -std::sin(radian), 0, 0,
             std::sin(radian),  std::cos(radian), 0, 0,
             0,                 0,                1, 0,
             0,                 0,                0, 1;



    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    // ============================================
    // 1. 基础几何参数计算
    // ============================================
    // 将 zNear 和 zFar 转换为坐标值 (因为相机看向 -Z 方向)
    // zNear, zFar 是距离，所以坐标是负数
    float n = -zNear; 
    float f = -zFar;

    // 将 FOV 转为弧度
    float fovY = eye_fov / 180.0f * MY_PI;
    
    // 计算近平面 (Near Plane) 的边界
    // top = |n| * tan(fov/2)
    float t = std::abs(n) * std::tan(fovY / 2.0f);
    float b = -t;
    float r = t * aspect_ratio;
    float l = -r;

    // ============================================
    // 2. 构建矩阵
    // ============================================

    // 按照课程推导，分步构建
    
    // Step 2.1: 挤压矩阵 (Persp -> Ortho)
    // 使得远平面上的点挤压后，x, y 坐标与近平面成比例
    Eigen::Matrix4f M_persp_ortho;
    M_persp_ortho << n, 0, 0, 0,
                     0, n, 0, 0,
                     0, 0, n + f, -n * f,
                     0, 0, 1, 0;

    // Step 2.2: 正交投影 - 平移 (Translate)
    // 将长方体的中心移动到原点
    Eigen::Matrix4f M_ortho_trans;
    M_ortho_trans << 1, 0, 0, -(r + l) / 2.0f,
                     0, 1, 0, -(t + b) / 2.0f,
                     0, 0, 1, -(n + f) / 2.0f,
                     0, 0, 0, 1;

    // Step 2.3: 正交投影 - 缩放 (Scale)
    // 将长方体缩放到 [-1, 1]^3
    Eigen::Matrix4f M_ortho_scale;
    M_ortho_scale << 2.0f / (r - l), 0, 0, 0,
                     0, 2.0f / (t - b), 0, 0,
                     0, 0, 2.0f / (n - f), 0,
                     0, 0, 0, 1;

    // 组合矩阵: M_proj = M_scale * M_trans * M_persp->ortho
    Eigen::Matrix4f M_ortho = M_ortho_scale * M_ortho_trans;
    projection = M_ortho * M_persp_ortho;


    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
