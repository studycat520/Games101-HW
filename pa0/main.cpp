#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
using namespace std;
using namespace Eigen;
int main(){

    //hw0
    // 3.1 作业描述
    // 给定一个点P =(2,1), 将该点绕原点先逆时针旋转45◦，再平移(1,2), 计算出
    // 变换后点的坐标（要求用齐次坐标进行计算）。

    //π=acos(-1)
    float PI = std::acos(-1.0f);
    float theta = 45.0f/180.0f * PI;
    Vector3f P(2.0f,1.0f,1.0f);
    Matrix3f R,T;
    R << cos(theta),-sin(theta),0,sin(theta),cos(theta),0,0,0,1;
    T << 1,0,1,0,1,2,0,0,1;
    P=T*R*P;
    cout<<P<<endl;

    // 3.2 编译
    // 为方便之后的作业编写，本次作业要求使用cmake 进行编译。
    // 首先，编写好本次作业的程序main.cpp。
    // 然后, 在main.cpp 所在目录下，打开终端(命令行)，依次输入：
    // • mkdir build: 创建名为build 的文件夹。
    // • cd build: 移动到build 文件夹下。
    // • cmake ..: 注意其中’..’ 表示上一级目录，若为’.’ 则表示当前目录。
    // • make: 编译程序，错误提示会显示在终端中。
    // • ./Transformation：若上一步无错误，则可运行程序(这里的Transformation
    // 为可执行文件名，可参照CMakeLists.txt 中修改)。


    // // Basic Example of cpp
    // std::cout << "Example of cpp \n";
    // float a = 1.0, b = 2.0;
    // std::cout << a << std::endl;
    // std::cout << a/b << std::endl;
    // std::cout << std::sqrt(b) << std::endl;
    // std::cout << std::acos(-1) << std::endl;
    // std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // // Example of vector
    // cout << "Example of vector \n";
    // // vector definition
    // Vector3f v(1.0f,2.0f,3.0f);
    // Vector3f w(1.0f,0.0f,0.0f);
    // // vector output
    // std::cout << "Example of output \n";
    // std::cout << v << std::endl;
    // // vector add
    // std::cout << "Example of add \n";
    // std::cout << v + w << std::endl;
    // // vector scalar multiply
    // std::cout << "Example of scalar multiply \n";
    // std::cout << v * 3.0f << std::endl;
    // std::cout << 2.0f * v << std::endl;

    // // Example of matrix
    // std::cout << "Example of matrix \n";
    // // matrix definition
    // Eigen::Matrix3f i,j;
    // i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    // j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // // matrix output
    // std::cout << "Example of output \n";
    // std::cout << i << std::endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v

    return 0;
}