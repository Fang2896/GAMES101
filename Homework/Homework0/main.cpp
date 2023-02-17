#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){
    std::cout << "Homework 1 start here: " << std::endl;
    Eigen::Vector3d p(2.0f, 1.0f, 1.0f);   // homogeneous coordinate
    double theta = (45.0 / 180.0) * M_PI;   // 这里一定要用45.0 / 180.0，不然会直接向下取整！
    Eigen::Matrix3d r_marix;
    r_marix << cos(theta), -1.0 * sin(theta), 0,
                sin(theta), cos(theta), 0,
                0, 0, 1;
    Eigen::Matrix3d tr_matrix;
    tr_matrix << 1, 0, 1,
                0, 1, 2,
                0, 0, 1;
    // transform!
    Eigen::Vector3d result = tr_matrix * r_marix * p;
    std::cout << "Result of transforming: " << std::endl;
    std::cout << result << std::endl;

    return 0;
}