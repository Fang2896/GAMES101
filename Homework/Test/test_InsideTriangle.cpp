#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

using namespace Eigen;


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f v01 = {_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0};
    Vector3f v12 = {_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0};
    Vector3f v20 = {_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0};
    
    Vector3f v0p = {x - _v[0].x(), y - _v[0].y(), 0};
    Vector3f v1p = {x - _v[1].x(), y - _v[1].y(), 0};
    Vector3f v2p = {x - _v[2].x(), y - _v[2].y(), 0};

    return (v01.cross(v0p).z() <= 0) && (v12.cross(v1p).z() <= 0) && (v20.cross(v2p).z() <= 0);
}

int main() {
  Vector3f v0 = {0, 0, 0};
  Vector3f v1 = {0, 10, 0};
  Vector3f v2 = {10, 0, 0};
  Vector3f v[3] = {v0, v1, v2};
  
    /*
  自行输入
  */
  while(1) {
    std::cout << "Please enter the Point:" << std::endl;
    float x, y;

    std::cin >> x >> y; //定义罗德里格斯旋转轴和角
    if(x == 0 && y == 0) {
      break;
    }

    bool flag = insideTriangle(x, y, v);
    if(flag) {
      std::cout << "Inside the Triangle!";
    } else {
      std::cout << "Outside the Triangle!";
    }
    std::cout << std::endl;
    
  }

  // Vector3f p0 = {0, 1, 0};
  // Vector3f p1 = {1, 0, 0};
  // std::cout << p0.cross(p1) << std::endl;
  std::cout << "OVER!" << std::endl;
  return 0;
}
