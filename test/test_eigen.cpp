/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: test_eigen.cpp
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-16 17:45:57
  * @last_modified_date: 2018-08-17 10:21:58
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

//INCLUDE
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>   // AngleAxis

//CODE

using namespace Eigen;
using namespace std;

// import most common Eigen types
int main(int, char *[])
{
    //Matrix3f m3; //3x3单精度矩阵
    //m3 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    //Matrix4f m4 = Matrix4f::Identity(); //4x4单位矩阵（单精度）
    //Vector4i v4(1, 2, 3, 4); // 长度为4的整型向量

    //// 输出结果
    //std::cout << "m3\n" << m3 << "\nm4:\n"
    //    << m4 << "\nv4:\n" << v4 << std::endl;

  /*
   * Vector3d
   */
  Vector3d v3;
  // Initiliazation
  // Vector3d v3(1.0,2.0,3.0);
  std::cout << "Vector Origin:\n" << v3 << std::endl;
  v3(0) = 100;
  v3[1] = 200;
  v3.z() = 300;
  std::cout << "Assigned:\n" << v3 << std::endl;
    // Low cost without assert
  //v3.coeff(0) = 900; // Return a const value
  v3.coeffRef(1) = 800;
  v3[2] = 700;
  std::cout << "Assigned:\n" << v3 << std::endl;
  std::cout << "===========================" << std::endl;

  /**
   * Matrix
   */
  Matrix3d m3, m3_;
  std::cout << "Vector Origin:\n" << m3 << std::endl;
  m3 << 1,2,3,
        4,5,6,
        7,8,9;
  m3_ = m3;
  m3_(0,0) = 100;
  m3_.coeffRef(1,1) = 900;
  std::cout << "Origin(new):\n" << m3 << std::endl;
  std::cout << "Assigned(new):\n" << m3_ << std::endl;
  std::cout << "===========================" << std::endl;

  /**
   * Quaternion
   * 旋转矩阵（3X3）:Eigen::Matrix3d
   * 旋转向量（3X1）:Eigen::AngleAxisd(theta, axisVector)
   * 四元数（4X1）:Eigen::Quaterniond
   * 平移向量（3X1）:Eigen::Vector3d
   * 变换矩阵（4X4）:Eigen::Isometry3d
   */
  Eigen::AngleAxisd vector_rotation(M_PI/4, Eigen::Vector3d(0,0,1));
  Eigen::Matrix3d matrix_rotation();
  //std::cout << "Rotation vector:\n" << vector_rotation.vec() << std::endl;
  std::cout << "Rotation vector(Matrix):\n" << vector_rotation.matrix() << std::endl;
  Eigen::Quaterniond quaternion(vector_rotation);
  //Eigen::Quaterniond quaternion(1,0,0,0);
  std::cout << "Quaterniond:\n"
            << "W: " << quaternion.w() << std::endl
            << "Vector:\n" << quaternion.vec() << std::endl
            << "Coeffs:\n" << quaternion.coeffs() << std::endl;


  return 0;
}
