/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: test_sophus.cpp
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-16 09:21:01
  * @last_modified_date: 2018-08-17 14:10:45
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

//INCLUDE
#include <sophus/so2.hpp>
#include <sophus/so3.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

//CODE
int main()
{
  // SO3
  // Default
  //Sophus::SO3<double> so3(Eigen::Quaternion<double>(
  //      double(-1), double(0.00001), double(0.0), double(0.0)));
  //std::vector<Sophus::SO3<double>, Eigen::aligned_allocator<Sophus::SO3<double>>> so3_vec_;
  //so3_vec_.push_back(so3);
  //int i=1;
  //std::cout << a_so3 << std::endl;
  //std::cout << so3_vec_.size() << std::endl;
  //std::cout << "Address: " << &so3_vec_.at(0) << std::endl;
  //std::cout << so3.matrix() << std::endl;
  //std::cout << "This is a test for sophus" << std::endl;

  /**
   * Default SO3
   */
  Sophus::SO3<double> so3 = Sophus::SO3<double>();
    // Get matrix
    std::cout << "Default constructor:\n" << std::endl;
    std::cout << "Adjoint Matrix:\n" << so3.Adj() << std::endl;
    std::cout << "Rotation Matrix:\n" << so3.matrix() << std::endl;
    std::cout << "Params(quaternion):\n" << so3.params() << std::endl;
    std::cout << "Data(quaternion):\n"
              << "[0]: " << so3.data()[0] << std::endl
              << "[1]: " << so3.data()[1] << std::endl
              << "[2]: " << so3.data()[2] << std::endl
              << "[3]: " << so3.data()[3] << std::endl;
    std::cout << "Log(SO3) get_so3:\n" << so3.log() << std::endl;
    std::cout << "Hat(so3) so3->SO3:\n" << so3.hat(so3.log()) << std::endl;
    std::cout << "Vee(SO3): SO3->so3\n" << so3.vee(so3.hat(so3.log()))<< std::endl;
    std::cout << "*******************************" << std::endl;
    so3.data()[2] = 0.707107;
    so3.data()[3] = 0.707107;
    std::cout << "Rotation Matrix:\n" << so3.matrix() << std::endl;
    std::cout << "Params(quaternion):\n" << so3.params() << std::endl;
    std::cout << "Data(quaternion)(Change):\n"
              << "[0]: " << so3.data()[0] << std::endl
              << "[1]: " << so3.data()[1] << std::endl
              << "[2]: " << so3.data()[2] << std::endl
              << "[3]: " << so3.data()[3] << std::endl;
    std::cout << "Log(SO3) get_so3:\n" << so3.log() << std::endl;
    std::cout << "Hat(so3) so3->SO3:\n" << so3.hat(so3.log()) << std::endl;
    std::cout << "Vee(SO3): SO3->so3\n" << so3.vee(so3.hat(so3.log()))<< std::endl;
    //auto recovery_so3 = so3.exp(so3.vee(so3.hat(so3.log())));
    //auto recovery_rotation = recovery_so3.matrix();
    //std::cout << "Recovery Matrix:\n" << recovery_rotation << std::endl;
    auto recovery_so3 = so3.exp(so3.vee(so3.hat(so3.log())));
    std::cout << "Recovery Matrix(from so3):\n" << recovery_so3.matrix() << std::endl;
    std::cout << "============END so3 Default=================" << std::endl;
    // Set quaternion
    Eigen::AngleAxisd vector_rotation(M_PI/4, Eigen::Vector3d(0,0,1));
    Eigen::Quaterniond q(vector_rotation);
    so3.setQuaternion(q);
    std::cout << "Default constructor:\n" << std::endl;
    std::cout << "Adjoint Matrix:\n" << so3.Adj() << std::endl;
    std::cout << "Rotation Matrix:\n" << so3.matrix() << std::endl;
    std::cout << "Params(quaternion):\n" << so3.params() << std::endl;
    std::cout << "Data(quaternion):\n"
              << "[0]: " << so3.data()[0] << std::endl
              << "[1]: " << so3.data()[1] << std::endl
              << "[2]: " << so3.data()[2] << std::endl
              << "[3]: " << so3.data()[3] << std::endl;
    std::cout << "Log(SO3) get_so3:\n" << so3.log() << std::endl;
    std::cout << "Hat(so3) so3->SO3:\n" << so3.hat(so3.log()) << std::endl;
    std::cout << "Vee(SO3): SO3->so3\n" << so3.vee(so3.hat(so3.log()))<< std::endl;
    std::cout << "Get quaternion:\n" << so3.unit_quaternion().coeffs() << std::endl;
    std::cout << "============END so3 setQ=================" << std::endl;
    //std::cout << "Omega Matrix:\n" << so3.hat() << std::endl;
    // Get quaternion
    //
    

  //==================================================

  Sophus::SO3<double> so3_test;
  so3_test = so3;
  so3_test.matrix();
  return 0;
}
