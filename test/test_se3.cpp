/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: test_se3.cpp
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-08-17 14:18:21
  * @last_modified_date: 2018-08-17 15:19:50
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

//INCLUDE
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <iostream>

//CODE
int main(int argc, char** argv)
{
  Sophus::SE3<double> se3;
    // Get Matrix
    std::cout << "Default constructor:\n" << std::endl;
    std::cout << "Transformation Matrix:\n" << se3.matrix() << std::endl;
    std::cout << "Transformation Matrix3X4:\n" << se3.matrix3x4() << std::endl;
    std::cout << "Params(quaternion):\n" << se3.params() << std::endl;
    std::cout << "SE3 Data(quaternion, translation):\n"
              << "[0]x: " << se3.data()[0] << std::endl
              << "[1]y: " << se3.data()[1] << std::endl
              << "[2]z: " << se3.data()[2] << std::endl
              << "[3]w: " << se3.data()[3] << std::endl
              << "[4]tx: " << se3.data()[4] << std::endl
              << "[5]ty: " << se3.data()[5] << std::endl
              << "[6]tz: " << se3.data()[6] << std::endl;
    std::cout << "Log(SE3) get_se3:\n" << se3.log() << std::endl;
    std::cout << "Hat(se3) se3->SE3:\n" << se3.hat(se3.log()) << std::endl;
    std::cout << "Vee(SE3): SE3->se3\n" << se3.vee(se3.hat(se3.log()))<< std::endl;
    std::cout << "*******************************" << std::endl;
    se3.data()[2] = 0.707107;
    se3.data()[3] = 0.707107;
    std::cout << "Rotation Matrix:\n" << se3.matrix() << std::endl;
    std::cout << "Transformation Matrix3X4:\n" << se3.matrix3x4() << std::endl;
    std::cout << "Params(quaternion):\n" << se3.params() << std::endl;
    std::cout << "Data(quaternion)(Change):\n"
              << "[0]: " << se3.data()[0] << std::endl
              << "[1]: " << se3.data()[1] << std::endl
              << "[2]: " << se3.data()[2] << std::endl
              << "[3]: " << se3.data()[3] << std::endl;
    std::cout << "Log(se3) get_se3:\n" << se3.log() << std::endl;
    std::cout << "Hat(se3) se3->SE3:\n" << se3.hat(se3.log()) << std::endl;
    std::cout << "Vee(se3): SE3->se3\n" << se3.vee(se3.hat(se3.log()))<< std::endl;
    //auto recovery_se3 = se3.exp(se3.vee(se3.hat(se3.log())));
    //auto recovery_rotation = recovery_se3.matrix();
    //std::cout << "Recovery Matrix:\n" << recovery_rotation << std::endl;
    auto recovery_se3 = se3.exp(se3.vee(se3.hat(se3.log())));
    std::cout << "Recovery Matrix(from se3):\n" << recovery_se3.matrix() << std::endl;
    std::cout << "============END se3 Default=================" << std::endl;
  // Set quaternion
    Eigen::AngleAxisd vector_rotation(M_PI/4, Eigen::Vector3d(0,0,1));
    Eigen::Quaterniond q(vector_rotation);
    Eigen::Vector3d vector_translation(1,1,1);
    se3.setQuaternion(q);
    se3.translation() = vector_translation;
    std::cout << "Transformation Matrix:\n" << se3.matrix() << std::endl;
    std::cout << "Transformation Matrix3X4:\n" << se3.matrix3x4() << std::endl;
    std::cout << "Params(quaternion):\n" << se3.params() << std::endl;
    std::cout << "SE3 Data(quaternion, translation):\n"
              << "[0]x: " << se3.data()[0] << std::endl
              << "[1]y: " << se3.data()[1] << std::endl
              << "[2]z: " << se3.data()[2] << std::endl
              << "[3]w: " << se3.data()[3] << std::endl
              << "[4]tx: " << se3.data()[4] << std::endl
              << "[5]ty: " << se3.data()[5] << std::endl
              << "[6]tz: " << se3.data()[6] << std::endl;
    std::cout << "Log(SE3) get_se3:\n" << se3.log() << std::endl;
    std::cout << "Hat(se3) se3->SE3:\n" << se3.hat(se3.log()) << std::endl;
    std::cout << "Vee(SE3): SE3->se3\n" << se3.vee(se3.hat(se3.log()))<< std::endl;
    //auto recovery_se3 = se3.exp(se3.vee(se3.hat(se3.log())));
    recovery_se3 = se3.exp(se3.vee(se3.hat(se3.log())));
    std::cout << "Recovery Matrix(from se3):\n" << recovery_se3.matrix() << std::endl;

  return 0;
}
