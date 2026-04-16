///////////////////////////////////////////////////////////
//                                                       //
//                        .                 .:-:         //
//                       :-:              :-::           //
//                      -----          .:---.            //
//                    .-------.     .:-----:             //
//                   :---------. .:-------.              //
//                  :--------------------.               //
//                ---------------------                  //
//               .-------:. :---------:                  //
//              :-----:.     .-------.                   //
//             .:---:         .-----.                    //
//            .:-:.             :-:                      //
//          .-:.                 .                       //
//         .:                                            //
//                                                       //
//    ███╗   ██╗███████╗██╗  ██╗████████╗    ███████╗    //
//    ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝    ██╔════╝    //
//    ██╔██╗ ██║█████╗   ╚███╔╝    ██║       █████╗      //
//    ██║╚██╗██║██╔══╝   ██╔██╗    ██║       ██╔══╝      //
//    ██║ ╚████║███████╗██╔╝ ██╗   ██║       ███████╗    //
//    ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝       ╚══════╝    //
//                                                       //
///////////////////////////////////////////////////////////
//                                                       //
// Copyright (c) 2026 XAUT NEXT-E. All Rights Reserved.  //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// Description:
// 自瞄坐标系变换
// 包含各种坐标系（包括UV坐标系上的变换）
// 并且包含对所有相机和装甲板参数的维护，很是方便

#include "ne_vision/ne_auto_aim_tf.hpp"

#include "ne_vision/utils/ne_param.hpp"
#include <vector>

namespace ne_vision
{

NeAutoAimTf::NeAutoAimTf(std::string camera_name)
{
  // 读取参数

  // 获取相机参数
  // TODO: 根据名字区分相机参数
  try
  {
    auto camera_param = NV_PARAM["hardware"]["camera"];

    param_.camera_matrix_cv =
        (cv::Mat_<double>(3, 3)
             << camera_param["camera_matrix"]["fx"].as<double>(),
         0,
         camera_param["camera_matrix"]["cx"].as<double>(),
         0,
         camera_param["camera_matrix"]["fy"].as<double>(),
         camera_param["camera_matrix"]["cy"].as<double>(),
         0,
         0,
         1);
    param_.dist_coeffs_cv =
        (cv::Mat_<double>(1, 5)
             << camera_param["dist_coeffs"]["k1"].as<double>(),
         camera_param["dist_coeffs"]["k2"].as<double>(),
         camera_param["dist_coeffs"]["p1"].as<double>(),
         camera_param["dist_coeffs"]["p2"].as<double>(),
         camera_param["dist_coeffs"]["k3"].as<double>());

    param_.camera_matrix_eigen << param_.camera_matrix_cv.at<double>(0, 0), 0,
        param_.camera_matrix_cv.at<double>(0, 2), 0,
        param_.camera_matrix_cv.at<double>(1, 1),
        param_.camera_matrix_cv.at<double>(1, 2), 0, 0, 1;
    param_.dist_coeffs_eigen << param_.dist_coeffs_cv.at<double>(0, 0),
        param_.dist_coeffs_cv.at<double>(0, 1),
        param_.dist_coeffs_cv.at<double>(0, 2),
        param_.dist_coeffs_cv.at<double>(0, 3),
        param_.dist_coeffs_cv.at<double>(0, 4);

    // 获取变换参数
    // 云台到相机
    const double r_r = camera_param["gimbal_to_camera"]["r"]["r"].as<double>(0);
    const double r_p = camera_param["gimbal_to_camera"]["r"]["p"].as<double>(0);
    const double r_y = camera_param["gimbal_to_camera"]["r"]["y"].as<double>(0);
    const double t_x = camera_param["gimbal_to_camera"]["t"]["x"].as<double>(0);
    const double t_y = camera_param["gimbal_to_camera"]["t"]["y"].as<double>(0);
    const double t_z = camera_param["gimbal_to_camera"]["t"]["z"].as<double>(0);

    param_.T_G_C.q() = Eigen::AngleAxisd(r_y, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(r_p, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(r_r, Eigen::Vector3d::UnitX());
    param_.T_G_C.t() = Eigen::Vector3d(t_x, t_y, t_z);

    // 与Opencv相机系转换
    Eigen::Matrix3d CC_to_C;
    // clang-format off
    CC_to_C  <<  0,  0,  1,
                -1,  0,  0,
                 0, -1,  0;
    // clang-format on
    param_.T_G_C.q() = param_.T_G_C.q() * Eigen::Quaterniond(CC_to_C);
  }
  catch (const std::exception& e)
  {
    NV_ERROR("Failed to load parameters for NeAutoAimTf: {}", e.what());
    std::exit(EXIT_FAILURE);
  }
}

cv::Point2d
NeAutoAimTf::ProjectToImagePlane(const interfaces::NeImuData_t& imu_data,
                                 const Eigen::Vector3d&         point_3d)
{
  // 填入Gimbal到IMU的变换
  NeTranslation T_I_G;
  T_I_G.q() = imu_data.quat;
  T_I_G.t().setZero();

  // 计算IMU系到相机系的变换
  auto T_C_I = (T_I_G * param_.T_G_C).Inverse();

  // 投影
  Eigen::Vector3d point_c = T_C_I * point_3d;
  Eigen::Vector3d tmp = (param_.camera_matrix_eigen * point_c) / point_c.z();

  return cv::Point2d(tmp.x(), tmp.y());
}

} // namespace ne_vision