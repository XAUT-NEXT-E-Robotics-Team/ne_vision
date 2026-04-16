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
//
// 注意的是，这里似乎有点发直觉，一般的话
// ^G_C T 协作 从C到G的变换
// 不过我喜欢写作 T_G_C

#pragma once

#include <opencv2/core/mat.hpp>
#include <string>

#include "ne_vision/interfaces/ne_imu_data.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/utils/ne_translation.hpp"
#include "ne_vision/utils/ne_math.hpp"

namespace ne_vision
{

class NeAutoAimTf
{

public:
  // 相机名称是为了在初始化时方便读取相机的参数
  explicit NeAutoAimTf(std::string camera_name = "default");

  // 获取一些你一定需要的参数
  inline void GetCameraMatrixEigen(Eigen::Matrix3d& camera_matrix)
  {
    camera_matrix = param_.camera_matrix_eigen;
  }

  inline void GetDistCoeffsEigen(Eigen::Matrix<double, 5, 1>& dist_coeffs)
  {
    dist_coeffs = param_.dist_coeffs_eigen;
  }

  inline void GetCameraToGimbal(NeTranslation& T_G_C) { T_G_C = param_.T_G_C; }

  inline void GetCameraMatrixCV(cv::Mat& camera_matrix)
  {
    camera_matrix = param_.camera_matrix_cv;
  }

  // 把一个IMU系下的点投影到相机
  cv::Point2d ProjectToImagePlane(const interfaces::NeImuData_t& imu_data,
                                  const Eigen::Vector3d&         point_3d);

private:
  std::string camera_name_;

  struct
  {
    // 相机参数
    cv::Mat                     camera_matrix_cv;
    cv::Mat                     dist_coeffs_cv;
    Eigen::Matrix3d             camera_matrix_eigen;
    Eigen::Matrix<double, 5, 1> dist_coeffs_eigen;

    // 变换参数
    NeTranslation T_G_C; // Camera to Gimbal
    NeTranslation T_G_M; // Muzzle to Gimbal

  } param_;
};

} // namespace ne_vision