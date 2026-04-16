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
// 云台控制参考值
// 自瞄算法部分的最后一步

#pragma once

#include "ne_vision/interfaces/ne_imu_data.hpp"
#include <string>
#include <chrono>
#include <vector>

namespace ne_vision
{

namespace interfaces
{

struct NeGimbalControlRef_t
{
  // 拍摄时间，用于可视化对齐
  std::chrono::steady_clock::time_point cap_stamp;

  // 目标预测后时间
  // std::chrono::steady_clock::time_point target_predicted_stamp;

  // 云台预测后时间
  // std::chrono::steady_clock::time_point gimbal_predicted_stamp;

  // 控制时间：期望的云台位姿是哪个时间点的
  std::chrono::steady_clock::time_point control_stamp;

  // 目标装甲板ID
  std::string armor_id = "NULL";

  // 目标有效性
  bool valid = false;

  // 参考目标值（发给电控）
  double yaw_ref = 0.0;
  double pitch_ref = 0.0;
  double yaw_v_ref = 0.0;
  double pitch_v_ref = 0.0;

  // 保存一些数据，用于可视化调试
  struct debug_t
  {
    // 选板的装甲板pose x y z yaw
    Eigen::Vector4d     target_armor_xyzy;
    std::vector<double> yaw_local_traj;
    std::vector<double> pitch_local_traj;

    // 当前预测的IMU数据（为统一接口这样写，只有四元数和角速度有效，加速度无效
    // 接收时间戳不变
    interfaces::NeImuData_t predicted_imu_data;
  } debug;
};

} // namespace interfaces

} // namespace ne_vision