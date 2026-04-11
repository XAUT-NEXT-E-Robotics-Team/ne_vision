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
//
// 目标（装甲板）的轨迹
//
// tracker_3d >=> [ne_aim_tra] >=> planner

#pragma once

#include <vector>
#include <memory>

#include "Eigen/src/Core/Matrix.h"

#include "types/ne_aim_predictor.hpp"
#include "ne_vision/interfaces/ne_imu_data.hpp"

namespace ne_vision
{
namespace interfaces
{
struct NeAimState_t
{
  std::chrono::steady_clock::time_point cap_stamp; // 拍摄时间 用于对齐

  bool has_target = false; // 是否有目标

  std::string armor_id = "NULL";

  // 使用 shared_ptr 保证预测器的多态生命周期，以及在 channel (消息队列)
  // 被线程间传递时的安全拷贝与销毁机制
  std::shared_ptr<NeAimPredictorBase> aim_predictor;

  // 预测器生成时的最新IMU
  // 如果没有目标，该IMU无效
  interfaces::NeImuData_t newest_imu;

  struct
  {
    std::vector<Eigen::Vector4d> all_armors;  // x y z yaw
    double                       model_dis;   // 目标距离
    double                       model_yaw;   // 模型yaw
    double                       model_omega; // 模型角速度
  } debug;
};
} // namespace interfaces
} // namespace ne_vision
