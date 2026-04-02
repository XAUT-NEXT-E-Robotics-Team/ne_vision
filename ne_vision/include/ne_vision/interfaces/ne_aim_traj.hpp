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

#include "Eigen/src/Core/Matrix.h"

namespace ne_vision
{
namespace interfaces
{
struct NeAimTraj_t
{
  std::chrono::steady_clock::time_point cap_stamp; // 拍摄时间

  double dt = 0; // 轨迹点之间的时间间隔

  std::vector<Eigen::Vector3d> traj_points;

  double aim_yaw = 0;
  double aim_pitch = 0;

  struct
  {
    std::vector<Eigen::Vector3d> all_armors;
    double                       model_dis;   // 目标距离
    double                       model_yaw;   // 模型yaw
    double                       model_omega; // 模型角速度
  } debug;
};
} // namespace interfaces
} // namespace ne_vision
