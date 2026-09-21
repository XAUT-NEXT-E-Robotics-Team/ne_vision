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
// ne_tracker_2d >=> [ne_armors_3d] >=> ne_observer_3d
//

#pragma once

#include <chrono>
#include <vector>
#include "ne_vision/utils/ne_translation.hpp"
#include "Eigen/Dense"

#include "ne_vision/utils/ne_math.hpp"

#include "ne_imu_data.hpp"

namespace ne_vision
{
namespace interfaces
{

struct NeArmors3D_t
{
  // stamp after matching with IMU data.
  std::chrono::steady_clock::time_point cap_stamp;

  // 如果没有识别到这里给NULL
  std::string aim_id;

  // 无论是否识别到这里都有效
  NeImuData_t   imu_data;         // 观测时刻最近的IMU数据，主要用来后续预测补偿
  NeTranslation gimbal_to_camera; // 云台到相机的位姿
  struct Armor3D_t
  {
    // Pose from IMU
    Eigen::Vector3d    t;
    Eigen::Quaterniond q;
    double             yaw;
    Eigen::Matrix4d    cov;
  };

  // 请优先根据这个数组大小是否为空判断是否识别到
  std::vector<Armor3D_t> armors;
};

} // namespace interfaces
} // namespace ne_vision
