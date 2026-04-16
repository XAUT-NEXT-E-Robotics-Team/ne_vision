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

  std::string aim_id;

  NeImuData_t imu_data; // 观测时刻最近的IMU数据，主要用来后续预测补偿

  struct Armor3D_t
  {
    // Pose from IMU
    Eigen::Vector3d    t;
    Eigen::Quaterniond q;
    double             yaw;
    Eigen::Matrix4d    cov;

    // Use to visualization.
    struct Debug_t
    {
      // LT, LB, RB, RT
      std::vector<cv::Point2d> re_projected_pts;

      // struct
      // {
      //   Eigen::Vector3d    t;
      //   Eigen::Quaterniond q;
      // } camera_to_imu;

    } debug;
  };

  std::vector<Armor3D_t> armors;
};

} // namespace interfaces
} // namespace ne_vision
