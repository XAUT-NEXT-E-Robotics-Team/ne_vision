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
#include "sophus/so2.hpp"

#include "ne_vision/utils/ne_math.hpp"

namespace ne_vision
{
namespace interfaces
{

struct NeArmors3D_t
{
  // stamp after matching with IMU data.
  std::chrono::steady_clock::time_point cap_stamp;

  std::string aim_id;

  struct Armor3D_t
  {
    // Pose from IMU
    Eigen::Vector3d    t;
    Eigen::Quaterniond q;
    Sophus::SO2d       yaw;
    Eigen::Matrix4d    cov;

    // Use to visualization.
    struct Debug_t
    {
      // LT, LB, RB, RT
      std::vector<cv::Point2d> re_projected_pts;

      struct
      {
        Eigen::Vector3d    t;
        Eigen::Quaterniond q;
      } camera_to_imu;

    } debug;
  };

  std::vector<Armor3D_t> armors;
};

} // namespace interfaces
} // namespace ne_vision
