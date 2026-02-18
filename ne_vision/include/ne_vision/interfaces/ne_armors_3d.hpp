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
    int GetId(std::string armor_id) const
    {
      if (armor_id == "outpost")
        return id_outpost;
      else
        return id;
    }
    void SetId(std::string armor_id, int id_i)
    {
      if (armor_id == "outpost")
        id_outpost = id_i;
      else
        id = id_i;
    }

    NePeriodicNumber<4> id;
    NePeriodicNumber<3> id_outpost;

    // Position in IMU frame in meters.
    Eigen::Vector3d pos_imu;

    // Rotation in IMU frame, only YAW(Z)
    Sophus::SO2d rot_imu;

    // Use to visualization.
    struct Debug_t
    {
      Eigen::Vector2d pos_current;

      bool is_main = false; // If this armor is 'main_armor'. If it isn't, the
                            // var below is meaningless.

      Eigen::Vector2d pos_last; // Last position of 'main_armor'
      Eigen::Vector2d pos_kf_p; // Predicted position of 'main_armor' by kf.

      // ATTAINTION: This var is meaningless when there are two armors is
      // detected. Why? See the description in ne_tracker_2d.hpp.
      // But in that cases, these var will be set a value to help visualization
      // to draw a circle with radius 'jump_radius' to show the jump range.
      double jump_radius; // If distance more than this, consider it as a jump.
      double real_distance; // Real distance between the predicted position and
                            // The detected position.

    } debug;
  };

  std::vector<Armor3D_t> armors;
};

} // namespace interfaces
} // namespace ne_vision
