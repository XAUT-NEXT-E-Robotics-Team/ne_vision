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
// ne_armor_2d interface for 2D armor
// detector >=> [ne_armor_2d] >=> tracker_2d
//
// ATTENTION: ANY define in here should be multi-thread safe

#pragma once

#include <chrono>
#include <string>

#include "Eigen/Dense"

namespace ne_vision
{
namespace interfaces
{

struct NeArmors2D_t
{
  // Timestamp of the captured frame
  std::chrono::steady_clock::time_point cap_stamp;

  // Frame height and width
  size_t frame_height = 0;
  size_t frame_width = 0;

  struct Armor_t
  {
    Armor_t() = default;
    Armor_t(const std::string& armor_id_str,
            double             LT_x,
            double             LT_y,
            double             LB_x,
            double             LB_y,
            double             RT_x,
            double             RT_y,
            double             RB_x,
            double             RB_y)
    {
      armor_id = armor_id_str;
      LT << LT_x, LT_y;
      LB << LB_x, LB_y;
      RT << RT_x, RT_y;
      RB << RB_x, RB_y;

      center = (LT + LB + RT + RB) / 4.0;
    }

    // Armor ID
    // 1, 2, 3, 4, 7, outpost, base
    std::string armor_id;

    char armor_color; // 这里是颜色 就 R B 和 N N就是无颜色

    // Armor position in the image R = LEFT & T = TOP
    // NOTE: Eigen use DEEP COPY to overload "=".
    //       So it safer than cv::Mat
    Eigen::Vector2d LT;
    Eigen::Vector2d LB;
    Eigen::Vector2d RT;
    Eigen::Vector2d RB;

    // Center will be aoto calculated by the constructor, and will not be
    // changed after that.
    Eigen::Vector2d center;
  };

  std::vector<Armor_t> armors;
};

} // namespace interfaces
} // namespace ne_vision
