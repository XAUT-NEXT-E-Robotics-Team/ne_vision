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

#include "ne_vision/debug/ne_rerun_debug.hpp"
#include "ne_vision/debug/ne_rerun_toolkit.hpp"
#include "ne_vision/interfaces/ne_frame_input.hpp"
#include "ne_vision/ne_channals.hpp"

namespace ne_vision
{

NeRerunDebug::NeRerunDebug(const std::string& name) : name_(name) {}

void NeRerunDebug::DebugTask()
{
  // 画图
  interfaces::NeFrameInput_t msg;
  if (NV_CHANNELS.frame_input_sPtr()->Receive(msg))
  {
    NV_RERUN_REC.LogCvMat(name_ + "/frame", msg.frame, msg.cap_stamp);
  }

  // 画识别器标注
  interfaces::NeArmors2D_t armors_2d;
  if (NV_CHANNELS.armor2d_sPtr()->Receive(armors_2d))
  {
    std::vector<std::vector<Eigen::Vector2d>> groups;
    for (const auto& armor : armors_2d.armors)
    {
      groups.emplace_back(std::vector<Eigen::Vector2d>({armor.LB, armor.RT}));
      groups.emplace_back(std::vector<Eigen::Vector2d>({armor.LT, armor.RB}));
    }

    NV_RERUN_REC.LogLines2D(name_ + "/frame/detector_result",
                            groups,
                            NV_RERUN_COLOR_GREEN,
                            2.0f,
                            armors_2d.cap_stamp);
  }
}

} // namespace ne_vision
