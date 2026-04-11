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

#include <string>
#include <chrono>

namespace ne_vision
{

namespace interfaces
{

struct NeGimbalControlRef_t
{
  // 拍摄时间，用于可视化对齐
  std::chrono::steady_clock::time_point cap_stamp;

  // 预测后时间
  std::chrono::steady_clock::time_point predicted_stamp;

  // 目标装甲板ID
  std::string armor_id = "NULL";

  // 目标有效性
  bool valid = false;

  // 参考目标值（发给电控）
  double yaw_ref = 0.0;
  double pitch_ref = 0.0;
  double yaw_v_ref = 0.0;
  double pitch_v_ref = 0.0;

  // 保存选板数据，用于可视化调试
  struct debug_t
  {
    // 选板的装甲板pose x y z yaw
    Eigen::Vector4d target_armor_xyzy;
  } debug;
};

} // namespace interfaces

} // namespace ne_vision