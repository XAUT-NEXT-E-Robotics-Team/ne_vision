///////////////////////////////////////////////////////////
//                                                       //
//                        .           .:-:               //
//                        :-:             :-::           //
//                      -----          .:---.            //
//                    .-------.     .:-----:             //
//                   :---------. .:-------.              //
//                  :--------------------.               //
//                 ---------------------                 //
//                .-------:. :---------:                 //
//               :-----:.     .-------.                  //
//              .:---:         .-----.                   //
//             .:-:.              :-:                    //
//           .-:.                  .                     //
//          .:                                           //
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
// 这里保存所有的channel，方便各个任务随意调用
// 外部也可以调用获取数据哦
//

#pragma once

#include <memory>

#include "ne_vision/utils/ne_channel.hpp"

#include "ne_vision/interfaces/ne_aim_traj.hpp"
#include "ne_vision/interfaces/ne_armors_2d.hpp"
#include "ne_vision/interfaces/ne_armors_3d.hpp"
#include "ne_vision/interfaces/ne_debug_frame.hpp"
#include "ne_vision/interfaces/ne_frame_input.hpp"
#include "ne_vision/interfaces/ne_imu_data.hpp"

namespace ne_vision
{

class NeChannels
{
public:
  static NeChannels& GetInstance()
  {
    static NeChannels instance;
    return instance;
  }

  NeChannels(const NeChannels&) = delete;
  NeChannels& operator=(const NeChannels&) = delete;

  NeChannels(NeChannels&&) = delete;
  NeChannels& operator=(NeChannels&&) = delete;

  auto frame_input_sPtr() const { return frame_input_sPtr_; }
  auto armor2d_sPtr() const { return armor2d_sPtr_; }
  auto imu_data_sPtr() const { return imu_data_sPtr_; }
  auto armor3d_sPtr() const { return armor3d_sPtr_; }
  auto aim_traj_sPtr() const { return aim_traj_sPtr_; }
  auto debug_frame_sPtr() const { return debug_frame_sPtr_; }

private:
  NeChannels()
  {
    frame_input_sPtr_ = std::make_shared<NeChannel<interfaces::NeFrameInput_t>>(
        "frame_input", NeChannelType_e::KEEP_ON_READ, 1);
    armor2d_sPtr_ = std::make_shared<NeChannel<interfaces::NeArmors2D_t>>(
        "armor2d", NeChannelType_e::KEEP_ON_READ, 1);
    imu_data_sPtr_ = std::make_shared<NeChannel<interfaces::NeImuData_t>>(
        "imu_data", NeChannelType_e::KEEP_ON_READ, 100);
    armor3d_sPtr_ = std::make_shared<NeChannel<interfaces::NeArmors3D_t>>(
        "armor3d", NeChannelType_e::KEEP_ON_READ, 1);
    aim_traj_sPtr_ = std::make_shared<NeChannel<interfaces::NeAimTraj_t>>(
        "aim_traj", NeChannelType_e::KEEP_ON_READ, 1);

    debug_frame_sPtr_ = std::make_shared<NeChannel<interfaces::NeDebugFrame_t>>(
        "debug_frame", NeChannelType_e::KEEP_ON_READ, 1);
  }

  ~NeChannels() = default;

  std::shared_ptr<NeChannel<interfaces::NeFrameInput_t>> frame_input_sPtr_;
  std::shared_ptr<NeChannel<interfaces::NeArmors2D_t>>   armor2d_sPtr_;
  std::shared_ptr<NeChannel<interfaces::NeImuData_t>>    imu_data_sPtr_;
  std::shared_ptr<NeChannel<interfaces::NeArmors3D_t>>   armor3d_sPtr_;
  std::shared_ptr<NeChannel<interfaces::NeAimTraj_t>>    aim_traj_sPtr_;
  std::shared_ptr<NeChannel<interfaces::NeDebugFrame_t>> debug_frame_sPtr_;
};

#define NV_CHANNELS NeChannels::GetInstance()

} // namespace ne_vision
