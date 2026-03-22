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
// 3D跟踪器仅作为任务分配和处理接口，具体如何处理各个不同的目标，由model
// 模块进行，见model模块相关头文件

#pragma once

#include <memory>

#include "ne_vision/utils/ne_channel.hpp"

#include "ne_vision/interfaces/ne_armors_3d.hpp"
#include "ne_vision/interfaces/ne_imu_data.hpp"
#include "ne_vision/interfaces/ne_aim_traj.hpp"

#include "ne_vision/models/ne_sion_model.hpp"

namespace ne_vision
{

class NeTracker3D final
{

private:
  using NeArmors3D_t = interfaces::NeArmors3D_t;
  using NeImuData_t = interfaces::NeImuData_t;
  using NeAimTraj_t = interfaces::NeAimTraj_t;
  using NeArmors3DCSPtr_t = std::shared_ptr<NeChannel<NeArmors3D_t>>;
  using NeImuDataCSPtr_t = std::shared_ptr<NeChannel<NeImuData_t>>;
  using NeAimTrajCSPtr_t = std::shared_ptr<NeChannel<NeAimTraj_t>>;

public:
  explicit NeTracker3D(const std::string&       name,
                       const NeArmors3DCSPtr_t& armors_3d_c_sPtr,
                       const NeImuDataCSPtr_t&  imu_data_c_sPtr,
                       const NeAimTrajCSPtr_t&  aim_traj_c_sPtr);
  ~NeTracker3D() = default;

  void Track();

  inline std::string GetName() const { return name_; }

private:
  std::string name_;

  NeArmors3DCSPtr_t armors_3d_c_sPtr_;
  NeImuDataCSPtr_t  imu_data_c_sPtr_;
  NeAimTrajCSPtr_t  aim_traj_c_sPtr_;

  std::string current_tracking_aim_ = "NULL";

  // 上次接收的装甲板时间戳，用于判断当前收到的装甲板是否是新的
  // 如果不是最新的，进入仅预测模式
  std::chrono::steady_clock::time_point last_cap_stamp_;

  // 这是一个保存所有模型的联合体，如果有新的模型，要放进去
  std::variant<std::monostate, sion::NeSionModel> model_;

  struct
  {
    double lose_time = 3.0; // 认为丢失的时间阈值，单位秒 TODO: 写到参数里边去
  } param_;
};

} // namespace ne_vision
