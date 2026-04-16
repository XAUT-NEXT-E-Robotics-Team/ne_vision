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
// Visualization module for debugging purposes.
//

#pragma once

#include <deque>
#include <string>

#include "ne_vision/interfaces/ne_aim_state.hpp"
#include "ne_vision/ne_channals.hpp"
#include "ne_vision/utils/ne_channel.hpp"

#include "ne_vision/interfaces/ne_frame_input.hpp"
#include "ne_vision/interfaces/ne_armors_2d.hpp"
#include "ne_vision/interfaces/ne_debug_frame.hpp"
#include "ne_vision/interfaces/ne_armors_3d.hpp"
#include "ne_vision/interfaces/ne_gimbal_control_ref.hpp"

#include "ne_vision/debug/ne_data_scope.hpp"

#include "ne_vision/ne_auto_aim_tf.hpp"

namespace ne_vision
{

class NeVisionVisualization final
{
private:
  using NeFrameInput_t = interfaces::NeFrameInput_t;
  using NeFrameInputCsPtr_t = std::shared_ptr<NeChannel<NeFrameInput_t>>;
  using NeArmors2D_t = interfaces::NeArmors2D_t;
  using NeArmors2DCsPtr_t = std::shared_ptr<NeChannel<NeArmors2D_t>>;
  using NeArmors3D_t = interfaces::NeArmors3D_t;
  using NeArmors3DCsPtr_t = std::shared_ptr<NeChannel<NeArmors3D_t>>;
  using NeDebugFrame_t = interfaces::NeDebugFrame_t;
  using NeDebugFrameCsPtr_t = std::shared_ptr<NeChannel<NeDebugFrame_t>>;
  using NeAimState_t = interfaces::NeAimState_t;
  using NeAimStateCSPtr_t = std::shared_ptr<NeChannel<NeAimState_t>>;
  using NeGimbalControlRef_t = interfaces::NeGimbalControlRef_t;
  using NeGimbalControlRefCSPtr_t =
      std::shared_ptr<NeChannel<NeGimbalControlRef_t>>;

  struct VisPack_t
  {
    // If a data put into the pack, the count will increase by 1.
    bool                 is_matched = false;
    std::string          match_msg;
    int                  data_count = 0;
    NeFrameInput_t       input;
    NeArmors2D_t         armors_2d;
    NeArmors3D_t         armors_3d;
    NeAimState_t         aim_state;
    NeGimbalControlRef_t gimbal_control_ref;

    // Control stamp 与 cap stamp 配对的控制ref
    NeGimbalControlRef_t control_stamp_gimbal_control_ref;
  };

public:
  explicit NeVisionVisualization(std::string name);
  ~NeVisionVisualization() = default;

  inline std::string GetName() { return name_; }

  // 需要可视化（或发布）哪些消息，就调用对应的Add函数
  inline void AddArmors2DData()
  {
    NV_ASSERT(NV_CHANNELS.armor2d_sPtr() != nullptr &&
              "Armors2D channel must not be null.");
    channels_.armors2d_c_sPtr = NV_CHANNELS.armor2d_sPtr();
  }

  inline void AddArmors3DData()
  {
    NV_ASSERT(NV_CHANNELS.armor3d_sPtr() != nullptr &&
              "Armors3D channel must not be null.");
    channels_.armors3d_c_sPtr = NV_CHANNELS.armor3d_sPtr();
  }

  inline void AddAimTrajData()
  {
    NV_ASSERT(NV_CHANNELS.aim_state_sPtr() != nullptr &&
              "AimTraj channel must not be null.");
    channels_.aim_state_c_sPtr = NV_CHANNELS.aim_state_sPtr();
  }

  inline void AddGimbalControlRefData()
  {
    NV_ASSERT(NV_CHANNELS.gimbal_control_ref_sPtr() != nullptr &&
              "GimbalControlRef channel must not be null.");
    channels_.gimbal_control_ref_c_sPtr = NV_CHANNELS.gimbal_control_ref_sPtr();
  }

  void Draw();

private:
  void receiveAll();
  bool matchAll();

  void drawArmors2D(cv::Mat& frame);
  void drawArmors3D(cv::Mat& frame);
  void drawTrackerResult(cv::Mat& frame);
  void drawGimbalControlRef(cv::Mat& frame);

  void computeExpectedBallistic2d(double gimbal_expected_pitch,
                                  std::vector<cv::Point2d>& expected_points_2d);

  struct
  {
    NeFrameInputCsPtr_t       input_c_sPtr = nullptr;
    NeArmors2DCsPtr_t         armors2d_c_sPtr = nullptr;
    NeDebugFrameCsPtr_t       debug_frame_c_sPtr = nullptr;
    NeArmors3DCsPtr_t         armors3d_c_sPtr = nullptr;
    NeAimStateCSPtr_t         aim_state_c_sPtr = nullptr;
    NeGimbalControlRefCSPtr_t gimbal_control_ref_c_sPtr = nullptr;
  } channels_;

  struct
  {
    std::deque<NeFrameInput_t>       frame_inputs;
    std::deque<NeArmors2D_t>         armors_2ds;
    std::deque<NeArmors3D_t>         armors_3ds;
    std::deque<NeAimState_t>         aim_states;
    std::deque<NeGimbalControlRef_t> gimbal_control_refs;
  } vis_queue_;

  struct
  {
    std::vector<Eigen::Vector3d> small_armor;
    std::vector<Eigen::Vector3d> large_armor;
    std::vector<Eigen::Vector3d> outpost_armor;
  } armor_param_;

  NeDataScopeManager scope_mng_;

  std::string name_;
  VisPack_t   vis_pack_;

  std::string out_video_path_;
  bool        record_video_ = false;

  std::unique_ptr<NeAutoAimTf> auto_aim_tf_uPtr_;
};

} // namespace ne_vision
