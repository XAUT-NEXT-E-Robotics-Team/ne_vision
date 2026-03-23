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

#include <string>

#include "ne_vision/interfaces/ne_aim_traj.hpp"
#include "ne_vision/ne_channals.hpp"
#include "ne_vision/utils/ne_channel.hpp"

#include "ne_vision/interfaces/ne_frame_input.hpp"
#include "ne_vision/interfaces/ne_armors_2d.hpp"
#include "ne_vision/interfaces/ne_debug_frame.hpp"
#include "ne_vision/interfaces/ne_armors_3d.hpp"

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
  using NeAimTraj_t = interfaces::NeAimTraj_t;
  using NeAimTrajCSPtr_t = std::shared_ptr<NeChannel<NeAimTraj_t>>;

  struct VisPack_t
  {
    // If a data put into the pack, the count will increase by 1.
    bool           is_matched = false;
    std::string    match_msg;
    int            data_count = 0;
    NeFrameInput_t input;
    NeArmors2D_t   armors_2d;
    NeArmors3D_t   armors_3d;
    NeAimTraj_t    aim_traj;
  };

public:
  explicit NeVisionVisualization(std::string name);
  ~NeVisionVisualization() = default;

  inline std::string GetName() { return name_; }

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
    NV_ASSERT(NV_CHANNELS.aim_traj_sPtr() != nullptr &&
              "AimTraj channel must not be null.");
    channels_.aim_traj_c_sPtr = NV_CHANNELS.aim_traj_sPtr();
  }

  void Draw();

private:
  void receiveAll();
  bool matchAll();

  void drawArmors2D(cv::Mat& frame);
  void drawArmors3D(cv::Mat& frame);
  void drawTrackerResult(cv::Mat& frame);

  cv::Point2d projectToImagePlane(const Eigen::Vector3d& point_3d);

  struct
  {
    NeFrameInputCsPtr_t input_c_sPtr = nullptr;
    NeArmors2DCsPtr_t   armors2d_c_sPtr = nullptr;
    NeDebugFrameCsPtr_t debug_frame_c_sPtr = nullptr;
    NeArmors3DCsPtr_t   armors3d_c_sPtr = nullptr;
    NeAimTrajCSPtr_t    aim_traj_c_sPtr = nullptr;
  } channels_;

  struct
  {
    std::deque<NeFrameInput_t> frame_inputs;
    std::deque<NeArmors2D_t>   armors_2ds;
    std::deque<NeArmors3D_t>   armors_3ds;
    std::deque<NeAimTraj_t>    aim_trajs;
  } vis_queue_;

  struct
  {
    cv::Mat                     camera_matrix_;
    cv::Mat                     dist_coeffs_;
    Eigen::Matrix3d             camera_matrix_eigen_;
    Eigen::Matrix<double, 5, 1> dist_coeffs_eigen_;
  } pro_param_;

  std::string name_;
  VisPack_t   vis_pack_;

  std::string out_video_path_;
  bool        record_video_ = false;
};

} // namespace ne_vision
