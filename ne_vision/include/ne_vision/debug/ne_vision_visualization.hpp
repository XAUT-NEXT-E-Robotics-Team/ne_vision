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

#include "ne_vision/utils/ne_channel.hpp"

#include "ne_vision/interfaces/ne_frame_input.hpp"
#include "ne_vision/interfaces/ne_armors_2d.hpp"
#include "ne_vision/interfaces/ne_debug_frame.hpp"
#include "ne_vision/interfaces/ne_armors_3d.hpp"

namespace ne_vision
{

/**
 * HOW IS WORKING?
 *
 * Receive data from all channels and store them in the visualization
 * queue. The queue will be cleared if the size exceeds the maximum limit. The
 * data in the queue will be matched based on the timestamp of the input
 * frame. If the timestamp of the data is less than the input frame, it will
 * be discarded. If the timestamp of the data is equal or later than the input
 * frame, it will be stored in the visualization pack. If the timestamp of the
 * data is greater than the input frame, it will be kept in the queue for the
 * next matching. The visualization pack will be marked as matched if all data
 * is matched successfully. The visualization pack will be cleared if the
 * matching fails. The visualization pack will be transmitted to the debug
 * frame channel after drawing.
 *
 * The matching process is based on the timestamp of the input frame,
 * which is captured by the camera. The timestamp of the data is also captured
 * by the camera, so they should be synchronized. However, there might be some
 * delay in the processing of the data, so the timestamp of the data might be
 * slightly different from the timestamp of the input frame. Therefore, we
 * need to use a queue to store the data and match them based on the
 * timestamp. The matching process will ensure that the data is matched
 * correctly and the visualization is accurate.
 */
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

  struct VisPack_t
  {
    // If a data put into the pack, the count will increase by 1.
    bool           is_matched = false;
    std::string    match_msg;
    int            data_count = 0;
    NeFrameInput_t input;
    NeArmors2D_t   armors_2d;
    NeArmors3D_t   armors_3d;
  };

public:
  explicit NeVisionVisualization(std::string                name,
                                 const NeFrameInputCsPtr_t& input_c_sPtr,
                                 const NeDebugFrameCsPtr_t& debug_frame_c_sPtr);
  ~NeVisionVisualization() = default;

  inline std::string GetName() { return name_; }

  // Set your channels before calling Draw().
  // If you don't want to visualize some data, just don't set the channel. The
  // visualization will be based on the data you have set.
  inline void SetArmors2DChannel(const NeArmors2DCsPtr_t& armors_2d_c_sPtr)
  {
    NV_ASSERT(armors_2d_c_sPtr != nullptr &&
              "Armors2D channel must not be null.");
    channels_.armors2d_c_sPtr = armors_2d_c_sPtr;
  }

  inline void SetArmors3DChannel(const NeArmors3DCsPtr_t& armors_3d_c_sPtr)
  {
    NV_ASSERT(armors_3d_c_sPtr != nullptr &&
              "Armors3D channel must not be null.");
    channels_.armors3d_c_sPtr = armors_3d_c_sPtr;
  }
  void Draw();

private:
  void receiveAll();
  bool matchAll();

  void drawArmors2D(cv::Mat& frame);
  void drawArmors3D(cv::Mat& frame);

  struct
  {
    NeFrameInputCsPtr_t input_c_sPtr = nullptr;
    NeArmors2DCsPtr_t   armors2d_c_sPtr = nullptr;
    NeDebugFrameCsPtr_t debug_frame_c_sPtr = nullptr;
    NeArmors3DCsPtr_t   armors3d_c_sPtr = nullptr;
  } channels_;

  struct
  {
    std::deque<NeFrameInput_t> frame_inputs;
    std::deque<NeArmors2D_t>   armors_2ds;
    std::deque<NeArmors3D_t>   armors_3ds;
  } vis_queue_;

  struct
  {
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
  } pro_param_;

  std::string name_;
  VisPack_t   vis_pack_;

  std::string out_video_path_;
  bool        record_video_ = false;
};

} // namespace ne_vision
