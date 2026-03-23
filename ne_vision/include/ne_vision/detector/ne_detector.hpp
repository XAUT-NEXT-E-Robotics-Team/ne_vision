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
// Detector
// Img >=> [ne_armor_2d](Point2D & Id ...)

#pragma once

#include <opencv2/core/utility.hpp>
#include <vector>
#include <memory>
#include <string>

#include "ne_vision/utils/ne_channel.hpp"

#include "ne_vision/interfaces/ne_frame_input.hpp"
#include "ne_vision/interfaces/ne_armors_2d.hpp"

#include "ne_vision/detector/openvino_infer.hpp"

namespace ne_vision
{

class NeDetector final
{

private:
  using NeFrameInput_t = interfaces::NeFrameInput_t;
  using NeArmors2D_t = interfaces::NeArmors2D_t;
  using NeFrameInputCsPtr_t = std::shared_ptr<NeChannel<NeFrameInput_t>>;
  using NeArmors2DCsPtr_t = std::shared_ptr<NeChannel<NeArmors2D_t>>;

public:
  explicit NeDetector(const std::string& name);
  ~NeDetector();

  std::string GetName() const { return name_; }

  void Detect();

private:
  void preProcess(cv::Mat& frame);
  void postProcess(size_t        width,
                   size_t        height,
                   NeArmors2D_t& armor_2d_s,
                   char          our_color);

  std::string name_;

  NeFrameInputCsPtr_t input_c_sPtr_;
  NeArmors2DCsPtr_t   armors_2d_c_sPtr_;

  NeFrameInput_t frame_i_;

  std::unique_ptr<infer::OpenvinoInfer> openvino_infer_uPtr_;

  std::array<std::string, 9> labels_to_str_;
};

} // namespace ne_vision
