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
// Detect

#include "ne_vision/detector/ne_detector.hpp"

#include <filesystem>
#include <exception>

#include "ne_vision/utils/ne_debug.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/utils/ne_code_profiler.hpp"
#include "ne_vision/ne_channals.hpp"

#ifdef NV_USE_TENSORRT
#include "ne_vision/detector/ne_cu/include/ne_cu_infer.h"
#else
#include "ne_vision/detector/openvino_infer.hpp"
#endif

#define MODEL_ROW 640.0f
#define MODEL_COL 640.0f

namespace ne_vision
{

NeDetector::NeDetector(const std::string& name) : name_(name)
{
  input_c_sPtr_     = NV_CHANNELS.frame_input_sPtr();
  armors_2d_c_sPtr_ = NV_CHANNELS.armor2d_sPtr();

  NV_ASSERT(input_c_sPtr_ != nullptr && armors_2d_c_sPtr_ != nullptr &&
            "input_c_sPtr_ and armor_2d_c_sPtr_ cannot be nullptr");

  std::string model_path_str =
      NV_PARAM["auto_aim"]["detector"]["model_path"].as<std::string>("model/0526");

  labels_to_str_ = {"7", "1", "2", "3", "4", "5", "outpost", "ignore", "base"};

  try
  {
#ifdef NV_USE_TENSORRT
    std::string model_engine = model_path_str + ".engine";
    if (!std::filesystem::exists(model_engine))
    {
      NV_ERROR("TensorRT engine not found: {}", model_engine);
      return;
    }
    infer_uPtr_ = std::make_unique<ne_cu::NeCudaInfer>();
    if (!infer_uPtr_->initModule(model_engine, 1, 22))
    {
      NV_ERROR("TensorRT initModule failed");
      infer_uPtr_.reset();
    }
    NV_INFO("Using TensorRT backend for detector.");
#else
    std::string model_xml = model_path_str + ".xml";
    std::string model_bin = model_path_str + ".bin";
    if (!std::filesystem::exists(model_xml) || !std::filesystem::exists(model_bin))
    {
      NV_ERROR("OpenVINO model not found: {}, {}", model_xml, model_bin);
      return;
    }
    infer_uPtr_ = std::make_unique<infer::OpenvinoInfer>(model_xml, model_bin, "AUTO");
    NV_INFO("Using OpenVINO backend for detector.");
#endif
  }
  catch (const std::exception& e)
  {
    NV_ERROR("Failed to create infer backend: {}", e.what());
  }
}

NeDetector::~NeDetector() {}

void NeDetector::Detect()
{
  NV_ASSERT(input_c_sPtr_ != nullptr && armors_2d_c_sPtr_ != nullptr &&
            "input_c_sPtr_ and armor_2d_c_sPtr_ cannot be nullptr");

  if (!input_c_sPtr_->Receive(frame_i_))
  {
    NV_WARN("Input frame is empty");
    return;
  }

  if (!infer_uPtr_)
  {
    NV_WARN("Infer backend not initialized, skipping detection.");
    return;
  }

  cv::Mat      frame    = frame_i_.frame;
  const size_t width    = frame.cols;
  const size_t height   = frame.rows;

  NeArmors2D_t armors_2d;
  armors_2d.cap_stamp    = frame_i_.cap_stamp;
  armors_2d.frame_height = height;
  armors_2d.frame_width  = width;

  std::vector<cv::Mat> batch = {frame};
  infer_uPtr_->dointerfence(batch, 0.45f, 0.65f);

  postProcess(width, height, armors_2d);
  armors_2d_c_sPtr_->Transmit(armors_2d);
}

void NeDetector::preProcess(cv::Mat& frame)
{
  cv::resize(frame, frame, cv::Size(MODEL_COL, MODEL_ROW));
}

void NeDetector::postProcess(size_t        width,
                             size_t        height,
                             NeArmors2D_t& armors_2d)
{
  if (infer_uPtr_->tmp_objects.empty())
    return;

#ifdef NV_USE_TENSORRT
  const float scale = std::min(MODEL_ROW / height, MODEL_COL / width);
  const float pad_x = (MODEL_COL - width  * scale) * 0.5f;
  const float pad_y = (MODEL_ROW - height * scale) * 0.5f;
#else
  const float scale_x = static_cast<float>(width)  / MODEL_COL;
  const float scale_y = static_cast<float>(height) / MODEL_ROW;
#endif

  for (auto& obj : infer_uPtr_->tmp_objects)
  {
    std::string label_str = labels_to_str_[obj.label];
    if (label_str == "ignore")
      continue;

    char armor_color;
    switch (obj.color)
    {
    case 1:  armor_color = 'R'; break;
    case 0:  armor_color = 'B'; break;
    default: armor_color = 'N'; break;
    }

    float pts[8];
    for (int k = 0; k < 8; ++k)
      pts[k] = obj.landmarks[k];

#ifdef NV_USE_TENSORRT
    for (int k = 0; k < 4; ++k)
    {
      pts[k * 2]     = std::clamp((pts[k * 2]     - pad_x) / scale, 0.0f, (float)(width  - 1));
      pts[k * 2 + 1] = std::clamp((pts[k * 2 + 1] - pad_y) / scale, 0.0f, (float)(height - 1));
    }
#else
    for (int k = 0; k < 4; ++k)
    {
      pts[k * 2]     *= scale_x;
      pts[k * 2 + 1] *= scale_y;
    }
#endif

    armors_2d.armors.emplace_back(label_str, armor_color,
                                  pts[0], pts[1],
                                  pts[2], pts[3],
                                  pts[6], pts[7],
                                  pts[4], pts[5]);
  }
}

} // namespace ne_vision
