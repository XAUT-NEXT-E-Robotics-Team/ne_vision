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

#include "ne_vision/utils/ne_debug.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"

#include "ne_vision/detector/openvino_infer.hpp"
#include <string>
#include <filesystem>
#include <exception>

#define MODEL_ROW 640
#define MODEL_COL 640

namespace ne_vision
{

NeDetector::NeDetector(const std::string&         name,
                       const NeFrameInputCsPtr_t& input_c_sPtr,
                       const NeArmors2DCsPtr_t&   armors_2d_c_sPtr)
    : name_(name), input_c_sPtr_(input_c_sPtr),
      armors_2d_c_sPtr_(armors_2d_c_sPtr)
{
  NV_ASSERT(input_c_sPtr_ != nullptr && armors_2d_c_sPtr_ != nullptr &&
            "input_c_sPtr_ and armor_2d_c_sPtr_ cannot be nullptr");

  /* === The param about detector === */
  // The bin, onnx, and xml file should have the same name.
  // You don't need to specify the extension.
  // You should    : model/0708
  // You shouldn't : model/0708.onnx

  // Read and validate model paths
  std::string model_path_str =
      NV_PARAM["auto_aim"]["detector"]["model_path"].as<std::string>(
          "model/0526");

  // model_path_str is like "model/0708"
  // If we are in Godot, we might need to handle resource paths or absolute
  // paths.

  std::string model_bin = model_path_str + ".bin";
  std::string model_xml = model_path_str + ".xml";

  // Check if files exist
  bool bin_exists = std::filesystem::exists(model_bin);
  bool xml_exists = std::filesystem::exists(model_xml);

  if (!bin_exists || !xml_exists)
  {
    NV_ERROR("Model files not found! xml: {}, bin: {}", model_xml, model_bin);
    NV_ERROR("Current working directory: {}",
             std::filesystem::current_path().string());
    return;
  }

  // @julyfun： 水晶
  labels_to_str_ = {"7", "1", "2", "3", "4", "5", "outpost", "ignore", "base"};

  // New Infer object safely
  try
  {
    openvino_infer_uPtr_ =
        std::make_unique<infer::OpenvinoInfer>(model_xml, model_bin, "AUTO");
  }
  catch (const std::exception& e)
  {
    NV_ERROR("Failed to create OpenvinoInfer: {}", e.what());
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

  cv::Mat frame = frame_i_.frame;

  NeArmors2D_t armors_2d;
  const size_t width = frame.cols;
  const size_t height = frame.rows;

  armors_2d.cap_stamp = frame_i_.cap_stamp;
  armors_2d.frame_height = height;
  armors_2d.frame_width = width;

  if (!openvino_infer_uPtr_)
  {
    NV_WARN("OpenVINO infer object not initialized, skipping detection.");
    return;
  }

  preProcess(frame);
  // 这里的detect_color暂时没用，我们需要拿到颜色取做装甲板闪烁续命
  openvino_infer_uPtr_->infer(frame, 0);
  postProcess(width, height, armors_2d, frame_i_.our_color);

  armors_2d_c_sPtr_->Transmit(armors_2d);
}

// 预处理
void NeDetector::preProcess(cv::Mat& frame)
{
  cv::resize(frame, frame, cv::Size(MODEL_COL, MODEL_ROW));
}

// 后处理
void NeDetector::postProcess(size_t        width,
                             size_t        height,
                             NeArmors2D_t& armors_2d,
                             char          out_color)
{
  if (openvino_infer_uPtr_->tmp_objects.empty())
  {
    armors_2d.armors.clear();
    return;
  }

  for (auto& obj : openvino_infer_uPtr_->tmp_objects)
  {
    double scale_x = static_cast<double>(width) / 640.0;
    double scale_y = static_cast<double>(height) / 640.0;

    std::string labels_str = labels_to_str_[obj.label];

    if (labels_str == "ignore")
      continue;

    // 在倪那边，0是红色，1是蓝色，3是无颜色
    char armor_color;
    switch (obj.color)
    {
    case 0: armor_color = 'R'; break;
    case 1: armor_color = 'B'; break;
    default: armor_color = 'N'; break; // 3
    }

    armors_2d.armors.emplace_back(labels_str,
                                  armor_color,
                                  obj.landmarks[0] * scale_x,
                                  obj.landmarks[1] * scale_y,
                                  obj.landmarks[2] * scale_x,
                                  obj.landmarks[3] * scale_y,
                                  obj.landmarks[6] * scale_x,
                                  obj.landmarks[7] * scale_y,
                                  obj.landmarks[4] * scale_x,
                                  obj.landmarks[5] * scale_y);
  }
}

} // namespace ne_vision
