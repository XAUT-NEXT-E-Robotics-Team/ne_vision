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

#include <string>
#include <filesystem>
#include <exception>

#include "ne_vision/utils/ne_debug.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/utils/ne_code_profiler.hpp"

#include "ne_vision/ne_channals.hpp"

#define USE_TENSORRT

#ifdef USE_TENSORRT
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
  input_c_sPtr_ = NV_CHANNELS.frame_input_sPtr();
  armors_2d_c_sPtr_ = NV_CHANNELS.armor2d_sPtr();

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
  // bool bin_exists = std::filesystem::exists(model_bin);
  // bool xml_exists = std::filesystem::exists(model_xml);

  // if (!bin_exists || !xml_exists)
  // {
  //   NV_ERROR("Model files not found! xml: {}, bin: {}", model_xml, model_bin);
  //   NV_ERROR("Current working directory: {}",
  //            std::filesystem::current_path().string());
  //   return;
  // }

  // @julyfun： 水晶
  //tennsorrt 与openvino的对应的转换关系是一样的
  labels_to_str_ = {"7", "1", "2", "3", "4", "5", "outpost", "ignore", "base"};

  // New Infer object safely
  try
  {
#ifdef USE_TENSORRT
    std::string model_engine = model_path_str + ".engine";
    infer_uPtr_ = std::make_unique<ne_cu::NeCudaInfer>();
    if(!infer_uPtr_ -> initModule(model_engine,1,22))
    {
     return; 
    }
    NV_INFO("Using TensorRT backend for detector.");
#else
    infer_uPtr_ =
        std::make_unique<infer::OpenvinoInfer>(model_xml, model_bin, "AUTO");
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

  // std::chrono::steady_clock::time_point now =
  // std::chrono::steady_clock::now();

  if (!input_c_sPtr_->Receive(frame_i_))
  {
    NV_WARN("Input frame is empty");
    return;
  }//收到帧

  cv::Mat frame = frame_i_.frame;

  NeArmors2D_t armors_2d;
  const size_t width = frame.cols;
  const size_t height = frame.rows;

  armors_2d.cap_stamp = frame_i_.cap_stamp;
  armors_2d.frame_height = height;
  armors_2d.frame_width = width;

  if (!infer_uPtr_)
  {
    NV_WARN("Infer backend not initialized, skipping detection.");
    return;
  }
  //xd,这里必须要
  #ifndef USE_TENSORRT
  {
  preProcess(frame);
  // 这里的detect_color暂时没用，我们需要拿到颜色取做装甲板闪烁续命
  infer_uPtr_->infer(frame, 0);
  postProcess(width, height, armors_2d, frame_i_.our_color);

  armors_2d_c_sPtr_->Transmit(armors_2d);
  }
  #endif
  std::vector<cv::Mat> batch;
  batch.push_back(frame);
  float conf_thresold = 0.65;
  float nms_thresold = 0.45;
  auto cu_results = infer_uPtr_ -> dointerfence(batch,nms_thresold,conf_thresold);
  if(cu_results.empty() || cu_results[0].empty())
  {
    return;
  }//推理为空
  const auto &tmp_results = cu_results[0];
  cv::Mat vis = frame.clone();
  const float scale = std::min(MODEL_ROW / width,MODEL_COL / height);
  const float pad_x = (MODEL_ROW - width * scale) * 0.5f;
  const float pad_y = (MODEL_COL - height * scale) * 0.5f;
  auto unletterbox = [&](float &x,float &y)
  {
    x = (x - pad_x) / scale;
    y = (y - pad_y) / scale;
    x = std::clamp(x,0.0f,static_cast<float>(width - 1));
    y = std::clamp(y,0.0f,static_cast<float>(height - 1));
    
  };
  for(auto &tmp: tmp_results)
  {
    float pts[8];
    for(int i = 0;i < 8;++i)
    {
      pts[i] = tmp.landmarks[i];
    }
    for(int i = 0;i < 8; ++i)
    {
      unletterbox(pts[i * 2],pts[i * 2 + 1]);
    }
    std::string armor_label = labels_to_str_[tmp.label];
    if(armor_label == "ignore")
    {
      continue;
    }
    char armor_color;
    switch (tmp.color)
    {
    case 1: armor_color = 'R'; break;
    case 0: armor_color = 'B'; break;
    default: armor_color = 'N'; break;
    } // 3
    // char aim_color == 'R' ? 'B' : 'R';
    // if (armor_color != aim_color)
    //   continue;
    NV_INFO("第 {}",pts[0]);
    NV_INFO("D  {}",pts[1]);
    NV_INFO("A {}",pts[2]);
    NV_INFO("S {}",pts[3]);
    
    // const cv::Point2f p0(pts[0], pts[1]);
    // const cv::Point2f p1(pts[2], pts[3]);
    // const cv::Point2f p2(pts[4], pts[5]);
    // const cv::Point2f p3(pts[6], pts[7]);
    // cv::line(frame, p0, p1, cv::Scalar(0, 255, 0), 2);
    // cv::line(frame, p1, p2, cv::Scalar(0, 255, 0), 2);
    // cv::line(frame, p2, p3, cv::Scalar(0, 255, 0), 2);
    // cv::line(frame, p3, p0, cv::Scalar(0, 255, 0), 2);
    armors_2d.armors.emplace_back(armor_label,
                                  armor_color,
                                  pts[0],
                                  pts[1],
                                  pts[2],
                                  pts[3],
                                  pts[6],
                                  pts[7],
                                  pts[4],
                                  pts[5]);
    armors_2d_c_sPtr_->Transmit(armors_2d);
    
  }
  // std::chrono::steady_clock::time_point end =
  // std::chrono::steady_clock::now(); double duration_ms =
  //     std::chrono::duration<double, std::milli>(end - now).count();
  // NV_DEBUG("Detector took {:.2f} ms", duration_ms);
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
  if (infer_uPtr_->tmp_objects.empty())
  {
    armors_2d.armors.clear();
    return;
  }

  for (auto& obj : infer_uPtr_->tmp_objects)
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
    case 1: armor_color = 'R'; break;
    case 0: armor_color = 'B'; break;
    default: armor_color = 'N'; break; // 3
    }

    char aim_color = out_color == 'R' ? 'B' : 'R';
    if (armor_color != aim_color)
      continue;

    // 构造将自动计算中心
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
