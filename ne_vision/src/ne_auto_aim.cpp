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

#include "ne_vision/ne_auto_aim.hpp"
#include "ne_vision/detector/ne_detector.hpp"
#include "ne_vision/interfaces/ne_debug_frame.hpp"
#include "ne_vision/interfaces/ne_frame_input.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/utils/ne_rerun_debug.hpp"
#include <cfloat>
#include <chrono>
#include <memory>

namespace ne_vision
{

/* === PUBLIC === */

NeAutoAim::NeAutoAim()
{
  // The step cannot be changed
  setupParameters();
  setupChannels();
}

NeAutoAim::~NeAutoAim() { Stop(); }

void NeAutoAim::UpdateFrame(const cv::Mat& frame, char our_color)
{
  if (is_running_ == false)
  {
    NV_WARN("AutoAim is not running, cannot update frame");
    return;
  }
  interfaces::NeFrameInput_t msg;
  msg.camera_name = "default";
  msg.cap_stamp = std::chrono::steady_clock::now();
  msg.frame = frame;

  if (our_color != 'B' && our_color != 'R')
  {
    NV_WARN(
        "Invalid our color! Is: {}({}). Set to B", our_color, (int)our_color);
    our_color = 'B';
  }

  msg.our_color = our_color;
  channels_.frame_input_sPtr_->Transmit(msg);
}

void NeAutoAim::UpdateImu(const Eigen::Vector3d&    acc,
                          const Eigen::Vector3d&    gyro,
                          const Eigen::Quaterniond& quat,
                          double                    delay_s)
{
  if (is_running_ == false)
  {
    NV_WARN("AutoAim is not running, cannot update IMU data");
    return;
  }
  interfaces::NeImuData_t msg;
  msg.receive_stamp = std::chrono::steady_clock::now();
  msg.receive_stamp +=
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(delay_s));
  msg.acc = acc;
  msg.gyro = gyro;
  msg.quat = quat;

  channels_.imu_data_sPtr_->Transmit(msg);
}

void NeAutoAim::Start(std::string config_file_path)
{
  NV_INFO("Starting NeAutoAim with config file: {}", config_file_path);

  if (!NeParam::Instance().LoadFromFile(config_file_path))
  {
    NV_ERROR("Aborting Start: Failed to load config file: {}",
             config_file_path);
    return;
  }

  NV_INFO("Loaded config from: {}", config_file_path);

  setupTasks();
  tasks_.detector_uPtr_->Start();
  tasks_.tracker_2d_uPtr_->Start();
  tasks_.debug_visualization_uPtr_->Start();
  is_running_ = true;
}

void NeAutoAim::AutoAim()
{
  if (is_running_ == false)
  {
    NV_WARN("AutoAim is not running, cannot auto aim");
    return;
  }
}

void NeAutoAim::DebugFrame(cv::Mat& frame)
{
  if (is_running_ == false)
  {
    frame = cv::Mat();
    NV_WARN("AutoAim is not running, cannot get debug frame");
    return;
  }

  interfaces::NeDebugFrame_t msg;
  if (!channels_.debug_frame_sPtr_->Receive(msg))
  {
    frame = cv::Mat();
    NV_WARN("Wait for data from {}", channels_.debug_frame_sPtr_->GetName());
    return;
  }
  if (msg.frame.Empty())
  {
    frame = cv::Mat();
    NV_WARN("Received empty frame from {}",
            channels_.debug_frame_sPtr_->GetName());
    return;
  }
  frame = msg.frame;
}

void NeAutoAim::Stop()
{
  tasks_.detector_uPtr_->Stop();
  is_running_ = false;
}

/* === PRIVATE === */

void NeAutoAim::setupParameters() {}

void NeAutoAim::setupChannels()
{
  channels_.frame_input_sPtr_ =
      std::make_shared<NeChannel<interfaces::NeFrameInput_t>>(
          "frame_input", NeChannelType_e::KEEP_ON_READ, 1);
  channels_.armor2d_sPtr_ =
      std::make_shared<NeChannel<interfaces::NeArmors2D_t>>(
          "armor2d", NeChannelType_e::KEEP_ON_READ, 1);
  channels_.imu_data_sPtr_ =
      std::make_shared<NeChannel<interfaces::NeImuData_t>>(
          "imu_data", NeChannelType_e::KEEP_ON_READ, 100);
  channels_.armor3d_sPtr_ =
      std::make_shared<NeChannel<interfaces::NeArmors3D_t>>(
          "armor3d", NeChannelType_e::KEEP_ON_READ, 1);

  channels_.debug_frame_sPtr_ =
      std::make_shared<NeChannel<interfaces::NeDebugFrame_t>>(
          "debug_frame", NeChannelType_e::KEEP_ON_READ, 1);
}

void NeAutoAim::setupTasks()
{
  task_objs_.detector_sPtr_ = std::make_shared<NeDetector>(
      "detector", channels_.frame_input_sPtr_, channels_.armor2d_sPtr_);
  tasks_.detector_uPtr_ =
      std::make_unique<NeTask>(task_objs_.detector_sPtr_->GetName(),
                               NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
                               channels_.frame_input_sPtr_,
                               task_objs_.detector_sPtr_.get(),
                               &NeDetector::Detect);

  task_objs_.debug_visualization_sPtr_ =
      std::make_shared<NeVisionVisualization>("debug_visualization",
                                              channels_.frame_input_sPtr_,
                                              channels_.debug_frame_sPtr_);
  task_objs_.debug_visualization_sPtr_->SetArmors2DChannel(
      channels_.armor2d_sPtr_);
  task_objs_.debug_visualization_sPtr_->SetArmors3DChannel(
      channels_.armor3d_sPtr_);
  tasks_.debug_visualization_uPtr_ =
      std::make_unique<NeTask>(task_objs_.debug_visualization_sPtr_->GetName(),
                               NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
                               channels_.frame_input_sPtr_,
                               task_objs_.debug_visualization_sPtr_.get(),
                               &NeVisionVisualization::Draw);

  task_objs_.tracker_2d_sPtr_ =
      std::make_shared<NeTracker2D>("tracker_2D",
                                    channels_.armor2d_sPtr_,
                                    channels_.imu_data_sPtr_,
                                    channels_.armor3d_sPtr_);
  tasks_.tracker_2d_uPtr_ =
      std::make_unique<NeTask>(task_objs_.tracker_2d_sPtr_->GetName(),
                               NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
                               channels_.armor2d_sPtr_,
                               task_objs_.tracker_2d_sPtr_.get(),
                               &NeTracker2D::Tarck2D);
}

} // namespace ne_vision
