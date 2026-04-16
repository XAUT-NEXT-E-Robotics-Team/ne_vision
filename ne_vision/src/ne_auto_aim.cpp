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

#include <cfloat>
#include <chrono>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "ne_vision/ne_auto_aim.hpp"
#include "ne_vision/detector/ne_detector.hpp"
#include "ne_vision/interfaces/ne_debug_frame.hpp"
#include "ne_vision/interfaces/ne_frame_input.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/ne_channals.hpp"

namespace ne_vision
{

/* === PUBLIC === */

NeAutoAim::NeAutoAim() {}

NeAutoAim::~NeAutoAim() { Stop(); }

void NeAutoAim::UpdateFrame(const cv::Mat& frame, std::string camera_name)
{
  if (is_running_ == false)
  {
    NV_WARN("AutoAim is not running, cannot update frame");
    return;
  }
  interfaces::NeFrameInput_t msg;
  msg.camera_name = camera_name;
  msg.cap_stamp = std::chrono::steady_clock::now();
  msg.frame = frame;

  NV_CHANNELS.frame_input_sPtr()->Transmit(msg);
}

void NeAutoAim::UpdateRobotInfo(char our_color, double bullet_velocity)
{
  // 弹速合理性检测和限幅
  if (bullet_velocity < 10)
  {
    NV_WARN("Received bullet velocity {} is too low, set to 10.0",
            bullet_velocity);
    bullet_velocity = 10.0;
  }
  else if (bullet_velocity > 30)
  {
    NV_WARN("Received bullet velocity {} is too high, set to 30.0",
            bullet_velocity);
    bullet_velocity = 30.0;
  }

  // 颜色合理性检测
  if (our_color != 'R' && our_color != 'B')
  {
    NV_WARN("Received our color {} is invalid, set to 'N'", our_color);
    our_color = 'N';
  }

  // 状态发消息
  interfaces::NeRobotState_t robot_state_msg;
  robot_state_msg.our_color = our_color;
  robot_state_msg.bullet_speed = bullet_velocity;
  NV_CHANNELS.robot_state_sPtr()->Transmit(robot_state_msg);
}

void NeAutoAim::UpdateImu(const Eigen::Vector3d&    acc,
                          const Eigen::Vector3d&    gyro,
                          const Eigen::Quaterniond& quat)
{
  if (is_running_ == false)
  {
    NV_WARN("AutoAim is not running, cannot update IMU data");
    return;
  }
  interfaces::NeImuData_t msg;
  msg.receive_stamp = std::chrono::steady_clock::now();
  msg.acc = acc;
  msg.gyro = gyro;
  msg.quat = quat;

  NV_CHANNELS.imu_data_sPtr()->Transmit(msg);

  interfaces::NeRobotState_t robot_state_msg;
  robot_state_msg.bullet_speed = 20.0; // TODO: 从实际数据获取弹速
  NV_CHANNELS.robot_state_sPtr()->Transmit(robot_state_msg);
}

void NeAutoAim::Start(std::string config_file_path)
{
  std::lock_guard lock(mtx_);
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
  tasks_.tracker_3d_uPtr_->Start();
  tasks_.mashiro_planner_uPtr_->Start();
  tasks_.debug_visualization_uPtr_->Start();
  is_running_ = true;
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
  if (!NV_CHANNELS.debug_frame_sPtr()->Receive(msg))
  {
    frame = cv::Mat();
    NV_INFO("Wait for data from {}", NV_CHANNELS.debug_frame_sPtr()->GetName());
    return;
  }
  if (msg.frame.Empty())
  {
    frame = cv::Mat();
    NV_WARN("Received empty frame from {}",
            NV_CHANNELS.debug_frame_sPtr()->GetName());
    return;
  }
  frame = msg.frame;
}

void NeAutoAim::AutoAim()
{
  std::lock_guard lock(mtx_);
  if (is_running_ == false)
  {
    // 没运行
    // TODO: 使用心跳机制实际检测
    NV_WARN("AutoAim is not running, cannot get result");
    current_result_.control_ref.yaw = 0;
    current_result_.control_ref.pitch = 0;
    current_result_.control_ref.yaw_v = 0;
    current_result_.control_ref.pitch_v = 0;
    current_state_ = NeAutoAimState_e::STOP;
    return;
  }
  else
  {
    current_state_ = NeAutoAimState_e::IDLE;
  }

  interfaces::NeGimbalControlRef_t msg;
  if (!NV_CHANNELS.gimbal_control_ref_sPtr()->Receive(msg))
  {
    // 无消息
    NV_INFO("Wait for data from {}",
            NV_CHANNELS.gimbal_control_ref_sPtr()->GetName());
    current_result_.control_ref.yaw = 0;
    current_result_.control_ref.pitch = 0;
    current_result_.control_ref.yaw_v = 0;
    current_result_.control_ref.pitch_v = 0;
    current_state_ = NeAutoAimState_e::IDLE;
    return;
  }
  if (!msg.valid)
  {
    // 无目标
    current_result_.control_ref.yaw = 0;
    current_result_.control_ref.pitch = 0;
    current_result_.control_ref.yaw_v = 0;
    current_result_.control_ref.pitch_v = 0;
    current_state_ = NeAutoAimState_e::IDLE;
    return;
  }

  current_result_.control_ref.yaw = msg.yaw_ref;
  current_result_.control_ref.pitch = msg.pitch_ref;
  current_result_.control_ref.yaw_v = msg.yaw_v_ref;
  current_result_.control_ref.pitch_v = msg.pitch_v_ref;
  current_state_ = NeAutoAimState_e::AIMING;

  current_result_.state = current_state_;
  return;
}

void NeAutoAim::Stop()
{
  std::lock_guard lock(mtx_);
  tasks_.detector_uPtr_->Stop();
  tasks_.tracker_2d_uPtr_->Stop();
  tasks_.tracker_3d_uPtr_->Stop();
  tasks_.mashiro_planner_uPtr_->Stop();
  tasks_.debug_visualization_uPtr_->Stop();
  is_running_ = false;
}

void NeAutoAim::GetResult(NeAutoAimResult_t& result)
{
  std::lock_guard lock(mtx_);
  result = current_result_;
}

/* === PRIVATE === */

void NeAutoAim::setupTasks()
{
  task_objs_.detector_sPtr_ = std::make_shared<NeDetector>("detector");
  tasks_.detector_uPtr_ =
      std::make_unique<NeTask>(task_objs_.detector_sPtr_->GetName(),
                               NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
                               NV_CHANNELS.frame_input_sPtr(),
                               task_objs_.detector_sPtr_.get(),
                               &NeDetector::Detect);

  task_objs_.tracker_2d_sPtr_ = std::make_shared<NeTracker2D>("tracker_2D");
  tasks_.tracker_2d_uPtr_ =
      std::make_unique<NeTask>(task_objs_.tracker_2d_sPtr_->GetName(),
                               NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
                               NV_CHANNELS.armor2d_sPtr(),
                               task_objs_.tracker_2d_sPtr_.get(),
                               &NeTracker2D::Tarck2D);

  task_objs_.tracker_3d_sPtr_ = std::make_shared<NeTracker3D>("tracker_3D");
  tasks_.tracker_3d_uPtr_ =
      std::make_unique<NeTask>(task_objs_.tracker_3d_sPtr_->GetName(),
                               NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
                               NV_CHANNELS.armor3d_sPtr(),
                               task_objs_.tracker_3d_sPtr_.get(),
                               &NeTracker3D::Track);

  task_objs_.mashiro_planner_sPtr_ =
      std::make_shared<NeMashiroPlanner>("mashiro_planner");
  tasks_.mashiro_planner_uPtr_ =
      std::make_unique<NeTask>(task_objs_.mashiro_planner_sPtr_->GetName(),
                               NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
                               NV_CHANNELS.aim_state_sPtr(),
                               task_objs_.mashiro_planner_sPtr_.get(),
                               &NeMashiroPlanner::Plan);

  task_objs_.debug_visualization_sPtr_ =
      std::make_shared<NeVisionVisualization>("debug_visualization");
  task_objs_.debug_visualization_sPtr_->AddArmors2DData();
  task_objs_.debug_visualization_sPtr_->AddArmors3DData();
  task_objs_.debug_visualization_sPtr_->AddAimTrajData();
  task_objs_.debug_visualization_sPtr_->AddGimbalControlRefData();
  tasks_.debug_visualization_uPtr_ =
      std::make_unique<NeTask>(task_objs_.debug_visualization_sPtr_->GetName(),
                               NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
                               NV_CHANNELS.frame_input_sPtr(),
                               task_objs_.debug_visualization_sPtr_.get(),
                               &NeVisionVisualization::Draw);
}

} // namespace ne_vision
