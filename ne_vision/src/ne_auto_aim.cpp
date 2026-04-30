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
// Auto-aiming module 实现

#include <memory>
#include <mutex>

#include "ne_vision/ne_auto_aim.hpp"
#include "ne_vision/interfaces/ne_debug_frame.hpp"
#include "ne_vision/interfaces/ne_frame_input.hpp"
#include "ne_vision/interfaces/ne_gimbal_control_ref.hpp"
#include "ne_vision/ne_channals.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"

namespace ne_vision
{

/* === PUBLIC === */

NeAutoAim::NeAutoAim()
{
  result_sPtr_ = std::make_shared<NeAutoAimResult_t>();
}

NeAutoAim::~NeAutoAim() { Stop(); }

void NeAutoAim::UpdateFrame(const cv::Mat& frame, std::string camera_name)
{
  if (!is_running_.load(std::memory_order_acquire))
  {
    NV_WARN("AutoAim not running, drop frame");
    return;
  }
  interfaces::NeFrameInput_t msg;
  msg.camera_name = std::move(camera_name);
  msg.cap_stamp = std::chrono::steady_clock::now();
  msg.frame = frame;
  NV_CHANNELS.frame_input_sPtr()->Transmit(msg);
}

void NeAutoAim::UpdateImu(const Eigen::Vector3d&    acc,
                          const Eigen::Vector3d&    gyro,
                          const Eigen::Quaterniond& quat)
{
  if (!is_running_.load(std::memory_order_acquire))
  {
    NV_WARN("AutoAim not running, drop IMU");
    return;
  }
  interfaces::NeImuData_t msg;
  msg.receive_stamp = std::chrono::steady_clock::now();
  msg.acc = acc;
  msg.gyro = gyro;
  msg.quat = quat;
  NV_CHANNELS.imu_data_sPtr()->Transmit(msg);
}

void NeAutoAim::UpdateRobotInfo(char our_color, double bullet_velocity)
{
  if (bullet_velocity < 10)
  {
    NV_WARN("bullet_velocity {} too low, clamped to 10", bullet_velocity);
    bullet_velocity = 10.0;
  }
  else if (bullet_velocity > 30)
  {
    NV_WARN("bullet_velocity {} too high, clamped to 30", bullet_velocity);
    bullet_velocity = 30.0;
  }

  if (our_color != 'R' && our_color != 'B')
  {
    NV_WARN("our_color '{}' invalid, set to 'N'", our_color);
    our_color = 'N';
  }

  interfaces::NeRobotState_t msg;
  msg.our_color = our_color;
  msg.bullet_speed = bullet_velocity;
  NV_CHANNELS.robot_state_sPtr()->Transmit(msg);
}

/* === 回调注册 === */

void NeAutoAim::SetGimbalCallback(GimbalCallback_t cb)
{
  std::lock_guard lock(cb_mtx_);
  gimbal_cb_sPtr_ = std::make_shared<GimbalCallback_t>(std::move(cb));
}

void NeAutoAim::SetDebugCallback(DebugCallback_t cb)
{
  std::lock_guard lock(cb_mtx_);
  debug_cb_sPtr_ = std::make_shared<DebugCallback_t>(std::move(cb));
}

/* === 生命周期 === */

void NeAutoAim::Start(std::string config_file_path)
{
  std::lock_guard lock(startup_mtx_);

  if (is_running_.load(std::memory_order_acquire))
  {
    NV_WARN("NeAutoAim already running");
    return;
  }

  NV_INFO("Starting NeAutoAim with config: {}", config_file_path);

  if (!NeParam::Instance().LoadFromFile(config_file_path))
  {
    NV_ERROR("Aborting: failed to load config: {}", config_file_path);
    return;
  }

  setupTasks();

  tasks_.detector_uPtr_->Start();
  tasks_.tracker_2d_uPtr_->Start();
  tasks_.tracker_3d_uPtr_->Start();
  tasks_.mashiro_planner_uPtr_->Start();
  tasks_.debug_visualization_uPtr_->Start();
  tasks_.gimbal_result_uPtr_->Start();
  tasks_.debug_dispatch_uPtr_->Start();

  is_running_.store(true, std::memory_order_release);
  NV_INFO("NeAutoAim started");
}

void NeAutoAim::Stop()
{
  // exchange(false) 保证并发 Stop() 只执行一次
  if (!is_running_.exchange(false, std::memory_order_acq_rel))
    return;

  auto stopTask = [](auto& t) {
    if (t)
      t->Stop();
  };
  stopTask(tasks_.detector_uPtr_);
  stopTask(tasks_.tracker_2d_uPtr_);
  stopTask(tasks_.tracker_3d_uPtr_);
  stopTask(tasks_.mashiro_planner_uPtr_);
  stopTask(tasks_.debug_visualization_uPtr_);
  stopTask(tasks_.gimbal_result_uPtr_);
  stopTask(tasks_.debug_dispatch_uPtr_);

  stop_cv_.notify_all();
  NV_INFO("NeAutoAim stopped");
}

/* === 非回调轮询接口 === */

void NeAutoAim::GetResult(NeAutoAimResult_t& result) const
{
  std::lock_guard lock(result_mtx_);
  auto ptr = result_sPtr_;
  if (ptr)
    result = *ptr;
}

void NeAutoAim::GetDebugFrame(cv::Mat& frame) const
{
  if (!is_running_.load(std::memory_order_acquire))
  {
    frame = cv::Mat();
    return;
  }
  interfaces::NeDebugFrame_t msg;
  if (!NV_CHANNELS.debug_frame_sPtr()->Receive(msg, true))
  {
    frame = cv::Mat();
    return;
  }
  frame = msg.frame.Empty() ? cv::Mat() : static_cast<cv::Mat>(msg.frame);
}

/* === 主线程阻塞 === */

void NeAutoAim::Spin()
{
  std::unique_lock lock(stop_mtx_);
  stop_cv_.wait(
      lock, [this] { return !is_running_.load(std::memory_order_acquire); });
}

/* === PRIVATE === */

void NeAutoAim::updateResult()
{
  interfaces::NeGimbalControlRef_t msg;

  auto new_result = std::make_shared<NeAutoAimResult_t>();

  if (!NV_CHANNELS.gimbal_control_ref_sPtr()->Receive(msg))
  {
    new_result->state = NeAutoAimState_e::IDLE;
  }
  else if (!msg.valid)
  {
    new_result->state = NeAutoAimState_e::IDLE;
  }
  else
  {
    new_result->state = NeAutoAimState_e::AIMING;
    new_result->control_ref.yaw = msg.yaw_ref;
    new_result->control_ref.pitch = msg.pitch_ref;
    new_result->control_ref.yaw_v = msg.yaw_v_ref;
    new_result->control_ref.pitch_v = msg.pitch_v_ref;
  }

  std::lock_guard lock(result_mtx_);
  result_sPtr_ = new_result;

  std::shared_ptr<GimbalCallback_t> cb;
  {
    std::lock_guard lock2(cb_mtx_);
    cb = gimbal_cb_sPtr_;
  }
  if (cb && *cb)
    (*cb)(*new_result);
}

void NeAutoAim::dispatchDebug()
{
  // debug_frame channel 有新帧时唤醒（由 NeTask 调度）
  // 调试帧本身可通过 GetDebugFrame() 读取，回调只做通知
  std::shared_ptr<DebugCallback_t> cb;
  {
    std::lock_guard lock(cb_mtx_);
    cb = debug_cb_sPtr_;
  }
  if (cb && *cb)
    (*cb)();
}

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
                               NeTaskType_e::WAIT_FOR_INTERVAL,
                               10ms,
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

  // gimbal_result: 监听 gimbal_control_ref channel，驱动结果更新与云台回调
  tasks_.gimbal_result_uPtr_ =
      std::make_unique<NeTask>("gimbal_result",
                               NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
                               NV_CHANNELS.gimbal_control_ref_sPtr(),
                               this,
                               &NeAutoAim::updateResult);

  // debug_dispatch: 监听 debug_frame channel，驱动调试回调
  tasks_.debug_dispatch_uPtr_ =
      std::make_unique<NeTask>("debug_dispatch",
                               NeTaskType_e::WAIT_FOR_CHANNEL_DATA,
                               NV_CHANNELS.debug_frame_sPtr(),
                               this,
                               &NeAutoAim::dispatchDebug);
}

} // namespace ne_vision
