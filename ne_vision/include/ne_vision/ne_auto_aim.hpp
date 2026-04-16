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
// Auto-aiming module for ne_vision.

#pragma once

#include "gtest/gtest.h"
#include <atomic>
#include <memory>
#include <mutex>

#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/utils/ne_task.hpp"

#include "ne_vision/detector/ne_detector.hpp"
#include "ne_vision/tracker/ne_tracker_2d.hpp"
#include "ne_vision/tracker/ne_tracker_3d.hpp"
#include "ne_vision/planner/ne_mashiro_planner.hpp"
#include "ne_vision/debug/ne_vision_visualization.hpp"

namespace ne_vision
{

enum class NeAutoAimState_e
{
  STOP,    // 所有任务正常启动后该标识将变为 AIMING 或 IDLE，是默认的状态
  ERROR,   // 错误：系统不能RUN了，比如配置错误或其他更严重的错误
  WARNING, // 警告：不该发生的情况发生了，一般可以忽视，但是一直产生的warn需要警惕，比如串口输入不合法
  IDLE, // 空闲：系统在运行，但没有跟踪目标（电控在这个状态下不应该采信自瞄的输出）
  AIMING, // 自瞄：系统在跟踪目标并输出云台控制参考（电控应该在这时候采信自瞄的输出）

  // 建议：
  // 应该只在IDLE与AIMING状态电控允许自瞄
};

struct NeAutoAimResult_t
{
  // 状态信息
  NeAutoAimState_e state = NeAutoAimState_e::IDLE;

  // 目标控制值，仅状态为AIMING时有效
  struct
  {
    double yaw = 0;
    double pitch = 0;
    double yaw_v = 0;
    double pitch_v = 0;
  } control_ref;

  // 目标信息，仅状态为AIMING时有效
  struct
  {
    // TODO: 现在不加
  } target_info;
};

class NeAutoAim final
{
public:
  explicit NeAutoAim();
  ~NeAutoAim();

  void UpdateFrame(const cv::Mat& frame, std::string camera_name = "default");
  void UpdateImu(const Eigen::Vector3d&    acc,
                 const Eigen::Vector3d&    gyro,
                 const Eigen::Quaterniond& quat);

  // 传入一个0的IMU
  inline void UpdateTestImu()
  {
    Eigen::Vector3d    acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d    gyro = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

    UpdateImu(acc, gyro, quat);
  }

  // 传入机器人基本信息
  void UpdateRobotInfo(char our_color, double bullet_velocity);

  inline bool IsRunning() const { return is_running_; }
  void        Start(std::string config_file_path);

  // autoaim 主要负责订阅控制数据和更新状态信息，需要在串口回调中调用
  void AutoAim();

  void DebugFrame(cv::Mat& frame);

  // 获取输出结果
  // 返回false指没有结果：具体是哪种情况需要根据状态区分
  inline void GetResult(NeAutoAimResult_t& result) const
  {
    std::lock_guard lock(mtx_);
    result = current_result_;
  }

  void Stop();

  void GetResult(NeAutoAimResult_t& result);

  const NeParam& Params();

private:
  void setupTasks();

  struct
  {
    std::unique_ptr<NeTask> detector_uPtr_;
    std::unique_ptr<NeTask> tracker_2d_uPtr_;
    std::unique_ptr<NeTask> tracker_3d_uPtr_;
    std::unique_ptr<NeTask> mashiro_planner_uPtr_;

    std::unique_ptr<NeTask> debug_visualization_uPtr_;
  } tasks_;

  struct
  {
    std::shared_ptr<NeDetector>       detector_sPtr_;
    std::shared_ptr<NeTracker2D>      tracker_2d_sPtr_;
    std::shared_ptr<NeTracker3D>      tracker_3d_sPtr_;
    std::shared_ptr<NeMashiroPlanner> mashiro_planner_sPtr_;

    std::shared_ptr<NeVisionVisualization> debug_visualization_sPtr_;
  } task_objs_;

  std::atomic_bool is_running_ = false;

  double muzzel_velocity_ = 20.0;

  NeAutoAimResult_t current_result_;
  NeAutoAimState_e  current_state_ = NeAutoAimState_e::STOP;

  mutable std::mutex mtx_;
};

} // namespace ne_vision
