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
//
// 双模式：
//   回调模式 - SetGimbalCallback / SetDebugCallback，由内部 NeTask 调度
//   轮询模式 - GetResult / GetDebugFrame，任意时刻无锁读取
//
// 使用示例:
//   NeAutoAim auto_aim;
//   auto_aim.SetGimbalCallback([](const NeAutoAimResult_t& r) { /* 发串口 */
//   }); auto_aim.SetDebugCallback([&]() { cv::Mat f; auto_aim.GetDebugFrame(f);
//   }); auto_aim.Start("config.yaml"); auto_aim.Spin(); // 阻塞主线程直到
//   Stop()

#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
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
  STOP,    // 默认状态，Start 后变为 IDLE 或 AIMING
  ERROR,   // 不可恢复错误
  WARNING, // 可忽视的异常
  IDLE,    // 运行中但无跟踪目标（电控不应采信）
  AIMING,  // 跟踪中并输出云台控制参考（电控应采信）
};

struct NeAutoAimResult_t
{
  NeAutoAimState_e state = NeAutoAimState_e::IDLE;

  struct
  {
    double yaw = 0;
    double pitch = 0;
    double yaw_v = 0;
    double pitch_v = 0;
  } control_ref;

  struct
  {
    // TODO: 目标信息
  } target_info;
};

class NeAutoAim final
{
public:
  // 云台控制回调：每当 gimbal_control_ref channel 有新数据时调用，由内部 NeTask
  // 驱动
  using GimbalCallback_t = std::function<void(const NeAutoAimResult_t&)>;

  // 调试帧回调：每当 debug_frame channel 有新数据时调用，void，不传出数据
  // 用户可在回调中调用 GetDebugFrame() 获取帧
  using DebugCallback_t = std::function<void()>;

  explicit NeAutoAim();
  ~NeAutoAim();

  /* === 数据输入 === */

  void UpdateFrame(const cv::Mat& frame, std::string camera_name = "default");

  void UpdateImu(const Eigen::Vector3d&    acc,
                 const Eigen::Vector3d&    gyro,
                 const Eigen::Quaterniond& quat);

  void UpdateRobotInfo(char our_color, double bullet_velocity);

  inline void UpdateTestImu()
  {
    UpdateImu(Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(),
              Eigen::Quaterniond::Identity());
  }

  /* === 回调注册（建议在 Start() 前调用） === */

  /**
   * @brief 注册云台控制回调
   * @note 线程安全，可在任意时刻调用（包括 Start 后）
   */
  void SetGimbalCallback(GimbalCallback_t cb);

  /**
   * @brief 注册调试帧回调（void，无数据传出）
   * @note 线程安全，可在任意时刻调用
   */
  void SetDebugCallback(DebugCallback_t cb);

  /* === 生命周期 === */

  void Start(std::string config_file_path);
  void Stop();

  inline bool IsRunning() const
  {
    return is_running_.load(std::memory_order_acquire);
  }

  /* === 非回调轮询接口（无锁，线程安全） === */

  /**
   * @brief 获取最新自瞄结果（无锁，与回调并用安全）
   */
  void GetResult(NeAutoAimResult_t& result) const;

  /**
   * @brief 获取最新调试帧（直接读取 channel，与调试回调并用安全）
   */
  void GetDebugFrame(cv::Mat& frame) const;

  /* === 主线程阻塞 === */

  /**
   * @brief 阻塞当前线程直到 Stop() 被调用
   */
  void Spin();

  const NeParam& Params();

private:
  // gimbal_result_uPtr_ task 体：读 gimbal_control_ref
  // channel，更新结果，触发回调
  void updateResult();

  // debug_dispatch_uPtr_ task 体：debug_frame channel 有新数据时触发调试回调
  void dispatchDebug();

  void setupTasks();

  /* === 内部任务 === */

  struct
  {
    std::unique_ptr<NeTask> detector_uPtr_;
    std::unique_ptr<NeTask> tracker_2d_uPtr_;
    std::unique_ptr<NeTask> tracker_3d_uPtr_;
    std::unique_ptr<NeTask> mashiro_planner_uPtr_;
    std::unique_ptr<NeTask> debug_visualization_uPtr_;
    std::unique_ptr<NeTask> gimbal_result_uPtr_;  // 监听 gimbal_control_ref
    std::unique_ptr<NeTask> debug_dispatch_uPtr_; // 监听 debug_frame
  } tasks_;

  struct
  {
    std::shared_ptr<NeDetector>            detector_sPtr_;
    std::shared_ptr<NeTracker2D>           tracker_2d_sPtr_;
    std::shared_ptr<NeTracker3D>           tracker_3d_sPtr_;
    std::shared_ptr<NeMashiroPlanner>      mashiro_planner_sPtr_;
    std::shared_ptr<NeVisionVisualization> debug_visualization_sPtr_;
  } task_objs_;

  /* === 状态 === */

  std::atomic<bool> is_running_{false};
  double            muzzel_velocity_ = 20.0;

  std::shared_ptr<NeAutoAimResult_t> result_sPtr_;
  mutable std::mutex                 result_mtx_;

  std::shared_ptr<GimbalCallback_t> gimbal_cb_sPtr_;
  std::shared_ptr<DebugCallback_t>  debug_cb_sPtr_;
  mutable std::mutex                cb_mtx_;

  /* === Spin() 阻塞机制 === */

  std::condition_variable stop_cv_;
  std::mutex              stop_mtx_;

  /* === Start/Stop 互斥（防止并发重入） === */

  std::mutex startup_mtx_;
};

} // namespace ne_vision
