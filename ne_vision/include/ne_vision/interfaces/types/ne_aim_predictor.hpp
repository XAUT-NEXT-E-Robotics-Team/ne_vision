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

// Description: AI 写的
//
// 目标云台轨迹预测器父类（接口）
// 用于替代传统直接传输固定点集的方式，通过面向对象的方法，让规划器动态调用预测函数，
// 并能在传输中保证线程安全，作为轨迹的生成器给 MPC
// 规划器提供每个预测窗口步长内的真实角度约束。

#pragma once

#include <chrono>
#include <mutex>
#include <shared_mutex>

#include <Eigen/Dense>

#include "ne_vision/interfaces/ne_imu_data.hpp"

namespace ne_vision
{
namespace interfaces
{

/**
 * @brief 云台目标轨迹预测器基类
 *
 * 作为轨迹的“生成器”而非具体点集的拷贝容器。
 * 其将被 Model 层中新编写的具体动力学或运动学模型所继承，在多线程间传输使用。
 */
class NeAimPredictorBase
{
public:
  NeAimPredictorBase() = default;

  /**
   * @brief 构造函数
   *
   * @param cap_stamp 预测器计算依赖的原始视觉捕捉(Capture)系统时刻
   * @param imu_stamp 预测器建立或最后一次参数更新时的最新IMU时间(并非当前时间)
   */
  NeAimPredictorBase(std::chrono::steady_clock::time_point cap_stamp,
                     std::chrono::steady_clock::time_point imu_stamp)
      : cap_stamp_(cap_stamp), imu_stamp_(imu_stamp)
  {
  }

  virtual ~NeAimPredictorBase() = default;

  /**
   * @brief 核心预测生成接口
   *
   * 给定一个相对于时间起点的流逝时间，预测出届时云台所需追踪目标的绝对 Yaw 和
   * Pitch 期望值。 此虚函数由子类（具体物理/数学模型）实现内部推演机制。
   *
   * @param dt 相对于 update_stamp 或预测起始状态的步长时间间隔（秒）
   * @param[out] yaw_target 预测出的目标云台偏航角 (Yaw)
   * @param[out] pitch_target 预测出的目标云台俯仰角 (Pitch)
   * @return true 预测成功
   * @return false 预测失败（如时间远超置信范围无法预测等情况）
   */
  virtual bool Predict(double             dt,
                       const NeImuData_t& imu_data,
                       Eigen::Vector3d&   target_position,
                       double&            target_yaw) const = 0;
  /**
   * @brief 获取拍摄/捕捉时刻的时间戳（线程安全）
   */
  std::chrono::steady_clock::time_point GetCapStamp() const
  {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return cap_stamp_;
  }

  /**
   * @brief 获取当前预测器建立/更新时的系统时间戳（线程安全）
   */
  std::chrono::steady_clock::time_point GetImuStamp() const
  {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return imu_stamp_;
  }

  /**
   * @brief 线程安全地更新时间戳标定
   * 在使用复用对象而不销毁重建时，可以保证并发更新的读写安全。
   */
  void UpdateTimestamps(std::chrono::steady_clock::time_point cap_stamp,
                        std::chrono::steady_clock::time_point imu_stamp)
  {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    cap_stamp_ = cap_stamp;
    imu_stamp_ = imu_stamp;
  }

protected:
  // 时间状态管理
  std::chrono::steady_clock::time_point
      cap_stamp_; ///< 视觉原始捕捉数据的实际时间
  std::chrono::steady_clock::time_point
      imu_stamp_; ///< 预测器建立或最后一次参数更新时的最新IMU时间(并非当前时间)

  // 读写锁，保证多线程安全
  mutable std::shared_mutex mtx_;
};

} // namespace interfaces
} // namespace ne_vision