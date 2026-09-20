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
// IMU(extern) >=> [ne_imu_data] >=> tracker_2d
//

#pragma once

#include "Eigen/Dense"
#include "Eigen/src/Geometry/Quaternion.h"
#include <chrono>

namespace ne_vision
{
namespace interfaces
{

struct NeImuData_t
{

  // 同步器使用差值函数处理IMU数据
  // 加速度、角速度线性插值，姿态使用归一化四元数球面插值。
  // 越界时返回对应端点，不外推；时间区间无效时返回 a1。
  static NeImuData_t
  Interpolate(const NeImuData_t&                           a1,
              const NeImuData_t&                           a2,
              const std::chrono::steady_clock::time_point& t1,
              const std::chrono::steady_clock::time_point& t2,
              const std::chrono::steady_clock::time_point& target_time)
  {
    if (t2 <= t1)
      return a1;
    if (target_time <= t1)
    {
      auto result = a1;
      result.receive_stamp = t1;
      return result;
    }
    if (target_time >= t2)
    {
      auto result = a2;
      result.receive_stamp = t2;
      return result;
    }

    const double alpha =
        std::chrono::duration<double>(target_time - t1).count() /
        std::chrono::duration<double>(t2 - t1).count();

    NeImuData_t result;
    result.receive_stamp = target_time;
    result.acc = (1.0 - alpha) * a1.acc + alpha * a2.acc;
    result.gyro = (1.0 - alpha) * a1.gyro + alpha * a2.gyro;
    result.quat =
        a1.quat.normalized().slerp(alpha, a2.quat.normalized()).normalized();
    return result;
  }

  // Timestamp of receiving the IMU data
  std::chrono::steady_clock::time_point receive_stamp;

  Eigen::Vector3d    acc;  // Acceleration in m/s^2
  Eigen::Vector3d    gyro; // Angular velocity in rad/s. Not use now
  Eigen::Quaterniond quat; // Orientation as a quaternion
};

} // namespace interfaces
} // namespace ne_vision
