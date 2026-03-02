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
// 没啥跟踪的2D跟踪器，干如下几件事情
// 1. 跟踪（反击打灯效，防止一两次误识别更改目标）
// 2. 选板（按照中心选板，还能给后面减少压力）
// 3. PNP
// 4. 优化（上交优化或者自创的一种优化，要计算好协方差）
// 5. 重投影调试（这个可以没有）
//
// Data flow:
// [ne_armors_2d] + [ne_imu_data] >=> tracker_2d >=> [ne_armors_3d]
//

#pragma once

#include <memory>
#include <chrono>
#include <opencv2/core/mat.hpp>
#include <vector>

#include "opencv2/opencv.hpp"
#include "ceres/ceres.h"
#include "sophus/so2.hpp"

#include "ne_vision/interfaces/ne_armors_2d.hpp"
#include "ne_vision/interfaces/ne_imu_data.hpp"
#include "ne_vision/interfaces/ne_armors_3d.hpp"

#include "ne_vision/utils/ne_channel.hpp"
#include "ne_vision/utils/ne_math.hpp"
#include "ne_vision/utils/ne_param.hpp"

#define NE_TRACKER_2D_OUTPOST_NAME "outpost"
namespace ne_vision
{

namespace detail
{

// 用来ceres优化
struct ArmorReprojectionError
{
  // 修改为指针引用外部数据，避免拷贝，支持动态更新
  ArmorReprojectionError(const std::vector<Eigen::Vector2d>* img_points,
                         const std::vector<Eigen::Vector3d>* obj_points,
                         const Eigen::Quaterniond*           q_c_i,
                         const Eigen::Matrix3d*              camera_matrix,
                         const Eigen::Vector3d*              t,
                         const double*                       fixed_pitch_rad)
      : img_points_(img_points), obj_points_(obj_points), q_c_i_(q_c_i),
        camera_matrix_(camera_matrix), t_(t), fixed_pitch_rad_(fixed_pitch_rad)
  {
  }

  template <typename T>
  bool operator()(const T* const yaw_ptr, T* residuals) const
  {
    // 1. 将输入参数转换为Eigen类型
    T yaw = yaw_ptr[0];

    // 2. 将成员变量转换为类型T
    Eigen::Quaternion<T>   q_c_i_T = q_c_i_->cast<T>();
    Eigen::Matrix<T, 3, 3> K = camera_matrix_->cast<T>();
    Eigen::Matrix<T, 3, 1> t = t_->cast<T>();

    // 3. 构造从Armor到IMU的旋转：先绕Z轴旋转yaw，再绕Y轴旋转固定的pitch
    Eigen::Quaternion<T> q_yaw(
        Eigen::AngleAxis<T>(yaw, Eigen::Matrix<T, 3, 1>::UnitZ()));
    Eigen::Quaternion<T> q_pitch(Eigen::AngleAxis<T>(
        T(*fixed_pitch_rad_), Eigen::Matrix<T, 3, 1>::UnitY()));
    Eigen::Quaternion<T> q_i_a = q_yaw * q_pitch;

    // 4. 计算从Armor到Camera的旋转
    Eigen::Quaternion<T> q_c_a = q_c_i_T * q_i_a;

    // 5. 对每个点进行投影并计算残差
    //    注意顺序 (LT, LB, RB, RT)
    for (int i = 0; i < 4; ++i)
    {
      // 点坐标转换为类型T
      Eigen::Matrix<T, 3, 1> p_obj = (*obj_points_)[i].cast<T>();

      // 转换到相机坐标系 P_cam = q_c_a * P_obj + t
      Eigen::Matrix<T, 3, 1> p_cam = q_c_a * p_obj + t;

      // 投影到图像坐标系: P_pix = K * P_cam / Z
      Eigen::Matrix<T, 3, 1> p_pix = K * p_cam / p_cam.z();

      // 计算重投影误差 (理论投影坐标 - 实际观测坐标)
      // 将 x 和 y 误差连续存储在 residuals 数组中
      residuals[2 * i] = p_pix.x() - T((*img_points_)[i].x());
      residuals[2 * i + 1] = p_pix.y() - T((*img_points_)[i].y());
    }

    return true;
  }

private:
  const std::vector<Eigen::Vector2d>* img_points_;
  const std::vector<Eigen::Vector3d>* obj_points_;
  const Eigen::Quaterniond*           q_c_i_;
  const Eigen::Matrix3d*              camera_matrix_;
  const double*                       fixed_pitch_rad_;
  const Eigen::Vector3d*              t_;
};

// 更全面的优化，包括平移，用来提取雅克比
struct OptimizeCost
{
  OptimizeCost(const std::vector<Eigen::Vector2d>* img_points,
               const std::vector<Eigen::Vector3d>* obj_points,
               const Eigen::Quaterniond*           q_g_c,
               const Eigen::Quaterniond*           q_i_g,
               const Eigen::Matrix3d*              camera_matrix,
               const Eigen::Vector3d*              t_g_c,
               const double*                       fixed_pitch_rad)
      : img_points_(img_points), obj_points_(obj_points),
        camera_matrix_(camera_matrix), fixed_pitch_rad_(fixed_pitch_rad),
        q_g_c_(q_g_c), q_i_g_(q_i_g), t_g_c_(t_g_c)
  {
  }

  // 这里的yaw和t都是IMU系下的（注意）
  template <typename T>
  bool
  operator()(const T* const yaw_ptr, const T* const t_ptr, T* residuals) const
  {
    using Vec3_t = Eigen::Matrix<T, 3, 1>;
    using Vec2_t = Eigen::Matrix<T, 2, 1>;
    using Quat_t = Eigen::Quaternion<T>;
    using Mat3_t = Eigen::Matrix<T, 3, 3>;

    // 每次计算重新计算 q_c_i，虽然有一点开销，但避免了维护外部变量的复杂性
    // 或者可以在外部维护 q_c_i 并传入指针
    Quat_t q_c_i_T =
        (q_g_c_->conjugate() * q_i_g_->conjugate()).cast<T>(); // q_c_i

    // 类型转换
    T      yaw = yaw_ptr[0];
    Vec3_t t(t_ptr[0], t_ptr[1], t_ptr[2]);
    Quat_t q_g_c_T = q_g_c_->cast<T>();
    Mat3_t K = camera_matrix_->cast<T>();
    Vec3_t t_g_c_T = t_g_c_->cast<T>();

    // 根据固定角构造从Armor到IMU的旋转
    Quat_t q_yaw(Eigen::AngleAxis<T>(yaw, Vec3_t::UnitZ()));
    Quat_t q_pitch(Eigen::AngleAxis<T>(T(*fixed_pitch_rad_), Vec3_t::UnitY()));
    Quat_t q_i_a = q_yaw * q_pitch;

    // 计算从Armor到Camera的旋转
    Quat_t q_c_a = q_c_i_T * q_i_a;

    // 将t变换到相机系
    Vec3_t t_c = q_c_i_T * t - q_g_c_T.conjugate() * t_g_c_T;

    // 重投影并计算残差
    for (int i = 0; i < 4; ++i)
    {
      Vec3_t p_obj = (*obj_points_)[i].cast<T>();
      Vec3_t p_cam = q_c_a * p_obj + t_c;
      Vec3_t p_pix = K * p_cam / p_cam.z();

      residuals[2 * i] = p_pix.x() - T((*img_points_)[i].x());
      residuals[2 * i + 1] = p_pix.y() - T((*img_points_)[i].y());
    }
    return true;
  }

private:
  const std::vector<Eigen::Vector2d>* img_points_;
  const std::vector<Eigen::Vector3d>* obj_points_;
  const Eigen::Quaterniond*           q_i_g_;
  const Eigen::Quaterniond*           q_g_c_;
  const Eigen::Vector3d*              t_g_c_;
  const Eigen::Matrix3d*              camera_matrix_;
  const double*                       fixed_pitch_rad_;
};

} // namespace detail
class NeTracker2D final
{

private:
  using NeArmors2D_t = interfaces::NeArmors2D_t;
  using NeImuData_t = interfaces::NeImuData_t;
  using NeArmors3D_t = interfaces::NeArmors3D_t;
  using NeArmors2DCsPtr_t = std::shared_ptr<NeChannel<NeArmors2D_t>>;
  using NeImuDataCsPtr_t = std::shared_ptr<NeChannel<NeImuData_t>>;
  using NeArmors3DCsPtr_t = std::shared_ptr<NeChannel<NeArmors3D_t>>;

  struct TrackerAimArmor_t
  {
    NeArmors2D_t::Armor_t armor;
    bool                  pnp_is_valid = false;
    struct
    {
      Eigen::Vector3d    t;
      Eigen::Quaterniond q;
    } camera_to_armor;
    struct
    {
      Eigen::Vector3d    t;
      Eigen::Quaterniond q;
      Sophus::SO2d       yaw;
    } imu_to_armor;

    NeArmors3D_t::Armor3D_t::Debug_t debug_info;
  };

  struct TrackerAim_t
  {
    std::chrono::steady_clock::time_point cap_stamp;

    bool IsLost() const { return lost_count >= lost_count_threshold; }

    bool IsDetected() const { return lost_count == 0; }

    int lost_count = 0;

    // n帧丢失认为目标已经丢失了
    int lost_count_threshold = 10;

    std::string aim_id = "";

    std::vector<TrackerAimArmor_t> aim_armors;
  };

public:
  explicit NeTracker2D(const std::string&       name,
                       const NeArmors2DCsPtr_t& armors_2d_cs_ptr,
                       const NeImuDataCsPtr_t&  imu_data_cs_ptr,
                       const NeArmors3DCsPtr_t& armors_3d_cs_ptr);
  ~NeTracker2D() = default;

  inline std::string GetName() const { return name_; }

  void Tarck2D();

private:
  void trackAndChoose();
  void solvePnP();
  bool matchStamp();
  void transformToImuFrame();
  void yawOptimize();
  void optimize();
  void lmOptimize();
  void reprojectAndFillDebugInfo();

  std::string name_;

  NeArmors2DCsPtr_t armors_2d_c_sPtr_;
  NeImuDataCsPtr_t  imu_data_c_sPtr_;
  NeArmors3DCsPtr_t armors_3d_c_sPtr_;

  NeArmors2D_t armors_2d_;
  NeArmors3D_t armors_3d_;
  NeImuData_t  imu_data_;

  TrackerAim_t current_aim_;

  struct
  {
    std::vector<cv::Point3d>& GetObjectPoints(std::string armor_id)
    {
      if (armor_id == "outpost")
        return object_points_outpost_armor;
      else if (armor_id == "base" || armor_id == "1")
        return object_points_large_armor;
      else
        return object_points_small_armor;
    }

    auto& GetObjectPointsEigen(std::string armor_id)
    {
      if (armor_id == "outpost")
        return object_points_outpost_armor_eigen;
      else if (armor_id == "base" || armor_id == "1")
        return object_points_large_armor_eigen;
      else
        return object_points_small_armor_eigen;
    }

    void ConvertToEigen()
    {
      cv::cv2eigen(camera_matrix, camera_matrix_eigen);

      for (const auto& pt : object_points_small_armor)
        object_points_small_armor_eigen.emplace_back(pt.x, pt.y, pt.z);
      for (const auto& pt : object_points_large_armor)
        object_points_large_armor_eigen.emplace_back(pt.x, pt.y, pt.z);
      for (const auto& pt : object_points_outpost_armor)
        object_points_outpost_armor_eigen.emplace_back(pt.x, pt.y, pt.z);
    }

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    std::vector<cv::Point3d> object_points_small_armor;
    std::vector<cv::Point3d> object_points_large_armor;
    std::vector<cv::Point3d> object_points_outpost_armor;

    Eigen::Matrix3d camera_matrix_eigen;

    std::vector<Eigen::Vector3d> object_points_small_armor_eigen;
    std::vector<Eigen::Vector3d> object_points_large_armor_eigen;
    std::vector<Eigen::Vector3d> object_points_outpost_armor_eigen;
  } pnp_param_;

  struct
  {
    Eigen::Quaterniond q;
    Eigen::Vector3d    t;
  } gimbal_to_camera_;

  struct
  {
    Eigen::Quaterniond q;
    Eigen::Vector3d    t;
  } imu_to_gimbal_;

  struct
  {
    ceres::Solver::Options          options;
    std::unique_ptr<ceres::Problem> problem;
    std::unique_ptr<ceres::Problem> problem_full;

    // 每次优化前只需更新这些变量的值
    std::vector<Eigen::Vector2d> img_points;
    std::vector<Eigen::Vector3d> obj_points;
    Eigen::Quaterniond           q_c_i; // for yawOptimize
    Eigen::Vector3d              t_c_a; // for yawOptimize

    // specifically for optimize()
    Eigen::Quaterniond q_g_c;
    Eigen::Quaterniond q_i_g;
    Eigen::Vector3d    t_g_c;

    double          fixed_pitch_rad;
    double          yaw; // 待优化变量
    Eigen::Vector3d t;   // 待优化变量
  } yaw_optimize_;
};

} // namespace ne_vision
