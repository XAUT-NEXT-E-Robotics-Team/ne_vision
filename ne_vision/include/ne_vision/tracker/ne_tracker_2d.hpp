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
// This node(part) do these things:
// 1. Choose the aim car according to the distance of center and the priority.
// 2. Track 2D armors and put true ID for them.
// 3. Use pnp to solve the 3D position of each armor in CAMERA FRAME.
// 4. Match the stamp of each armor data from cameras with the nearest IMU data.
// 5. Transform the 3D position of each armor from CAMERA FRAME to IMU FRAME.
//
// Data flow:
// [ne_armors_2d] + [ne_imu_data] >=> tracker_2d >=> [ne_armors_3d]
//

#pragma once

#include <memory>
#include <list>
#include <chrono>
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
  ArmorReprojectionError(const std::vector<Eigen::Vector2d>& img_points,
                         const std::vector<Eigen::Vector3d>& obj_points,
                         const Eigen::Quaterniond&           q_gimbal_to_camera,
                         const Eigen::Quaterniond&           q_imu,
                         const Eigen::Matrix3d&              camera_matrix,
                         const Eigen::Vector3d&              t,
                         double                              fixed_pitch_rad)
      : img_points_(img_points), obj_points_(obj_points),
        camera_matrix_(camera_matrix), t_(t),
        fixed_pitch_rad_(fixed_pitch_rad) // 初始化成员变量
  {
    // 计算从IMU到Camera的旋转
    q_c_i_ = q_gimbal_to_camera.conjugate() * q_imu.conjugate();
  }

  template <typename T>
  bool operator()(const T* const yaw_ptr, T* residuals) const
  {
    // 1. 将输入参数转换为Eigen类型
    T yaw = yaw_ptr[0];

    // 2. 将成员变量转换为类型T
    Eigen::Quaternion<T>   q_c_i_T = q_c_i_.cast<T>();
    Eigen::Matrix<T, 3, 3> K = camera_matrix_.cast<T>();
    Eigen::Matrix<T, 3, 1> t = t_.cast<T>();

    // 3. 构造从Armor到IMU的旋转：先绕Z轴旋转yaw，再绕Y轴旋转固定的pitch
    Eigen::Quaternion<T> q_yaw(
        Eigen::AngleAxis<T>(yaw, Eigen::Matrix<T, 3, 1>::UnitZ()));
    Eigen::Quaternion<T> q_pitch(Eigen::AngleAxis<T>(
        T(fixed_pitch_rad_), Eigen::Matrix<T, 3, 1>::UnitY()));
    Eigen::Quaternion<T> q_i_a = q_yaw * q_pitch;

    // 4. 计算从Armor到Camera的旋转
    Eigen::Quaternion<T> q_c_a = q_c_i_T * q_i_a;

    // 5. 对每个点进行投影并计算残差
    //    注意顺序 (LT, LB, RB, RT)
    for (int i = 0; i < 4; ++i)
    {
      // 点坐标转换为类型T
      Eigen::Matrix<T, 3, 1> p_obj = obj_points_[i].cast<T>();

      // 转换到相机坐标系 P_cam = q_c_a * P_obj + t
      Eigen::Matrix<T, 3, 1> p_cam = q_c_a * p_obj + t;

      // 投影到图像坐标系: P_pix = K * P_cam / Z
      Eigen::Matrix<T, 3, 1> p_pix = K * p_cam / p_cam.z();

      // 计算重投影误差 (理论投影坐标 - 实际观测坐标)
      // 将 x 和 y 误差连续存储在 residuals 数组中
      residuals[2 * i] = p_pix.x() - T(img_points_[i].x());
      residuals[2 * i + 1] = p_pix.y() - T(img_points_[i].y());
    }

    return true;
  }

private:
  std::vector<Eigen::Vector2d> img_points_;
  std::vector<Eigen::Vector3d> obj_points_;
  Eigen::Quaterniond           q_c_i_;
  Eigen::Matrix3d              camera_matrix_;
  double                       fixed_pitch_rad_;
  Eigen::Vector3d t_; // 新增：用于存储不参与优化的平移向量
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

  std::string name_;

  NeArmors2DCsPtr_t armors_2d_cs_ptr_;
  NeImuDataCsPtr_t  imu_data_cs_ptr_;
  NeArmors3DCsPtr_t armors_3d_cs_ptr_;

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
    ceres::Solver::Options options;
  } yaw_optimize_;
};

} // namespace ne_vision
