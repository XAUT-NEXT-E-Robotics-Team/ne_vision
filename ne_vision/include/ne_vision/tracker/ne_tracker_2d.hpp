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
// #include "ceres/ceres.h"
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

      // 4x4 x y z yaw
      Eigen::Matrix4d cov;
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
    int lost_count_threshold = 50;

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

  // 识别误差表征图像点识别的不确定性，不是重投影误差
  double detector_variance_;
};

} // namespace ne_vision
