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
// 2D-tracker:
// 英语会不了一点
// 这里是一种Lazy的跟踪方法。
// 我们先假设，当两块装甲板存在时，不可能出现不能配对的情况。因为当我们考虑上个帧和当前帧的关系，
// 在两块装甲板同时出现情况下，如果上一帧不指向这两块装甲板的任何一块，就意味着，只能指向相差90度
// 的两个（上一帧可视），这在图像上是不可能出现的。
// 所以再存在两个装甲板时，我们将当前追踪指向距离最近的哪个识别到的装甲板就可以了。
// 而当一块装甲板跳变（没有出现两装甲板过渡现象）时，我们需要根据阈值来进行判断。

#pragma once

#include <memory>
#include <list>
#include <chrono>

#include "ne_vision/interfaces/ne_armors_2d.hpp"
#include "ne_vision/interfaces/ne_imu_data.hpp"
#include "ne_vision/interfaces/ne_armors_3d.hpp"

#include "ne_vision/utils/ne_channel.hpp"
#include "ne_vision/utils/ne_math.hpp"
#include "ne_vision/utils/ne_param.hpp"

#include "ne_vision/tracker/kf/ne_tracker_2d_kf.hpp"

#define NE_TRACKER_2D_OUTPOST_NAME "outpost"
namespace ne_vision
{

class NeTracker2D final
{
private:
  // May be some param I am lazy to write in yaml.

  // If the armor is lost some times consecutively, we will consider it as lost
  // and delete it from tracker buffer.
  constexpr static int kLostCountThres_ = 10;

  // If the distance between the center is larger than the ratio * the armor's
  // length. We will consider it as not matched.
  constexpr static double kMatchDistanceRatio_ = 1.0;

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
    NePeriodicNumber<4>   id;
    NePeriodicNumber<3>   id_outpost;

    NeArmors3D_t::Armor3D_t::Debug_t debug;

    void IdAdd(int num)
    {
      id_outpost += num;
      id += num;
    }

    void IdInit()
    {
      id_outpost = 0;
      id = 0;
    }
  };

  struct TrackerAim_t
  {
    TrackerAim_t(const NeArmors2D_t::Armor_t& armor_visible)
    {
      const double sigma_q_x =
          NV_PARAM["auto_aim"]["tracker_2d"]["sigma_q_x"].as<double>();
      const double sigma_q_y =
          NV_PARAM["auto_aim"]["tracker_2d"]["sigma_q_y"].as<double>();
      const double sigma_r_x =
          NV_PARAM["auto_aim"]["tracker_2d"]["sigma_r_x"].as<double>();
      const double sigma_r_y =
          NV_PARAM["auto_aim"]["tracker_2d"]["sigma_r_y"].as<double>();

      kf_uPtr = std::make_unique<kf::NeTracker2DKf>(
          sigma_q_x, sigma_q_y, sigma_r_x, sigma_r_y);
      kf_uPtr->Init(armor_visible.center);

      armor_main.armor = armor_visible;

      armor_main.IdInit();
      armor_other.IdInit();
    }

    std::unique_ptr<kf::NeTracker2DKf> kf_uPtr;
    int                                lost_count = 0;
    bool                               is_init = false;

    // If it is true, the other armor is valid.
    bool other_armor_is_valid = false;

    TrackerAimArmor_t armor_main;
    TrackerAimArmor_t armor_other;
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
  void chooseAim();
  void track2D();
  void solvePnP();
  void matchStamp();
  void transformToImuFrame();

  int matchSort(TrackerAim_t& aim, const NeArmors2D_t::Armor_t& armor_detected);

  std::string name_;

  NeArmors2DCsPtr_t armors_2d_cs_ptr_;
  NeImuDataCsPtr_t  imu_data_cs_ptr_;
  NeArmors3DCsPtr_t armors_3d_cs_ptr_;

  NeArmors2D_t armors_2d_;
  NeArmors3D_t armors_3d_;

  std::list<TrackerAim_t> tracker_aim_list_;

  double                                dt_;
  std::chrono::steady_clock::time_point last_time_point_;
};

} // namespace ne_vision
