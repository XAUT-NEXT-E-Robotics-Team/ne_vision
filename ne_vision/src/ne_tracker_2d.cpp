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
//

#include <algorithm>
#include <vector>

#include "ne_vision/tracker/ne_tracker_2d.hpp"

#include "ne_vision/utils/ne_log.hpp"

#define BEBUG_LOG

namespace ne_vision
{

NeTracker2D::NeTracker2D(const std::string&       name,
                         const NeArmors2DCsPtr_t& armors_2d_cs_ptr,
                         const NeImuDataCsPtr_t&  imu_data_cs_ptr,
                         const NeArmors3DCsPtr_t& armors_3d_cs_ptr)
    : name_(name), armors_2d_cs_ptr_(armors_2d_cs_ptr),
      imu_data_cs_ptr_(imu_data_cs_ptr), armors_3d_cs_ptr_(armors_3d_cs_ptr)
{
  last_time_point_ = std::chrono::steady_clock::now();
}

void NeTracker2D::Tarck2D()
{
  armors_3d_.armors.clear();

  if (armors_2d_cs_ptr_ && armors_2d_cs_ptr_->Receive(armors_2d_))
  {
    armors_3d_.cap_stamp = armors_2d_.cap_stamp;

    // Cau dt
    auto now = std::chrono::steady_clock::now();
    dt_ = std::chrono::duration<double>(now - last_time_point_).count();
    last_time_point_ = now;

    track2D();
    // for (auto& each_aim : tracker_aim_list_)
    // {
    //   NV_DEBUG(
    //       "Aim ID: {}, Lost Count: {}, Main Armor Center: ({:.2f}, {:.2f}), "
    //       "Other Armor{:.2f}, {:.2f}), Other Armor Valid: {}",
    //       each_aim.armor_main.id,
    //       each_aim.lost_count,
    //       each_aim.armor_main.armor.center.x(),
    //       each_aim.armor_main.armor.center.y(),
    //       each_aim.armor_other.armor.center.x(),
    //       each_aim.armor_other.armor.center.y(),
    //       each_aim.other_armor_is_valid);
    // }
    chooseAim();

    matchStamp();
    solvePnP();
    transformToImuFrame();

    for (auto& each_aim : tracker_aim_list_)
    {
      armors_3d_.aim_id = each_aim.armor_main.id;

      NeArmors3D_t::Armor3D_t armor_3d;

      armor_3d.SetId(armors_3d_.aim_id, each_aim.armor_main.id);
      armor_3d.debug = each_aim.armor_main.debug;

      armors_3d_.armors.push_back(armor_3d);

      // If the other armor is valid, also push it to armors_3d_.
      if (each_aim.other_armor_is_valid)
      {
        armor_3d.SetId(armors_3d_.aim_id, each_aim.armor_other.id);
        armor_3d.debug = each_aim.armor_other.debug;
        armors_3d_.armors.push_back(armor_3d);
      }
    }

    armors_3d_cs_ptr_->Transmit(armors_3d_);
  }
}

void NeTracker2D::track2D()
{
  for (auto it = tracker_aim_list_.begin(); it != tracker_aim_list_.end();)
  {
    if (it->lost_count > kLostCountThres_)
    {
      // The aim is lost, remove it from the list.
      it = tracker_aim_list_.erase(it);
      continue;
    }

    if (armors_2d_.armors.empty())
    {
      // No armor is detected, just increase the lost count.
      it->lost_count++;
      it++;
      continue;
    }

    // Find all armors in same armor id
    std::vector<NeArmors2D_t::Armor_t> same_id_armors;

    auto tmp_it = std::stable_partition(
        armors_2d_.armors.begin(),
        armors_2d_.armors.end(),
        [&](const NeArmors2D_t::Armor_t& armor_detected) {
          return it->armor_main.armor.armor_id == armor_detected.armor_id;
        });
    std::move(
        armors_2d_.armors.begin(), tmp_it, std::back_inserter(same_id_armors));
    armors_2d_.armors.erase(armors_2d_.armors.begin(),
                            tmp_it); // Remove the same id armors

    if (same_id_armors.empty() || same_id_armors.size() > 2)
    {
      // No armor with same id is detected, just increase the lost count.
      // If more than 2 armors with same id are detected, it is also considered
      // as lost.
      it->lost_count++;
      it++;
      continue;
    }

    it->lost_count = 0; // Matched, reset lost count

    it->kf_uPtr->Predict(dt_);

    if (same_id_armors.size() == 2)
    {
      // Two armors with same id are detected, just match and update the one
      // with smaller distance to the predicted position.

      std::sort(same_id_armors.begin(),
                same_id_armors.end(),
                [&](const NeArmors2D_t::Armor_t& armor1,
                    const NeArmors2D_t::Armor_t& armor2) {
                  // return (it->armor_main.armor.center - armor1.center).norm()
                  // <
                  //        (it->armor_main.armor.center -
                  //        armor2.center).norm();
                  return (it->kf_uPtr->GetPrePos() - armor1.center).norm() <
                         (it->kf_uPtr->GetPrePos() - armor2.center).norm();
                });

      it->armor_main.armor = same_id_armors[0];

      it->armor_other.armor = same_id_armors[1];
      it->other_armor_is_valid = true;

      // handle ID
      it->armor_other.id = it->armor_main.id;
      it->armor_other.id_outpost = it->armor_main.id_outpost;
      if (it->armor_other.armor.center.x() < it->armor_main.armor.center.x())
        it->armor_other.IdAdd(1); // Left
      else
        it->armor_other.IdAdd(-1); // Right

      // START DEBUG
      it->armor_main.debug.jump_radius = 4;
      it->armor_main.debug.real_distance = 3;
      it->armor_main.debug.pos_kf_p = it->kf_uPtr->GetPrePos();
      it->armor_main.debug.pos_last = it->armor_main.armor.center;
      it->armor_main.debug.pos_current = it->armor_main.armor.center;
      it->armor_other.debug.pos_current = it->armor_other.armor.center;
      // END DEBUG

      it->kf_uPtr->Update(it->armor_main.armor.center);
    }
    else
    {
      // Only one armor with same id is detected, just match and update it.
      const auto id_single = matchSort(*it, same_id_armors[0]);
      it->armor_main.armor = same_id_armors[0];
      it->other_armor_is_valid = false;
      it->armor_main.IdAdd(id_single);
      it->lost_count = 0;

      if (id_single != 0)
      {
        // If jump, reset and init kf (beacuse the aim is changed)
        it->kf_uPtr->Reset();
        it->kf_uPtr->Init(it->armor_main.armor.center);
      }
      else
      {
        it->kf_uPtr->Update(it->armor_main.armor.center);
      }
    }
  }

  // If there are some armors are detected but not matched with any aim, create
  // new aims for them.

  std::unordered_map<std::string, std::vector<NeArmors2D_t::Armor_t>>
      grouped_armors;
  for (const auto& armor_detected : armors_2d_.armors)
  {
    grouped_armors[armor_detected.armor_id].push_back(armor_detected);
  }

  for (const auto& group : grouped_armors)
  {
    const auto& same_id_armors = group.second;
    if (same_id_armors.size() > 2)
      continue; // If more than 2 armors with same id are detected, it is
                // considered as error.

    TrackerAim_t new_aim(same_id_armors[0]);

    // START DEBUG
    new_aim.armor_main.debug.is_main = true;
    new_aim.armor_other.debug.is_main = false;
    // END DEBUG

    if (same_id_armors.size() == 2)
    {
      new_aim.armor_other.armor = same_id_armors[1];
      new_aim.other_armor_is_valid = true;

      // handle ID (default ID is set)
      if (new_aim.armor_other.armor.center.x() <
          new_aim.armor_main.armor.center.x())
        new_aim.armor_other.IdAdd(1); // Left
      else
        new_aim.armor_other.IdAdd(-1); // Right
    }
    tracker_aim_list_.emplace_back(std::move(new_aim));
  }
}

void NeTracker2D::chooseAim() {}

void NeTracker2D::solvePnP() {}

void NeTracker2D::matchStamp() {}

void NeTracker2D::transformToImuFrame() {}

int NeTracker2D::matchSort(TrackerAim_t&                aim,
                           const NeArmors2D_t::Armor_t& armor_detected)
{
  // Calculate the length of the detected armor and the visible armor.
  // The calculate the avg of them.
  // 英语编不下去了 ABAB
  // 无论对于可视装甲板（跟踪buffer中）还是对于识别装甲板，计算装甲板长方法一致且如下：
  // 1. 先计算一边的直线方程
  // 2. 取另一边的俩角点作点到直线距离并求平均。
  // 然后两种装甲板再求平均作为评判标准。

#define Y1 aim.armor_main.armor.LB.y()
#define Y2 aim.armor_main.armor.LT.y()
#define X1 aim.armor_main.armor.LB.x()
#define X2 aim.armor_main.armor.LT.x()
  const double A1 = Y1 - Y2;
  const double B1 = X2 - X1;
  const double C1 = X1 * Y2 - X2 * Y1;
#undef Y1
#undef Y2
#undef X1
#undef X2
#define Y1 armor_detected.LB.y()
#define Y2 armor_detected.LT.y()
#define X1 armor_detected.LB.x()
#define X2 armor_detected.LT.x()
  const double A2 = Y1 - Y2;
  const double B2 = X2 - X1;
  const double C2 = X1 * Y2 - X2 * Y1;

#undef Y1
#undef Y2
#undef X1
#undef X2

#define X0 aim.armor_main.armor.RB.x()
#define Y0 aim.armor_main.armor.RB.y()
#define X1 aim.armor_main.armor.RT.x()
#define Y1 aim.armor_main.armor.RT.y()
  const double l_1_0 =
      std::abs(A1 * X0 + B1 * Y0 + C1) / std::sqrt(A1 * A1 + B1 * B1);
  const double l_1_1 =
      std::abs(A1 * X1 + B1 * Y1 + C1) / std::sqrt(A1 * A1 + B1 * B1);
  const double avg_l_1 = (l_1_0 + l_1_1) / 2.0;
#undef Y0
#undef Y1
#undef X0
#undef X1
#define X0 armor_detected.RB.x()
#define Y0 armor_detected.RB.y()
#define X1 armor_detected.RT.x()
#define Y1 armor_detected.RT.y()
  const double l_2_0 =
      std::abs(A2 * X0 + B2 * Y0 + C2) / std::sqrt(A2 * A2 + B2 * B2);
  const double l_2_1 =
      std::abs(A2 * X1 + B2 * Y1 + C2) / std::sqrt(A2 * A2 + B2 * B2);
  const double avg_l_2 = (l_2_0 + l_2_1) / 2.0;
#undef Y0
#undef Y1
#undef X0
#undef X1

  const double l = (avg_l_1 + avg_l_2) / 2.0;
  const double d = (aim.kf_uPtr->GetPrePos() - armor_detected.center).norm();
  // const double d = (aim.armor_main.armor.center -
  // armor_detected.center).norm();

  const double jump_radius = kMatchDistanceRatio_ * l;

  // START DEBUG
  aim.armor_main.debug.jump_radius = jump_radius;
  aim.armor_main.debug.real_distance = d;
  aim.armor_main.debug.pos_kf_p = aim.kf_uPtr->GetPrePos();
  aim.armor_main.debug.pos_last = aim.armor_main.armor.center;
  aim.armor_main.debug.pos_current = armor_detected.center;
  // END DEBUG

  //   NV_DEBUG("l{} d{}", l, d);
  if (d < jump_radius)
    return 0; // Matched
  else
  {
    // if (aim.armor_main.armor.center.x() < armor_detected.center.x())
    if (aim.kf_uPtr->GetPrePos().x() < armor_detected.center.x())
      return -1; // Jumped right
    else
      return 1; // Jumped left
  }
}
} // namespace ne_vision
