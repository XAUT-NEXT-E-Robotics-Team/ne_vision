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

#include "ne_vision/debug/ne_vision_visualization.hpp"

#include "Eigen/Dense"
#include "ne_vision/ne_channals.hpp"
#include "opencv2/opencv.hpp"

#include "ne_vision/utils/ne_debug.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_rerun_debug.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/utils/ne_code_profiler.hpp"

#define VIS_QUEUE_MAX_SIZE 100

namespace ne_vision
{

NeVisionVisualization::NeVisionVisualization(std::string name)
    : name_(std::move(name))
{
  channels_.input_c_sPtr = NV_CHANNELS.frame_input_sPtr();
  channels_.debug_frame_c_sPtr = NV_CHANNELS.debug_frame_sPtr();

  NV_ASSERT(channels_.input_c_sPtr != nullptr &&
            channels_.debug_frame_c_sPtr != nullptr &&
            "Input and debug frame channels must not be null.");

  // 投影工具
  auto_aim_tf_uPtr_ = std::make_unique<NeAutoAimTf>();

  try
  {

    // Armor 3D parameters
    const double small_armor_width =
        NV_PARAM["rm"]["armor"]["small"]["width"].as<double>();
    const double small_armor_height =
        NV_PARAM["rm"]["armor"]["small"]["height"].as<double>();
    armor_param_.small_armor = std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(0, small_armor_width / 2.0, small_armor_height / 2.0),
        Eigen::Vector3d(0, small_armor_width / 2.0, -small_armor_height / 2.0),
        Eigen::Vector3d(0, -small_armor_width / 2.0, -small_armor_height / 2.0),
        Eigen::Vector3d(0, -small_armor_width / 2.0, small_armor_height / 2.0)};

    const double large_armor_width =
        NV_PARAM["rm"]["armor"]["large"]["width"].as<double>();
    const double large_armor_height =
        NV_PARAM["rm"]["armor"]["large"]["height"].as<double>();
    armor_param_.large_armor = std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(0, large_armor_width / 2.0, large_armor_height / 2.0),
        Eigen::Vector3d(0, large_armor_width / 2.0, -large_armor_height / 2.0),
        Eigen::Vector3d(0, -large_armor_width / 2.0, -large_armor_height / 2.0),
        Eigen::Vector3d(0, -large_armor_width / 2.0, large_armor_height / 2.0)};

    const double outpost_armor_width =
        NV_PARAM["rm"]["armor"]["outpost"]["width"].as<double>();
    const double outpost_armor_height =
        NV_PARAM["rm"]["armor"]["outpost"]["height"].as<double>();
    armor_param_.outpost_armor = std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(
            0, outpost_armor_width / 2.0, outpost_armor_height / 2.0),
        Eigen::Vector3d(
            0, outpost_armor_width / 2.0, -outpost_armor_height / 2.0),
        Eigen::Vector3d(
            0, -outpost_armor_width / 2.0, -outpost_armor_height / 2.0),
        Eigen::Vector3d(
            0, -outpost_armor_width / 2.0, outpost_armor_height / 2.0)};
  }
  catch (const std::exception& e)
  {
    NV_ERROR("Failed to load parameters: {}", e.what());
    std::exit(EXIT_FAILURE);
  }
}

void NeVisionVisualization::receiveAll()
{
  NV_ASSERT(channels_.input_c_sPtr &&
            "Input and debug frame channels must not be null.");
  if (channels_.input_c_sPtr->Empty())
  {
    NV_WARN("Waiting for input frames for visualization...");
    return;
  }

  if (channels_.input_c_sPtr->Receive(vis_queue_.frame_inputs.emplace_back()))
  {
    while (vis_queue_.frame_inputs.size() > VIS_QUEUE_MAX_SIZE)
    {
      vis_queue_.frame_inputs.pop_front();
    }
  }

  if (channels_.armors2d_c_sPtr)
  {
    NeArmors2D_t armors_2d;
    if (channels_.armors2d_c_sPtr->Receive(armors_2d, true))
    {
      vis_queue_.armors_2ds.push_back(armors_2d);
      while (vis_queue_.armors_2ds.size() > VIS_QUEUE_MAX_SIZE)
      {
        vis_queue_.armors_2ds.pop_front();
      }
    }
  }

  if (channels_.armors3d_c_sPtr)
  {
    NeArmors3D_t armors_3d;
    if (channels_.armors3d_c_sPtr->Receive(armors_3d))
    {
      vis_queue_.armors_3ds.push_back(armors_3d);
      while (vis_queue_.armors_3ds.size() > VIS_QUEUE_MAX_SIZE)
      {
        vis_queue_.armors_3ds.pop_front();
      }
    }
  }

  if (channels_.aim_state_c_sPtr)
  {
    NeAimState_t aim_state;
    if (channels_.aim_state_c_sPtr->Receive(aim_state))
    {
      vis_queue_.aim_states.push_back(aim_state);
      while (vis_queue_.aim_states.size() > VIS_QUEUE_MAX_SIZE)
      {
        vis_queue_.aim_states.pop_front();
      }
    }
  }

  if (channels_.gimbal_control_ref_c_sPtr)
  {
    NeGimbalControlRef_t gimbal_control_ref;
    if (channels_.gimbal_control_ref_c_sPtr->Receive(gimbal_control_ref))
    {
      vis_queue_.gimbal_control_refs.push_back(gimbal_control_ref);
      while (vis_queue_.gimbal_control_refs.size() > VIS_QUEUE_MAX_SIZE)
      {
        vis_queue_.gimbal_control_refs.pop_front();
      }
    }
  }
}

bool NeVisionVisualization::matchAll()
{
  while (!vis_queue_.frame_inputs.empty())
  {
    vis_pack_.input = vis_queue_.frame_inputs.front();
    auto target_stamp = vis_pack_.input.cap_stamp;

    bool need_wait = false;
    bool frame_dropped_by_algo = false;

    // 1. Armors 2D 配对
    if (channels_.armors2d_c_sPtr)
    {
      while (!vis_queue_.armors_2ds.empty() &&
             vis_queue_.armors_2ds.front().cap_stamp < target_stamp)
      {
        vis_queue_.armors_2ds.pop_front();
      }
      if (vis_queue_.armors_2ds.empty())
      {
        need_wait = true;
      }
      else if (vis_queue_.armors_2ds.front().cap_stamp > target_stamp)
      {
        frame_dropped_by_algo = true;
      }
    }

    // 2. Armors 3D 配对
    if (channels_.armors3d_c_sPtr)
    {
      while (!vis_queue_.armors_3ds.empty() &&
             vis_queue_.armors_3ds.front().cap_stamp < target_stamp)
      {
        vis_queue_.armors_3ds.pop_front();
      }
      if (vis_queue_.armors_3ds.empty())
      {
        need_wait = true;
      }
      else if (vis_queue_.armors_3ds.front().cap_stamp > target_stamp)
      {
        frame_dropped_by_algo = true;
      }
    }

    // 3. Aim State 配对
    if (channels_.aim_state_c_sPtr)
    {
      while (!vis_queue_.aim_states.empty() &&
             vis_queue_.aim_states.front().cap_stamp < target_stamp)
      {
        vis_queue_.aim_states.pop_front();
      }
      if (vis_queue_.aim_states.empty())
      {
        need_wait = true;
      }
      else if (vis_queue_.aim_states.front().cap_stamp > target_stamp)
      {
        frame_dropped_by_algo = true;
      }
    }

    // 4. Gimbal Control Ref 配对 (双时间轴处理核心)
    if (channels_.gimbal_control_ref_c_sPtr)
    {
      // --- 步骤 4.1: 无损搜索 control_stamp ---
      // 在破坏队列之前，先遍历寻找 control_stamp 最接近且 <= 图像 cap_stamp
      // 的数据
      bool found_control = false;
      auto best_control_it = vis_queue_.gimbal_control_refs.begin();

      for (auto it = vis_queue_.gimbal_control_refs.begin();
           it != vis_queue_.gimbal_control_refs.end();
           ++it)
      {
        // 假设你要找的是发生在这个图像时刻之前最新的那个控制指令
        if (it->control_stamp <= target_stamp)
        {
          best_control_it = it;
          found_control = true;
        }
        else
        {
          break; // 因为时间戳是递增的，遇到未来时间直接停止搜索以节省性能
        }
      }

      if (found_control)
      {
        vis_pack_.control_stamp_gimbal_control_ref = *best_control_it;
      }
      else
      {
        vis_pack_.control_stamp_gimbal_control_ref.valid = false;
      }

      // --- 步骤 4.2: 严格对齐 cap_stamp ---
      while (!vis_queue_.gimbal_control_refs.empty() &&
             vis_queue_.gimbal_control_refs.front().cap_stamp < target_stamp)
      {
        vis_queue_.gimbal_control_refs.pop_front();
      }
      if (vis_queue_.gimbal_control_refs.empty())
      {
        need_wait = true;
      }
      else if (vis_queue_.gimbal_control_refs.front().cap_stamp > target_stamp)
      {
        frame_dropped_by_algo = true;
      }
    }

    // 决策逻辑 -----------------------------------------

    // 场景A：算法丢弃了该帧，导致 cap_stamp 断层。丢弃废图，看下一张。
    if (frame_dropped_by_algo)
    {
      vis_queue_.frame_inputs.pop_front();
      continue;
    }

    // 场景B：数据还没算完，卡住画面等回调函数装填数据。
    if (need_wait)
    {
      return false;
    }

    // 场景C：cap_stamp 完美对齐！提取所有主数据。
    if (channels_.armors2d_c_sPtr)
    {
      vis_pack_.armors_2d = vis_queue_.armors_2ds.front();
      vis_queue_.armors_2ds.pop_front();
    }
    if (channels_.armors3d_c_sPtr)
    {
      vis_pack_.armors_3d = vis_queue_.armors_3ds.front();
      vis_queue_.armors_3ds.pop_front();
    }
    if (channels_.aim_state_c_sPtr)
    {
      vis_pack_.aim_state = vis_queue_.aim_states.front();
      vis_queue_.aim_states.pop_front();
    }
    if (channels_.gimbal_control_ref_c_sPtr)
    {
      vis_pack_.gimbal_control_ref = vis_queue_.gimbal_control_refs.front();
      vis_queue_.gimbal_control_refs.pop_front();
      // 注：vis_pack_.control_stamp_gimbal_control_ref 已在步骤 4.1
      // 中被提前装填好了！
    }

    // 所有数据提取完毕，安全丢弃当前主图像
    vis_queue_.frame_inputs.pop_front();
    vis_pack_.is_matched = true;

    return true;
  }

  return false;
}

void NeVisionVisualization::Draw()
{
  receiveAll();
  if (matchAll())
  {
    NeDebugFrame_t debug_frame;
    cv::Mat        frame = vis_pack_.input.frame;

    drawArmors2D(frame);
    drawArmors3D(frame);
    drawTrackerResult(frame);
    drawGimbalControlRef(frame);

    scope_mng_.DrawAll(frame);

    debug_frame.frame = frame;
    channels_.debug_frame_c_sPtr->Transmit(debug_frame);

    // NV_DEBUG("fps: {}",
    //          NV_PROFILE_INSTANCE("detector")->GetResult().GetCurrentPeriodS()
    //          *
    //              1000);

    NV_REC_LOG_FRAME("debug_frame", debug_frame.frame);
  }
}

void NeVisionVisualization::drawArmors2D(cv::Mat& frame)
{
  if (channels_.armors2d_c_sPtr == nullptr || !vis_pack_.is_matched ||
      vis_pack_.armors_2d.armors.empty())
  {
    return;
  }

  for (const auto armor : vis_pack_.armors_2d.armors)
  {
    cv::Point s1(armor.LB.x(), armor.LB.y());
    cv::Point e1(armor.RT.x(), armor.RT.y());
    cv::Point s2(armor.RB.x(), armor.RB.y());
    cv::Point e2(armor.LT.x(), armor.LT.y());

    cv::line(frame, s1, e1, cv::Scalar(0, 255, 0), 1);
    cv::line(frame, s2, e2, cv::Scalar(0, 255, 0), 1);
    cv::putText(frame,
                armor.armor_id,
                cv::Point(armor.RT.x(), armor.RT.y() - 10),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);
    cv::circle(frame, s1, 2, cv::Scalar(255, 0, 0), -1);
    cv::circle(frame, e1, 2, cv::Scalar(0, 0, 255), -1);
    cv::circle(frame, s2, 2, cv::Scalar(255, 0, 0), -1);
    cv::circle(frame, e2, 2, cv::Scalar(0, 0, 255), -1);
  }
}

void NeVisionVisualization::drawArmors3D(cv::Mat& frame)
{
  if (channels_.armors3d_c_sPtr == nullptr || !vis_pack_.is_matched)
  {
    return;
  }

  // for (const auto& each : vis_pack_.armors_3d.armors)
  // {
  //   for (size_t i = 0; i < each.debug.re_projected_pts.size(); i++)
  //   {
  //     cv::line(frame,
  //              each.debug.re_projected_pts[i],
  //              each.debug.re_projected_pts[(i + 1) %
  //                                          each.debug.re_projected_pts.size()],
  //              cv::Scalar(0, 0, 255),
  //              2);
  //   }
  // }
}

void NeVisionVisualization::drawTrackerResult(cv::Mat& frame)
{
  if (channels_.aim_state_c_sPtr == nullptr || !vis_pack_.is_matched)
  {
    return;
  }

  if (channels_.armors3d_c_sPtr == nullptr)
  {
    NV_WARN("Armors3d channel must be added to visualization, cannot draw "
            "tracker result.");
    return;
  }

  const std::string&           armor_id = vis_pack_.aim_state.armor_id;
  std::vector<Eigen::Vector3d> armor_points;
  if (armor_id == "1" || armor_id == "base")
  {
    armor_points = armor_param_.large_armor;
  }
  else if (armor_id == "outpost")
  {
    armor_points = armor_param_.outpost_armor;
  }
  else
  {
    // Default to small armor for 2, 3, 4, 7
    armor_points = armor_param_.small_armor;
  }

  for (const auto& each : vis_pack_.aim_state.debug.all_armors)
  {
    double          yaw = each(3);
    Eigen::Matrix3d R_Z =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    std::vector<cv::Point2d> img_pts;
    img_pts.reserve(4);
    for (const auto& corner : armor_points)
    {
      Eigen::Vector3d world_pt = R_Z * corner + each.head<3>();
      img_pts.push_back(auto_aim_tf_uPtr_->ProjectToImagePlane(
          vis_pack_.armors_3d.imu_data, world_pt));
    }

    for (size_t i = 0; i < 4; ++i)
    {
      size_t j = (i + 1) % 4;
      if (img_pts[i].x >= 0 && img_pts[i].y >= 0 && img_pts[j].x >= 0 &&
          img_pts[j].y >= 0)
      {
        cv::line(frame, img_pts[i], img_pts[j], cv::Scalar(230, 158, 202), 1);
      }
    }
  }
  scope_mng_.AddText("m_dis",
                     cv::format("%.2f", vis_pack_.aim_state.debug.model_dis));
  scope_mng_.AddText("m_omega",
                     cv::format("%.2f", vis_pack_.aim_state.debug.model_omega));
  scope_mng_.AddPoint("m_yaw",
                      vis_pack_.aim_state.cap_stamp,
                      vis_pack_.aim_state.debug.model_yaw);
}

void NeVisionVisualization::drawGimbalControlRef(cv::Mat& frame)
{
  if (channels_.gimbal_control_ref_c_sPtr == nullptr || !vis_pack_.is_matched)
  {
    return;
  }

  if (!vis_pack_.gimbal_control_ref.valid)
  {
    return;
  }

  const std::string&           armor_id = vis_pack_.gimbal_control_ref.armor_id;
  std::vector<Eigen::Vector3d> armor_points;
  if (armor_id == "1" || armor_id == "base")
  {
    armor_points = armor_param_.large_armor;
  }
  else if (armor_id == "outpost")
  {
    armor_points = armor_param_.outpost_armor;
  }
  else
  {
    armor_points = armor_param_.small_armor;
  }

  const Eigen::Vector4d& target_armor_xzyyaw =
      vis_pack_.gimbal_control_ref.debug.target_armor_xyzy;
  const double yaw = target_armor_xzyyaw(3);
  // NV_DEBUG("YAW{}", yaw);
  Eigen::Matrix3d R_Z =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // debug pose uses [x, z, y, yaw], convert to [x, y, z] for rendering.
  Eigen::Vector3d target_center_xyz(
      target_armor_xzyyaw(0), target_armor_xzyyaw(2), target_armor_xzyyaw(1));

  std::vector<cv::Point2d> img_pts;
  img_pts.reserve(4);
  for (const auto& corner : armor_points)
  {
    Eigen::Vector3d world_pt = R_Z * corner + target_center_xyz;
    img_pts.push_back(auto_aim_tf_uPtr_->ProjectToImagePlane(
        vis_pack_.armors_3d.imu_data, world_pt));
  }

  for (size_t i = 0; i < 4; ++i)
  {
    size_t j = (i + 1) % 4;
    if (img_pts[i].x >= 0 && img_pts[i].y >= 0 && img_pts[j].x >= 0 &&
        img_pts[j].y >= 0)
    {
      cv::line(frame, img_pts[i], img_pts[j], cv::Scalar(0, 0, 255), 1);
    }
  }

  cv::Point2d center_img = auto_aim_tf_uPtr_->ProjectToImagePlane(
      vis_pack_.armors_3d.imu_data, target_center_xyz);
  if (center_img.x >= 0 && center_img.y >= 0)
  {
    cv::circle(frame, center_img, 3, cv::Scalar(0, 0, 255), -1);
  }
}

} // namespace ne_vision
