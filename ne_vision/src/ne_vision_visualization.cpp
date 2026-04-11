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
  try
  {
    // Camera intrinsic parameters.
    const double fx =
        NV_PARAM["hardware"]["camera"]["camera_matrix"]["fx"].as<double>();
    const double fy =
        NV_PARAM["hardware"]["camera"]["camera_matrix"]["fy"].as<double>();
    const double cx =
        NV_PARAM["hardware"]["camera"]["camera_matrix"]["cx"].as<double>();
    const double cy =
        NV_PARAM["hardware"]["camera"]["camera_matrix"]["cy"].as<double>();
    pro_param_.camera_matrix_ =
        (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    pro_param_.camera_matrix_eigen_ =
        (Eigen::Matrix3d() << fx, 0, cx, 0, fy, cy, 0, 0, 1).finished();

    const double k1 =
        NV_PARAM["hardware"]["camera"]["dist_coeffs"]["k1"].as<double>();
    const double k2 =
        NV_PARAM["hardware"]["camera"]["dist_coeffs"]["k2"].as<double>();
    const double p1 =
        NV_PARAM["hardware"]["camera"]["dist_coeffs"]["p1"].as<double>();
    const double p2 =
        NV_PARAM["hardware"]["camera"]["dist_coeffs"]["p2"].as<double>();
    const double k3 =
        NV_PARAM["hardware"]["camera"]["dist_coeffs"]["k3"].as<double>();
    pro_param_.dist_coeffs_ = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
    pro_param_.dist_coeffs_eigen_ =
        (Eigen::Matrix<double, 5, 1>() << k1, k2, p1, p2, k3).finished();

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

    if (channels_.armors2d_c_sPtr)
    {
      if (vis_queue_.armors_2ds.empty())
      {
        return false;
      }
      while (!vis_queue_.armors_2ds.empty() &&
             vis_queue_.armors_2ds.front().cap_stamp <
                 vis_pack_.input.cap_stamp)
      {
        vis_queue_.armors_2ds.pop_front();
      }
      if (vis_queue_.armors_2ds.empty())
      {
        return false;
      }
      if (vis_queue_.armors_2ds.front().cap_stamp > vis_pack_.input.cap_stamp)
      {
        vis_queue_.frame_inputs.pop_front();
        continue;
      }
      vis_pack_.armors_2d = vis_queue_.armors_2ds.front();
    }

    if (channels_.armors3d_c_sPtr)
    {
      if (vis_queue_.armors_3ds.empty())
      {
        return false;
      }
      while (!vis_queue_.armors_3ds.empty() &&
             vis_queue_.armors_3ds.front().cap_stamp <
                 vis_pack_.input.cap_stamp)
      {
        vis_queue_.armors_3ds.pop_front();
      }
      if (vis_queue_.armors_3ds.empty())
      {
        return false;
      }
      if (vis_queue_.armors_3ds.front().cap_stamp > vis_pack_.input.cap_stamp)
      {
        vis_queue_.frame_inputs.pop_front();
        continue;
      }
      vis_pack_.armors_3d = vis_queue_.armors_3ds.front();
    }

    if (channels_.aim_state_c_sPtr)
    {
      if (vis_queue_.aim_states.empty())
      {
        return false;
      }
      while (!vis_queue_.aim_states.empty() &&
             vis_queue_.aim_states.front().cap_stamp <
                 vis_pack_.input.cap_stamp)
      {
        vis_queue_.aim_states.pop_front();
      }
      if (vis_queue_.aim_states.empty())
      {
        return false;
      }
      if (vis_queue_.aim_states.front().cap_stamp > vis_pack_.input.cap_stamp)
      {
        vis_queue_.frame_inputs.pop_front();
        continue;
      }
      vis_pack_.aim_state = vis_queue_.aim_states.front();
    }

    if (channels_.gimbal_control_ref_c_sPtr)
    {
      if (vis_queue_.gimbal_control_refs.empty())
      {
        return false;
      }
      while (!vis_queue_.gimbal_control_refs.empty() &&
             vis_queue_.gimbal_control_refs.front().cap_stamp <
                 vis_pack_.input.cap_stamp)
      {
        vis_queue_.gimbal_control_refs.pop_front();
      }
      if (vis_queue_.gimbal_control_refs.empty())
      {
        return false;
      }
      if (vis_queue_.gimbal_control_refs.front().cap_stamp >
          vis_pack_.input.cap_stamp)
      {
        vis_queue_.frame_inputs.pop_front();
        continue;
      }
      vis_pack_.gimbal_control_ref = vis_queue_.gimbal_control_refs.front();
    }

    vis_queue_.frame_inputs.pop_front();
    if (channels_.armors2d_c_sPtr)
    {
      vis_queue_.armors_2ds.pop_front();
    }
    if (channels_.armors3d_c_sPtr)
    {
      vis_queue_.armors_3ds.pop_front();
    }
    if (channels_.aim_state_c_sPtr)
    {
      vis_queue_.aim_states.pop_front();
    }
    if (channels_.gimbal_control_ref_c_sPtr)
    {
      vis_queue_.gimbal_control_refs.pop_front();
    }
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

cv::Point2d
NeVisionVisualization::projectToImagePlane(const Eigen::Vector3d& point_3d)
{
  cv::Point2d point_2d = {-1, -1};
  if (channels_.armors2d_c_sPtr == nullptr)
  {
    NV_WARN("Armors2D channel is not set, cannot project to image plane.");
    return point_2d;
  }

  if (vis_pack_.armors_3d.armors.empty())
  {
    // 这里应该是不能正常重投影的，但是有可能被多次调用，免得日志太多
    return point_2d;
  }
  auto q_c_i = vis_pack_.armors_3d.armors.at(0).debug.camera_to_imu.q;
  auto t_c_i = vis_pack_.armors_3d.armors.at(0).debug.camera_to_imu.t;

  // 计算到相机的旋转和平移
  auto& q_c_p = q_c_i;
  auto  t_c_p = q_c_i * point_3d + t_c_i;

  // 进行投影
  auto P = (pro_param_.camera_matrix_eigen_ * t_c_p) / t_c_p.z();

  cv::Point2d projected_point(P.x(), P.y());

  return projected_point;
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
      img_pts.push_back(projectToImagePlane(world_pt));
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
    img_pts.push_back(projectToImagePlane(world_pt));
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

  cv::Point2d center_img = projectToImagePlane(target_center_xyz);
  if (center_img.x >= 0 && center_img.y >= 0)
  {
    cv::circle(frame, center_img, 3, cv::Scalar(0, 0, 255), -1);
  }
}

} // namespace ne_vision
