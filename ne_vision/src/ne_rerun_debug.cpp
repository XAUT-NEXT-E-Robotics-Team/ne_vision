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

#include <vector>

#include "ne_vision/debug/ne_rerun_debug.hpp"
#include "ne_vision/debug/ne_debug_def.hpp"
#include "ne_vision/debug/ne_rerun_toolkit.hpp"
#include "ne_vision/interfaces/ne_armors_3d.hpp"
#include "ne_vision/interfaces/ne_frame_input.hpp"
#include "ne_vision/ne_channals.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/utils/ne_tools.hpp"
#include "ne_vision/utils/ne_tools.hpp"

namespace ne_vision
{

NeRerunDebug::NeRerunDebug(const std::string& name) : name_(name)
{
  // 获取装甲板的尺寸信息
  auto armor = NV_PARAM["rm"]["armor"];
  armor_info_.small_w = armor["small"]["width"].as<double>();
  armor_info_.small_h = armor["small"]["height"].as<double>();
  armor_info_.large_w = armor["large"]["width"].as<double>();
  armor_info_.large_h = armor["large"]["height"].as<double>();
  armor_info_.outpost_w = armor["outpost"]["width"].as<double>();
  armor_info_.outpost_h = armor["outpost"]["height"].as<double>();

  // 厚度可以不配置
  armor_info_.thickness = armor["thickness"].as<double>(armor_info_.thickness);
}

void NeRerunDebug::DebugTask()
{
  if (!NV_RERUN_REC.IsEnabled())
    return;

  // 画图 + 内参数发布
  interfaces::NeFrameInput_t msg;
  if (NV_CHANNELS.frame_input_sPtr()->Receive(msg) && !msg.frame.Empty())
  {
    // 等待第一次有效图像接收到后，根据实际情况发布一次相机和图像参数
    if (!run_once_flag_)
    {
      try
      {
        auto camera = NV_PARAM["hardware"]["camera"]["camera_matrix"];
        Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
        K(0, 0) = camera["fx"].as<double>();
        K(1, 1) = camera["fy"].as<double>();
        K(0, 2) = camera["cx"].as<double>();
        K(1, 2) = camera["cy"].as<double>();
        if (K.cast<float>().allFinite() && K(0, 0) > 0.0 && K(1, 1) > 0.0)
        {
          NV_RERUN_REC.LogStaticPinhole(name_ + NVD_CAMERA_PARAM_P,
                                        K,
                                        msg.frame.Size().width,
                                        msg.frame.Size().height);
          run_once_flag_ = true;
        }
        else
        {
          NV_WARN("Invalid camera intrinsics for Rerun: check fx/fy/cx/cy.");
        }
      }
      catch (const YAML::Exception& e)
      {
        NV_WARN("Failed to read camera intrinsics for Rerun: {}", e.what());
      }
    }

    NV_RERUN_REC.LogCvMat(name_ + NVD_CAMERA_FRAME_P, msg.frame, msg.cap_stamp);
  }

  // 画识别器标注
  interfaces::NeArmors2D_t armors_2d;
  if (NV_CHANNELS.armor2d_sPtr()->Receive(armors_2d))
  {
    std::vector<std::vector<Eigen::Vector2d>> groups;
    for (const auto& armor : armors_2d.armors)
    {
      groups.emplace_back(std::vector<Eigen::Vector2d>({armor.LB, armor.RT}));
      groups.emplace_back(std::vector<Eigen::Vector2d>({armor.LT, armor.RB}));
    }

    NV_RERUN_REC.LogLines2D(name_ + NVD_CAMERA_DETECT_RESULT_P,
                            groups,
                            NV_RERUN_COLOR_GREEN,
                            2.0f,
                            armors_2d.cap_stamp);
  }

  // 画2D trakcer结果（3D，IMU系），并写tf
  interfaces::NeArmors3D_t armors_3d;
  if (NV_CHANNELS.armor3d_sPtr()->Receive(armors_3d))
  {
    // 发布imu到相机系的TF
    NV_RERUN_REC.LogTF(name_ + NVD_IMU_P,
                       NVD_GIMBAL_N,
                       armors_3d.imu_data.quat,
                       Eigen::Vector3d(0, 0, 0),
                       armors_3d.cap_stamp);
    NV_RERUN_REC.LogTF(name_ + NVD_GIMBAL_P,
                       NVD_CAMERA_N,
                       armors_3d.gimbal_to_camera.q(),
                       armors_3d.gimbal_to_camera.t(),
                       armors_3d.cap_stamp);

    if (!armors_3d.armors.empty())
    {
      std::vector<NeRerunBox> boxes;
      NeRerunBox              box;
      for (const auto& armor : armors_3d.armors)
      {
        auto type = GetArmorTypeFromId(armors_3d.aim_id);

        double w = armor_info_.small_w;
        double h = armor_info_.small_h;

        if (type == "large")
        {
          w = armor_info_.large_w;
          h = armor_info_.large_h;
        }
        else if (type == "outpost")
        {
          w = armor_info_.outpost_w;
          h = armor_info_.outpost_h;
        }

        box.position = armor.t;
        box.rotation = armor.q;

        box.length = armor_info_.thickness;
        box.width = w;
        box.height = h;

        double pitch = GetPitchFromId(armors_3d.aim_id);

        // box.rotation =
        //     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()) *
        //     box.rotation;

        box.color = NV_RERUN_COLOR_GREEN;

        boxes.push_back(box);
      }

      NV_RERUN_REC.LogBoxes3D(
          name_ + NVD_DETECTED_ARMOR3D_P, boxes, armors_3d.cap_stamp);
    }
  }

  // 画3D tracker结果（3D，IMU系）
  interfaces::NeArmors3D_t armors_3d_tracked;
}

} // namespace ne_vision
