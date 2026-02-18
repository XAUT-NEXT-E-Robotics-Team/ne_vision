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

#include "opencv2/opencv.hpp"

#include "ne_vision/utils/ne_debug.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include <Eigen/src/SVD/JacobiSVD.h>

#define VIS_QUEUE_MAX_SIZE 100

namespace ne_vision
{

NeVisionVisualization::NeVisionVisualization(
    std::string                name,
    const NeFrameInputCsPtr_t& input_c_sPtr,
    const NeDebugFrameCsPtr_t& debug_frame_c_sPtr)
    : name_(std::move(name))
{
  NV_ASSERT(input_c_sPtr != nullptr && debug_frame_c_sPtr != nullptr &&
            "Input and debug frame channels must not be null.");

  channels_.input_c_sPtr = input_c_sPtr;
  channels_.debug_frame_c_sPtr = debug_frame_c_sPtr;
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

    debug_frame.frame = frame;
    channels_.debug_frame_c_sPtr->Transmit(debug_frame);
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

    vis_queue_.frame_inputs.pop_front();
    if (channels_.armors2d_c_sPtr)
    {
      vis_queue_.armors_2ds.pop_front();
    }
    if (channels_.armors3d_c_sPtr)
    {
      vis_queue_.armors_3ds.pop_front();
    }

    vis_pack_.is_matched = true;

    return true;
  }
  return false;
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

  for (const auto& armor : vis_pack_.armors_3d.armors)
  {
    if (armor.debug.is_main)
    {
      cv::circle(frame,
                 cv::Point(armor.debug.pos_last.x(), armor.debug.pos_last.y()),
                 5,
                 cv::Scalar(255, 0, 255),
                 -1);
      // cv::circle(frame,
      //            cv::Point(armor.debug.pos_last.x(),
      //            armor.debug.pos_last.y()), armor.debug.jump_radius,
      //            cv::Scalar(255, 0, 255),
      //            1);
      // cv::circle(frame,
      //            cv::Point(armor.debug.pos_last.x(),
      //            armor.debug.pos_last.y()), armor.debug.real_distance,
      //            cv::Scalar(255, 255, 0),
      //            1);
      cv::circle(frame,
                 cv::Point(armor.debug.pos_kf_p.x(), armor.debug.pos_kf_p.y()),
                 5,
                 cv::Scalar(0, 255, 255),
                 -1);
      cv::circle(frame,
                 cv::Point(armor.debug.pos_kf_p.x(), armor.debug.pos_last.y()),
                 armor.debug.jump_radius,
                 cv::Scalar(255, 0, 255),
                 1);
      cv::circle(frame,
                 cv::Point(armor.debug.pos_kf_p.x(), armor.debug.pos_last.y()),
                 armor.debug.real_distance,
                 cv::Scalar(255, 255, 0),
                 1);
    }

    cv::putText(frame,
                std::to_string(armor.GetId(vis_pack_.armors_3d.aim_id)),
                cv::Point(armor.debug.pos_current.x(),
                          armor.debug.pos_current.y() - 10),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(255, 255, 0),
                1);
  }
}
} // namespace ne_vision
