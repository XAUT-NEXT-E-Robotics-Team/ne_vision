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

#include "ne_vision_gd/ne_vision_gd.hpp"

#include "godot_cpp/variant/packed_byte_array.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "godot_cpp/core/class_db.hpp"

#include "godot_cpp/variant/dictionary.hpp"
#include "godot_cpp/variant/variant.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include <opencv2/highgui.hpp>

namespace ne_vision
{

// Redefine the weak logging functions.
// Godot's has its own logging system.
void NvWeakLogTrace(const std::string& msg)
{
  godot::UtilityFunctions::print(msg.c_str());
}

void NvWeakLogDebug(const std::string& msg)
{
  godot::UtilityFunctions::print(msg.c_str());
}

void NvWeakLogInfo(const std::string& msg)
{
  godot::UtilityFunctions::print(msg.c_str());
}

void NvWeakLogWarn(const std::string& msg)
{
  godot::UtilityFunctions::push_warning(msg.c_str());
}

void NvWeakLogError(const std::string& msg)
{
  godot::UtilityFunctions::push_error(msg.c_str());
}

namespace gdextension
{

void NeVisionGd::_bind_methods()
{
  godot::ClassDB::bind_method(godot::D_METHOD("updata_frame", "frame"),
                              &NeVisionGd::UpdataFrame);

  godot::ClassDB::bind_method(godot::D_METHOD("start", "config_path"),
                              &NeVisionGd::Start);

  godot::ClassDB::bind_method(godot::D_METHOD("get_visualize_frame", "frame"),
                              &NeVisionGd::GetViualizeFrame);

  godot::ClassDB::bind_method(
      godot::D_METHOD("update_imu", "acc", "gyro", "quat", "delay_s"),
      &NeVisionGd::UpdateImu);

  godot::ClassDB::bind_method(
      godot::D_METHOD("update_robot_info", "our_color", "bullet_velocity"),
      &NeVisionGd::UpdateRobotInfo);

  godot::ClassDB::bind_method(godot::D_METHOD("get_result"),
                              &NeVisionGd::GetResult);
}

NeVisionGd::NeVisionGd() { NV_INFO("NeVisionGd constructor called."); }

NeVisionGd::~NeVisionGd() { NV_INFO("NeVisionGd destructor called."); }

void NeVisionGd::Start(const godot::String& config_path)
{
  if (!auto_aim_uPtr_)
  {
    NV_WARN("auto_aim_uPtr_ is null. Creating new instance.");
    try
    {
      auto_aim_uPtr_ = std::make_unique<NeAutoAim>();
    }
    catch (const std::exception& e)
    {
      NV_ERROR("Failed to create NeAutoAim: {}", e.what());
      godot::UtilityFunctions::push_error(
          ("NeAutoAim init failed: " + std::string(e.what())).c_str());
      return;
    }
  }
  auto_aim_uPtr_->Start(config_path.utf8().get_data());
}

void NeVisionGd::UpdataFrame(const godot::Ref<godot::Image>& gd_img)
{
  if (!auto_aim_uPtr_)
  {
    static int log_counter = 0;
    if (log_counter++ % 60 == 0)
    {
      NV_WARN("NeAutoAim is not initialized.");
    }
    return;
  }

  if (gd_img.is_null())
  {
    NV_WARN("Received a null image from GD.");
    return;
  }

  if (gd_img->get_format() != godot::Image::Format::FORMAT_RGBA8)
  {
    gd_img->convert(godot::Image::Format::FORMAT_RGBA8);
  }

  godot::PackedByteArray img_data = gd_img->get_data();
  uint8_t*               data_ptr = reinterpret_cast<uint8_t*>(img_data.ptrw());

  int width = gd_img->get_width();
  int height = gd_img->get_height();

  cv::Mat cv_img(height, width, CV_8UC4, data_ptr);
  cv::cvtColor(cv_img, cv_img, cv::COLOR_RGBA2BGR);

  auto_aim_uPtr_->UpdateFrame(cv_img);
}

void NeVisionGd::UpdateImu(const godot::Vector3&    acc,
                           const godot::Vector3&    gyro,
                           const godot::Quaternion& quat,
                           godot::real_t            delay_s)
{
  if (!auto_aim_uPtr_)
  {
    NV_WARN("NeAutoAim is not initialized.");
    return;
  }

  auto_aim_uPtr_->UpdateImu(Eigen::Vector3d(acc.x, acc.y, acc.z),
                            Eigen::Vector3d(gyro.x, gyro.y, gyro.z),
                            Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z));
}

// I try to put gd_img in it, but it could not work anymore.
void NeVisionGd::GetViualizeFrame(godot::Ref<godot::Image> gd_img)
{
  if (!auto_aim_uPtr_)
  {
    NV_WARN("NeAutoAim is not initialized.");
    return;
  }

  cv::Mat vis_frame;
  auto_aim_uPtr_->DebugFrame(vis_frame);
  if (vis_frame.empty())
  {
    return;
  }

  cv::imshow("Debug Frame", vis_frame);
  if (cv::waitKey(1) == 27) // Press 'Esc' key to close the window
  {
  }
}

void NeVisionGd::UpdateRobotInfo(const godot::String& our_color,
                                 const godot::real_t  bullet_velocity)
{
  if (!auto_aim_uPtr_)
  {
    NV_WARN("NeAutoAim is not initialized.");
    return;
  }

  char              color_char = '0';
  godot::CharString utf8_str = our_color.utf8();
  if (utf8_str.length() > 0)
  {
    color_char = utf8_str.get_data()[0];
  }

  auto_aim_uPtr_->UpdateRobotInfo(color_char, bullet_velocity);
}

godot::Dictionary NeVisionGd::GetResult() const
{
  godot::Dictionary dict;
  if (!auto_aim_uPtr_)
  {
    dict["state"] = 0; // STOP
    return dict;
  }

  auto_aim_uPtr_->AutoAim();
  NeAutoAimResult_t result;
  auto_aim_uPtr_->GetResult(result);

  dict["state"] = static_cast<int>(result.state);
  dict["yaw"] = result.control_ref.yaw;
  dict["pitch"] = result.control_ref.pitch;
  dict["yaw_v"] = result.control_ref.yaw_v;
  dict["pitch_v"] = result.control_ref.pitch_v;
  return dict;
}

} // namespace gdextension

} // namespace ne_vision
