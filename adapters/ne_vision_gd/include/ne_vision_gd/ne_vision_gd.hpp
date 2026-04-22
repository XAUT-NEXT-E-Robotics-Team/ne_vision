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

#pragma once

#include <memory>

#include "godot_cpp/classes/ref.hpp"
#include "godot_cpp/classes/ref_counted.hpp"
#include "godot_cpp/classes/image.hpp"

#include "godot_cpp/core/math_defs.hpp"
#include "godot_cpp/variant/char_string.hpp"
#include "godot_cpp/variant/dictionary.hpp"
#include "godot_cpp/variant/quaternion.hpp"
#include "godot_cpp/variant/string.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include "ne_vision/ne_auto_aim.hpp"

#include "ne_vision/utils/ne_param.hpp"

namespace ne_vision
{

namespace gdextension
{

using namespace godot;

class NeVisionGd : public godot::RefCounted
{
  GDCLASS(NeVisionGd, godot::RefCounted)

public:
  NeVisionGd();
  ~NeVisionGd();

  void Start(const godot::String& config_path);
  void UpdataFrame(const godot::Ref<godot::Image>& gd_img);
  void GetViualizeFrame(godot::Ref<godot::Image> gd_img);
  void UpdateImu(const godot::Vector3&    acc,
                 const godot::Vector3&    gyro,
                 const godot::Quaternion& quat,
                 godot::real_t            delay_s);
  void UpdateRobotInfo(const godot::String& our_color,
                       const godot::real_t  bullet_velocity);

  godot::Dictionary GetResult() const;

  void          set_config_file_path(const godot::String& path);
  godot::String get_config_file_path() const;

protected:
  static void _bind_methods();

private:
  std::unique_ptr<NeAutoAim> auto_aim_uPtr_;
  std::string                config_file_path_;
};

} // namespace gdextension

}; // namespace ne_vision
