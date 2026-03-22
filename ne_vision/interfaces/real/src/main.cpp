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

#include "ne_vision/ne_auto_aim.hpp"

using namespace ne_vision;

int main()
{
  // 获取config地址
  const char* install_path_name = "NE_VISION_INSTALL_PATH";
  const char* install_path = std::getenv(install_path_name);
  NV_ASSERT(install_path != nullptr &&
            "You need to set the NE_VISION_INSTALL_PATH environment variable.");

  std::string install_path_str = std::string(install_path);
  std::string config_path = install_path_str + "/share/config/config.yaml";

  cv::Mat   frame;
  NeAutoAim auto_aim;

  auto_aim.Start(config_path);

  while (1)
  {
    auto_aim.UpdateFrame(frame, 'B');
    auto_aim.UpdateTestImu(-0.0001);
    auto_aim.AutoAim();

    double aim_yaw = 0;
    double aim_pitch = 0;

    auto_aim.GetResult(aim_yaw, aim_pitch);

    cv::Mat re;
    auto_aim.DebugFrame(re);
    if (!re.empty())
    {
      cv::imshow("debug", re);
    }
  }

  return 0;
}
