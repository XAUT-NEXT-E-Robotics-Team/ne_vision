/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-03-22 20:31:20
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-03-22 22:05:57
 * @FilePath: /ne_vision/ne_vision/interfaces/real/src/main.cpp
 * @Description: 我永远喜欢雪之下雪乃
 *
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved.
 */
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
#include "real/ne_hikcam.hpp"
#include <opencv2/highgui.hpp>
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
  HikCam    hik_cam;
  auto_aim.Start(config_path);
  hik_cam.StartDevice(0);
  hik_cam.SetResolution(1440, 1080);
  // hik_cam.SetPixelFormat(17301512);
  hik_cam.SetExposureTime(8000);
  hik_cam.SetGAIN(16.0);
  hik_cam.SetFrameRate(120);
  hik_cam.SetStreamOn();
  hik_cam.GetMat(frame);
  while (1)
  {

    hik_cam.GetMat(frame);
    auto_aim.UpdateFrame(frame, 'B'); // 接收颜色
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
    auto key = cv::waitKey(1);
    if (key == 27)
    {
      break;
    }
  }

  return 0;
}
