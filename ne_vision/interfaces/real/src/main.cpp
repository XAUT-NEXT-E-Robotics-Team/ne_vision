/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-03-22 20:31:20
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-03-23 01:09:38
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
#include "ne_vision/serial/ne_serial_driver.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
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
  // ne_serial::NeSerialDriver driver("/dev/ttyACM0",200000,false);
  std::mutex serial_mtx;
  float      gimbal_pitch = 0.0f;
  float      gimbal_yaw = 0.0f;
  float      muzzle_v = 25.0f;
  char       our_color = '\0';
  // driver.onGimbalState([&](const ne_serial::GimbalState& gimbal_state)
  // {
  //    std::lock_guard<std::mutex> lk(serial_mtx);
  //    gimbal_pitch = gimbal_state.pitch;
  //    gimbal_yaw = gimbal_state.yaw;
  //    muzzle_v = gimbal_state.muzzle_v;
  //    our_color = gimbal_state.our_color;
  // });
  // if(!driver.start())
  // {
  //   printf("Failed to open serial driver");
  //   return -1;
  // }

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
    char  tmp_color;
    float tmp_muzzle_v;
    {
      std::lock_guard<std::mutex> lk(serial_mtx);
      tmp_color = our_color;
      tmp_muzzle_v = muzzle_v;
    }

    auto_aim.UpdateFrame(frame, 'B'); // 接收颜色
    auto_aim.AutoAim();
    auto_aim.UpdateImu(Eigen::Vector3d::Zero(),
                       Eigen::Vector3d::Zero(),
                       Eigen::Quaterniond::Identity(),
                       0.0);

    double aim_yaw = 0;
    double aim_pitch = 0;

    auto_aim.GetResult(aim_yaw, aim_pitch);
    ne_serial::GimbalOrientation ne_orien;
    ne_orien.state = 0;
    SET_AUTO_AIM_STATE_ENABLE(ne_orien.state);
    if (tmp_color == 'R')
      SET_AUTO_AIM_STATE_WE_ARE_RED(ne_orien.state);
    ne_orien.pitch = static_cast<float>(aim_pitch);
    ne_orien.yaw = static_cast<float>(aim_yaw);
    ne_orien.fire = 0xff; // TODO: 根据自瞄置信度决定是否开火
    // driver.sendGimbalOrientation(ne_orien);

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
  // driver.stop();
  return 0;
}
