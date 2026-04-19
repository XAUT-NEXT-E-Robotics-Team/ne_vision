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
#include <chrono>
#include <mutex>
#include <thread>
using namespace ne_vision;

std::atomic_bool is_running = true;

// 图像线程只负责维护图像
void FrameThread(NeAutoAim& auto_aim, HikCam& hik_cam)
{
  cv::Mat frame;
  for (; is_running.load();)
  {
    hik_cam.GetMat(frame);
    if (!frame.empty())
    {
      // 图像更新
      auto_aim.UpdateFrame(frame);

      // 获取debug
      auto_aim.DebugFrame(frame);

      // 显示
      if (!frame.empty())
      {
        cv::imshow("debug", frame);
      }
    }
    auto key = cv::waitKey(1);
    if (key == 27)
    {
      is_running.store(false);
      break;
    }
  }
}

// 串口线程负责维护串口和控制
void SerialThread(int freq, NeAutoAim& auto_aim)
{
  // ne_serial::NeSerialDriver driver("/dev/ttyACM0", 200000, false);

  std::mutex             state_mtx;
  ne_serial::GimbalState latest_state;

  // driver.onGimbalState([&](const ne_serial::GimbalState& gs)
  // {
  //   std::lock_guard lk(state_mtx);
  //   latest_state = gs;
  // });

  // if (!driver.start())
  // {
  //   NV_ERROR("Serial driver failed to start");
  //   return;
  // }

  // // 读取最新串口状态并注入自瞄模块
  // auto readFromSerial = [&]() {
  //   ne_serial::GimbalState gs;
  //   {
  //     std::lock_guard lk(state_mtx);
  //     gs = latest_state;
  //   }

  //   // 0 = 红方, 1 = 蓝方（依协议约定）
  //   char color = (gs.our_color == 1) ? 'B' : 'R';
  //   auto_aim.UpdateRobotInfo(color, static_cast<double>(gs.muzzle_v));

  //   Eigen::Quaterniond quat = Eigen::AngleAxisd(static_cast<double>(gs.yaw),
  //                                               Eigen::Vector3d::UnitZ()) *
  //                             Eigen::AngleAxisd(static_cast<double>(gs.pitch),
  //                                               Eigen::Vector3d::UnitY());
  //   auto_aim.UpdateImu(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
  //   quat);
  // };

  NeAutoAimResult_t result;
  const auto        period = std::chrono::microseconds(1'000'000 / freq);
  for (; is_running.load();)
  {
    const auto t0 = std::chrono::steady_clock::now();

    // readFromSerial();
    auto_aim.UpdateRobotInfo('B', 20);
    auto_aim.UpdateImu(Eigen::Vector3d::Zero(),
                       Eigen::Vector3d::Zero(),
                       Eigen::Quaterniond::Identity());
    auto_aim.AutoAim();
    auto_aim.GetResult(result);

    // ne_serial::GimbalOrientation orient;
    // orient.yaw = static_cast<float>(result.control_ref.yaw);
    // orient.pitch = static_cast<float>(result.control_ref.pitch);
    // orient.fire = (result.state == NeAutoAimState_e::AIMING)
    //                   ? sentry_protocol::AUTO_AIM_FIRE
    //                   : sentry_protocol::AUTO_AIM_NO_FIRE;
    // orient.state = 0;
    // SET_AUTO_AIM_STATE_ENABLE(orient.state);
    // if (result.state == NeAutoAimState_e::AIMING)
    //   SET_AUTO_AIM_STATE_TRACKING(orient.state);
    // driver.sendGimbalOrientation(orient);

    std::this_thread::sleep_until(t0 + period);
  }

  // driver.stop();
}

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

  hik_cam.StartDevice(0);
  hik_cam.SetResolution(1440, 1080);
  hik_cam.SetPixelFormat(17301512);
  hik_cam.SetExposureTime(8000);
  hik_cam.SetGAIN(16.0);
  hik_cam.SetFrameRate(120);
  hik_cam.SetStreamOn();
  hik_cam.GetMat(frame);

  // 检测读取状态
  if (frame.empty())
  {
    NV_ERROR("Failed to get frame from camera, exiting...");
    return -1;
  }

  // 启动自瞄
  auto_aim.Start(config_path);

  // 开启两个线程：一个维护图像，一个维护串口和控制
  // 图像为自由频率
  // 串口为固定频率

  std::jthread frame_thread(FrameThread, std::ref(auto_aim), std::ref(hik_cam));
  std::jthread serial_thread(SerialThread, 100, std::ref(auto_aim));

  frame_thread.join();
  serial_thread.join();

  return 0;
}
