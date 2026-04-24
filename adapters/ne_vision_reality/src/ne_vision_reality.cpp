///////////////////////////////////////////////////////////
//                                                       //
//                        .                .:-:          //
//                        :-:              :-::          //
//                      -----          .:---.            //
//                    .-------.     .:-----:             //
//                   :---------. .:-------.              //
//                  :--------------------.               //
//                 ---------------------                 //
//                .-------:. :---------:                 //
//               :-----:.     .-------.                  //
//              .:---:         .-----.                   //
//            .:-:.              :-:                     //
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
// 串口回环性能测试：接收cmd=0x01，立即回环发送cmd=0x02

#include <atomic>
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>

#include "ne_serial_driver/ne_serial_driver.hpp"
#include "ne_hik_driver/ne_hik_driver.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/ne_auto_aim.hpp"

#pragma pack(push, 1)
struct GimbalInputProtocol_t
{
  // 下面分别是：四元数，角速度，加速度
  // 坐标系为 x 向前，y 向左，z 向上（右手系），与 IMU 坐标系一致
  float q[4];
  float gyro[3];
  float acc[3];

  // 我们的颜色和弹速
  char  out_color;
  float bullet_speed;
};
struct GimbalControlProtocol_t
{
  // 期望轨迹，注意：相对IMU系，未考虑ROLL
  float yaw;
  float pitch;
  float yaw_v;
  float pitch_v;

  // 命中概率，用户火控
  // 沿当前轨迹运动时子弹命中的概率
  // 0 表示没目标
  // 未实装
  uint8_t hit_probability;
};
#pragma pack(pop)

std::string GetExecutablePath()
{
  return std::filesystem::canonical("/proc/self/exe").parent_path().string();
}

int main()
{
  using namespace ne_vision;
  using namespace ne_vision::drivers;

  // 找配置文件
  auto exe_path = GetExecutablePath();
  auto config_path =
      exe_path + "/../share/ne_vision_reality/config/config.yaml";

  // 初始化自瞄
  NeAutoAim auto_aim;

  // 初始化串口
  NeSerialDriver serial_driver("/tmp/ttyV1", BaudRate_e::B_115200);

  // 初始化HIK
  NeHikDriverParams_t hik_params;
  hik_params.pixel_format = NeHikPixelFormat_e::BGR8;
  hik_params.width = 1440;
  hik_params.height = 1080;
  hik_params.exposure_us = 8000.0f;
  hik_params.gain_db = 16.0f;

  // 相机和相机回调
  NeHikDriver hik_driver(
      "camera",
      [&](cv::Mat& frame, auto stamp) {
        auto_aim.UpdateFrame(frame, "hik_camera");
      },
      hik_params);

  // 串口回调：仅接收数据
  auto input_protocol_h =
      serial_driver.NewProtocolHandler<GimbalInputProtocol_t>(0x01);
  input_protocol_h->AddCallBack([&](const auto& msg) {
    auto_aim.UpdateImu({msg.acc[0], msg.acc[1], msg.acc[2]},
                       {msg.gyro[0], msg.gyro[1], msg.gyro[2]},
                       {msg.q[0], msg.q[1], msg.q[2], msg.q[3]});
    auto_aim.UpdateRobotInfo(msg.out_color, msg.bullet_speed);
  });

  // gimbal控制输出回调
  auto_aim.SetGimbalCallback([&](const auto& ref) {
    GimbalControlProtocol_t output_msg;
    output_msg.yaw = ref.control_ref.yaw;
    output_msg.pitch = ref.control_ref.pitch;
    output_msg.yaw_v = ref.control_ref.yaw_v;
    output_msg.pitch_v = ref.control_ref.pitch_v;
    // hit_probability未实装，暂置0
    output_msg.hit_probability = 0;

    serial_driver.TransmitProtocol(0x02, output_msg);
  });

  // 调试回调
  auto_aim.SetDebugCallback([&]() {
    cv::Mat debug_frame;
    auto_aim.GetDebugFrame(debug_frame);
    if (!debug_frame.empty())
    {
      cv::imshow("Debug Frame", debug_frame);
      cv::waitKey(1);
    }
  });

  // 启动
  hik_driver.Open();
  serial_driver.Open();
  auto_aim.Start(config_path);

  auto_aim.Spin();

  // 停止
  auto_aim.Stop();
  serial_driver.Stop();
  hik_driver.Stop();

  return 0;
}