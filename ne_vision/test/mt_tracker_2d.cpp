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
// 只测试tracker_2d最好不要耦合其他模块

#include <cmath>
#include <exception>
#include <memory>

#include "mgl2/mgl.h"

#include "ne_vision/tracker/ne_tracker_2d.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/utils/ne_channel.hpp"
#include "ne_vision/utils/ne_math.hpp"
#include "ne_vision/utils/ne_path.hpp"

#include "ne_vision/interfaces/ne_armors_2d.hpp"
#include "ne_vision/interfaces/ne_imu_data.hpp"
#include "ne_vision/interfaces/ne_armors_3d.hpp"

using namespace ne_vision;

using NeArmors2D_t = interfaces::NeArmors2D_t;
using NeImuData_t = interfaces::NeImuData_t;
using NeArmors3D_t = interfaces::NeArmors3D_t;
using NeArmors2DCsPtr_t = std::shared_ptr<NeChannel<NeArmors2D_t>>;
using NeImuDataCsPtr_t = std::shared_ptr<NeChannel<NeImuData_t>>;
using NeArmors3DCsPtr_t = std::shared_ptr<NeChannel<NeArmors3D_t>>;

#define YAW_WITH_IMU_TEST_MAX_DEG  720
#define YAW_WITH_IMU_TEST_STEP_DEG 10

cv::Mat MglToMat(mglGraph& gr)
{
  int                  width = gr.GetWidth();
  int                  height = gr.GetHeight();
  const unsigned char* data = gr.GetRGB();
  cv::Mat              img;
  cv::cvtColor(
      cv::Mat(height, width, CV_8UC3, const_cast<unsigned char*>(data)),
      img,
      cv::COLOR_RGB2BGR);
  return img;
}

int main()
{
  NeArmors2DCsPtr_t armors_2d_c_sPtr =
      std::make_shared<NeChannel<NeArmors2D_t>>(
          "armor_2d", NeChannelType_e::KEEP_ON_READ, 1);
  NeArmors3DCsPtr_t armors_3d_c_sPtr =
      std::make_shared<NeChannel<NeArmors3D_t>>(
          "armors_3d", NeChannelType_e::KEEP_ON_READ, 1);
  NeImuDataCsPtr_t imu_data_c_sPtr = std::make_shared<NeChannel<NeImuData_t>>(
      "imu_data", NeChannelType_e::KEEP_ON_READ, 100);

  // 必须先处理参数
  std::string install_path = GetInstallPath();
  NV_INFO("Install path: {}", install_path);

  std::string config_path =
      install_path + "/share/test/config/mt_tracker_2d.yaml";

  NV_PARAM.Load(config_path);

  // 这里没有参数没办法初始化
  NeTracker2D tracker_2d(
      "tracker_2d", armors_2d_c_sPtr, imu_data_c_sPtr, armors_3d_c_sPtr);

  NeArmors2D_t armors_2d;
  NeImuData_t  imu_data;

  armors_2d.armors.emplace_back("4",
                                'R',
                                521.0671234130859,
                                455.2542886734009,
                                520.2849655151367,
                                471.274827003479,
                                558.9615612030029,
                                456.11876678466797,
                                558.2226104736328,
                                471.8865895271301);
  armors_2d.frame_height = 810;
  armors_2d.frame_width = 1080;

  std::vector<double> rads_un_wrap;
  std::vector<double> yaws;
  std::vector<double> distances;

  for (double deg = 0; deg <= YAW_WITH_IMU_TEST_MAX_DEG;
       deg += YAW_WITH_IMU_TEST_STEP_DEG)
  {
    double rad_un_wrap = math::DegToRad(deg);
    double rad = math::WrapToPi(rad_un_wrap);

    imu_data.receive_stamp = std::chrono::steady_clock::now();
    armors_2d.cap_stamp = imu_data.receive_stamp;

    imu_data.quat = math::EulerToQuaternion(0, 0, rad);

    auto yaw_test = math::QuaternionToYaw(imu_data.quat);

    armors_2d_c_sPtr->Transmit(armors_2d);
    imu_data_c_sPtr->Transmit(imu_data);

    tracker_2d.Tarck2D();

    NeArmors3D_t armors_3d;

    if (!armors_3d_c_sPtr->Receive(armors_3d))
    {
      NV_ERROR("Failed to receive armors_3d data");
      return 1;
    }
    if (armors_3d.armors.empty())
    {
      NV_WARN("No armor in armors_3d received");
    }
    rads_un_wrap.push_back(rad_un_wrap);
    yaws.push_back(
        armors_3d.armors.empty() ? 0 : math::WrapToPi(armors_3d.armors[0].yaw));
    distances.push_back(
        armors_3d.armors.empty() ? 0 : armors_3d.armors[0].t.norm());
  }

  // 画图
  mglData data_imu_yaw(rads_un_wrap.size());
  mglData data_armor_yaw(yaws.size());
  for (size_t i = 0; i < std::min(rads_un_wrap.size(), yaws.size()); ++i)
  {
    data_imu_yaw.a[i] = rads_un_wrap[i];
    data_armor_yaw.a[i] = yaws[i];
  }
  mglGraph gr;
  gr.SetSize(800, 600);
  // gr.Title("Yaw with IMU Testa imu_yaw -> armor_yaw");
  gr.SetRanges(0,
               math::DegToRad(YAW_WITH_IMU_TEST_MAX_DEG),
               -math::DegToRad(180),
               math::DegToRad(180));
  gr.Axis();
  gr.Grid();
  gr.Plot(data_imu_yaw, data_armor_yaw, "r*");
  auto img = MglToMat(gr);
  cv::imshow("Yaw with IMU Test", img);

  gr.SetSize(800, 600);
  gr.SetRanges(0,
               math::DegToRad(YAW_WITH_IMU_TEST_MAX_DEG),
               0,
               *std::max_element(distances.begin(), distances.end()));
  gr.Axis();
  gr.Grid();
  mglData data_distance(distances.size());
  for (size_t i = 0; i < distances.size(); ++i)
  {
    data_distance.a[i] = distances[i];
  }
  gr.Plot(data_imu_yaw, data_distance, "b*");
  auto img_distance = MglToMat(gr);
  cv::imshow("Distance with IMU Test", img_distance);
re:
  if (cv::waitKey(0) == 27)
  {
    return 0;
  }
  else
  {
    goto re;
  }
}
