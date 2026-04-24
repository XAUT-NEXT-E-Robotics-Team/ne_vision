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
// 海康相机驱动（MVS SDK）
//
// 采帧方式：MVS SDK 内置回调线程（MV_CC_RegisterImageCallBackEx），
//           驱动本身不创建采帧线程。
// 断线重连：watchdog std::jthread 监控最后收帧时间，超时则重连。
//
// 使用方式:
//   NeHikDriver driver("cam_left",
//     [](cv::Mat& frame, std::chrono::steady_clock::time_point stamp) {
//       // 处理帧
//     }, params);
//   driver.Open();
//   // ...
//   driver.Stop();
//

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>

namespace ne_vision
{
namespace drivers
{

enum class NeHikPixelFormat_e
{
  AUTO, // 不修改相机当前格式，由 onFrame 按实际格式转换
  BAYER_RG8,
  BAYER_BG8,
  BAYER_GB8,
  BAYER_GR8,
  RGB8,
  BGR8,
};

// 移到类外，避免 clang 对嵌套 struct 默认成员初始化器 + 类内默认实参的已知限制
struct NeHikDriverParams_t
{
  std::string serial_number = "";    // 空 = 使用第一个找到的设备
  float       exposure_us = 5000.0f; // 曝光时间，单位微秒
  float       gain_db = 0.0f;        // 增益 dB
  bool        auto_exposure = false;
  bool        auto_gain = false;
  int         watchdog_ms = 1000;       // 超过此时间无帧则认为断线
  int         retry_interval_ms = 1000; // 断线重连间隔

  // 分辨率：0 = 不修改，保持相机默认值
  int width = 0;
  int height = 0;

  NeHikPixelFormat_e pixel_format = NeHikPixelFormat_e::AUTO;
};

class NeHikDriver
{
public:
  using FrameCallback_t =
      std::function<void(cv::Mat&                              frame,
                         std::chrono::steady_clock::time_point stamp)>;

  using Params_t = NeHikDriverParams_t;

  /**
   * @brief 构造 NeHikDriver
   * @param camera_name 相机名称，用于日志
   * @param callback    收帧回调，在 MVS SDK 内置线程中调用
   * @param params      相机参数
   */
  explicit NeHikDriver(const std::string& camera_name,
                       FrameCallback_t    callback,
                       const Params_t&    params = {});

  ~NeHikDriver();

  /**
   * @brief 打开相机并启动 watchdog，支持断线重连
   */
  void Open();

  /**
   * @brief 停止 watchdog 并关闭相机
   */
  void Stop();

  inline bool IsOpen() const
  {
    return is_open_.load(std::memory_order_acquire);
  }
  inline std::string GetCameraName() const { return camera_name_; }
  inline std::string GetName() const { return "ne_hik_driver"; }

  // 由 cpp 内 MVS SDK 静态回调调用，不应由外部代码直接调用
  // info 实为 MV_FRAME_OUT_INFO_EX*，用 void* 避免头文件依赖 MVS SDK
  void onFrame(unsigned char* data, void* info);

  // MVS SDK 异常回调（设备断开事件），不应由外部代码直接调用
  void onDeviceLost();

private:
  bool openCamera();
  void closeCamera();
  void watchdogLoop(std::stop_token st);

  std::string     camera_name_;
  FrameCallback_t callback_;
  Params_t        param_;

  void*                handle_ = nullptr;
  std::atomic<bool>    is_open_{false};
  std::atomic<bool>    device_lost_{false}; // SDK 断开事件已触发，跳过阻塞式关闭调用
  std::atomic<int64_t> last_frame_ns_{0};
  std::jthread         watchdog_thread_;
};

} // namespace drivers
} // namespace ne_vision
