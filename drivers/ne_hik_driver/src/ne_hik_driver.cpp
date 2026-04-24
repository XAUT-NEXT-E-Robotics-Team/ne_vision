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
// 海康相机驱动实现

#include "ne_hik_driver/ne_hik_driver.hpp"

#include <chrono>
#include <cstring>
#include <thread>

#include <MvCameraControl.h>
#include <opencv2/imgproc.hpp>

#include "ne_vision/utils/ne_log.hpp"

namespace ne_vision
{
namespace drivers
{

/* === MVS SDK 静态回调，SDK自带的，高效捏～ === */

static void
mvCallback(unsigned char* data, MV_FRAME_OUT_INFO_EX* info, void* user)
{
  static_cast<NeHikDriver*>(user)->onFrame(data, info);
}

static void
mvExceptionCallback(unsigned int code, void* user)
{
  if (code == MV_EXCEPTION_DEV_DISCONNECT)
    static_cast<NeHikDriver*>(user)->onDeviceLost();
}

/* === 查找设备 === */

static int findDeviceIndex(const MV_CC_DEVICE_INFO_LIST& list,
                           const std::string&            serial_number)
{
  for (unsigned int i = 0; i < list.nDeviceNum; ++i)
  {
    const auto* info = list.pDeviceInfo[i];
    const char* sn = nullptr;

    if (info->nTLayerType == MV_GIGE_DEVICE)
      sn = reinterpret_cast<const char*>(
          info->SpecialInfo.stGigEInfo.chSerialNumber);
    else if (info->nTLayerType == MV_USB_DEVICE)
      sn = reinterpret_cast<const char*>(
          info->SpecialInfo.stUsb3VInfo.chSerialNumber);

    if (sn && serial_number == sn)
      return static_cast<int>(i);
  }
  return -1;
}

// 相机名字用于打印，并非真实相机名
// 回调懂得都懂
// 参数懂得都懂
NeHikDriver::NeHikDriver(const std::string& camera_name,
                         FrameCallback_t    callback,
                         const Params_t&    params)
    : camera_name_(camera_name), callback_(std::move(callback)), param_(params)
{
}

NeHikDriver::~NeHikDriver() { Stop(); }

// 启动用于看门口的线程
// 这个线程几乎零损耗
void NeHikDriver::Open()
{
  watchdog_thread_ =
      std::jthread([this](std::stop_token st) { watchdogLoop(st); });
}

void NeHikDriver::Stop()
{
  watchdog_thread_.request_stop();
  if (watchdog_thread_.joinable())
    watchdog_thread_.join();
  closeCamera();
}

// 打开相机的内部函数
// 外部通过open调用
bool NeHikDriver::openCamera()
{
  MV_CC_DEVICE_INFO_LIST device_list{};
  int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
  if (ret != MV_OK || device_list.nDeviceNum == 0)
  {
    NV_WARN("[{}] No HIK camera found (ret=0x{:X})", camera_name_, ret);
    return false;
  }

  int idx = 0;
  if (!param_.serial_number.empty())
  {
    idx = findDeviceIndex(device_list, param_.serial_number);
    if (idx < 0)
    {
      NV_WARN("[{}] SN '{}' not found", camera_name_, param_.serial_number);
      return false;
    }
  }

  ret = MV_CC_CreateHandleWithoutLog(&handle_, device_list.pDeviceInfo[idx]);
  if (ret != MV_OK)
  {
    NV_ERROR("[{}] CreateHandle failed (0x{:X})", camera_name_, ret);
    return false;
  }

  ret = MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0);
  if (ret != MV_OK)
  {
    NV_ERROR("[{}] OpenDevice failed (0x{:X})", camera_name_, ret);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }

  MV_CC_SetEnumValue(handle_,
                     "ExposureAuto",
                     param_.auto_exposure ? MV_EXPOSURE_AUTO_MODE_CONTINUOUS
                                          : MV_EXPOSURE_AUTO_MODE_OFF);
  if (!param_.auto_exposure)
    MV_CC_SetFloatValue(handle_, "ExposureTime", param_.exposure_us);

  MV_CC_SetEnumValue(handle_,
                     "GainAuto",
                     param_.auto_gain ? MV_GAIN_MODE_CONTINUOUS
                                      : MV_GAIN_MODE_OFF);
  if (!param_.auto_gain)
    MV_CC_SetFloatValue(handle_, "Gain", param_.gain_db);

  if (param_.width > 0)
    MV_CC_SetIntValueEx(handle_, "Width", param_.width);
  if (param_.height > 0)
    MV_CC_SetIntValueEx(handle_, "Height", param_.height);

  // clang-format off
  static const struct { NeHikPixelFormat_e fmt; MvGvspPixelType mvs; } kFmtMap[] = {
    { NeHikPixelFormat_e::BAYER_RG8, PixelType_Gvsp_BayerRG8  },
    { NeHikPixelFormat_e::BAYER_BG8, PixelType_Gvsp_BayerBG8  },
    { NeHikPixelFormat_e::BAYER_GB8, PixelType_Gvsp_BayerGB8  },
    { NeHikPixelFormat_e::BAYER_GR8, PixelType_Gvsp_BayerGR8  },
    { NeHikPixelFormat_e::RGB8,      PixelType_Gvsp_RGB8_Packed},
    { NeHikPixelFormat_e::BGR8,      PixelType_Gvsp_BGR8_Packed},
  };
  // clang-format on
  if (param_.pixel_format != NeHikPixelFormat_e::AUTO)
  {
    for (const auto& entry : kFmtMap)
    {
      if (entry.fmt == param_.pixel_format)
      {
        MV_CC_SetEnumValue(handle_, "PixelFormat", static_cast<unsigned int>(entry.mvs));
        break;
      }
    }
  }

  MV_CC_RegisterExceptionCallBack(handle_, mvExceptionCallback, this);

  ret = MV_CC_RegisterImageCallBackEx(handle_, mvCallback, this);
  if (ret != MV_OK)
  {
    NV_ERROR("[{}] RegisterCallback failed (0x{:X})", camera_name_, ret);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK)
  {
    NV_ERROR("[{}] StartGrabbing failed (0x{:X})", camera_name_, ret);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }

  last_frame_ns_.store(
      std::chrono::steady_clock::now().time_since_epoch().count(),
      std::memory_order_release);
  is_open_.store(true, std::memory_order_release);
  NV_INFO("[{}] Camera opened (idx={})", camera_name_, idx);
  return true;
}

void NeHikDriver::closeCamera()
{
  if (!handle_)
    return;

  // 设备已物理断开时跳过会阻塞的通信调用，直接销毁 handle
  if (!device_lost_.exchange(false, std::memory_order_acq_rel))
  {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
  }

  MV_CC_DestroyHandle(handle_);
  handle_ = nullptr;
  is_open_.store(false, std::memory_order_release);
}

void NeHikDriver::onDeviceLost()
{
  NV_WARN("[{}] Device lost (SDK exception)", camera_name_);
  device_lost_.store(true, std::memory_order_release);
  is_open_.store(false, std::memory_order_release);
}

// 看门狗的线程
void NeHikDriver::watchdogLoop(std::stop_token st)
{
  auto retry_sleep = [&] {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(param_.retry_interval_ms));
  };

  // 初次打开
  while (!st.stop_requested() && !openCamera())
  {
    NV_WARN(
        "[{}] Retry open in {}ms...", camera_name_, param_.retry_interval_ms);
    retry_sleep();
  }

  while (!st.stop_requested())
  {
    retry_sleep();

    if (is_open_.load(std::memory_order_acquire))
    {
      // 帧超时检测
      int64_t last       = last_frame_ns_.load(std::memory_order_acquire);
      int64_t now        = std::chrono::steady_clock::now().time_since_epoch().count();
      int64_t elapsed_ms = (now - last) / 1'000'000;

      if (elapsed_ms <= param_.watchdog_ms)
        continue;

      NV_WARN("[{}] No frame for {}ms, reconnecting...", camera_name_, elapsed_ms);
      closeCamera(); // 正常关闭（设备可能还活着但无帧）
    }
    else
    {
      // SDK exception callback 已将 is_open_ 置 false，清理 handle
      closeCamera(); // device_lost_=true，跳过阻塞调用，仅销毁 handle
    }

    // 循环尝试重连
    while (!st.stop_requested() && !openCamera())
    {
      NV_WARN("[{}] Retry open in {}ms...", camera_name_, param_.retry_interval_ms);
      retry_sleep();
    }
  }

  closeCamera();
}

void NeHikDriver::onFrame(unsigned char* data, void* raw_info)
{
  auto* info = static_cast<MV_FRAME_OUT_INFO_EX*>(raw_info);

  last_frame_ns_.store(
      std::chrono::steady_clock::now().time_since_epoch().count(),
      std::memory_order_release);

  auto stamp = std::chrono::steady_clock::now();

  int     w = static_cast<int>(info->nWidth);
  int     h = static_cast<int>(info->nHeight);
  cv::Mat bgr;

  cv::Mat raw_bayer(h, w, CV_8UC1, data);
  cv::Mat raw_rgb(h, w, CV_8UC3, data);

  switch (info->enPixelType)
  {
  case PixelType_Gvsp_BayerRG8:
    cv::cvtColor(raw_bayer, bgr, cv::COLOR_BayerRG2BGR);
    break;
  case PixelType_Gvsp_BayerBG8:
    cv::cvtColor(raw_bayer, bgr, cv::COLOR_BayerBG2BGR);
    break;
  case PixelType_Gvsp_BayerGB8:
    cv::cvtColor(raw_bayer, bgr, cv::COLOR_BayerGB2BGR);
    break;
  case PixelType_Gvsp_BayerGR8:
    cv::cvtColor(raw_bayer, bgr, cv::COLOR_BayerGR2BGR);
    break;
  case PixelType_Gvsp_RGB8_Packed:
    cv::cvtColor(raw_rgb, bgr, cv::COLOR_RGB2BGR);
    break;
  case PixelType_Gvsp_BGR8_Packed: bgr = raw_rgb.clone(); break;
  default:
    NV_WARN("[{}] Unsupported pixel format 0x{:X}",
            camera_name_,
            static_cast<uint32_t>(info->enPixelType));
    return;
  }

  callback_(bgr, stamp);
}

} // namespace drivers
} // namespace ne_vision
