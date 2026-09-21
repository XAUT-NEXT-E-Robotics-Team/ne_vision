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
// 工具包本体

#pragma once

#include <chrono>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include "rerun.hpp"
#include "rerun/recording_stream.hpp"
#include "spdlog/spdlog.h"

#define NV_RERUN_REC ne_vision::NeRerunToolkit::Instance()

#define NV_RERUN_COLOR(r, g, b, a) rerun::Color(r, g, b, a)
#define NV_RERUN_COLOR_GREEN       NV_RERUN_COLOR(0, 255, 0, 255)
#define NV_RERUN_COLOR_RED         NV_RERUN_COLOR(255, 0, 0, 255)
#define NV_RERUN_COLOR_BLUE        NV_RERUN_COLOR(0, 0, 255, 255)
#define NV_RERUN_COLOR_YELLOW      NV_RERUN_COLOR(255, 255, 0, 255)
#define NV_RERUN_COLOR_CYAN        NV_RERUN_COLOR(0, 255, 255, 255)
#define NV_RERUN_COLOR_MAGENTA     NV_RERUN_COLOR(255, 0, 255, 255)
#define NV_RERUN_COLOR_WHITE       NV_RERUN_COLOR(255, 255, 255, 255)
#define NV_RERUN_COLOR_PURPLE      NV_RERUN_COLOR(128, 0, 128, 255)

namespace ne_vision
{

enum class RecType_e
{
  ONLINE,
  FILE
};

// 几何体结构体区
struct NeRerunBox
{
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d    position = Eigen::Vector3d::Zero(); // box 中心
  double             length = 1.0;                       // 长（x）
  double             width = 1.0;                        // 宽（y）
  double             height = 1.0;                       // 高（z）
  rerun::Color       color = NV_RERUN_COLOR_GREEN;       // RGBA，建议使用宏定义
};

class NeRerunToolkit final
{
public:
  static NeRerunToolkit& Instance();

  // 如果是online，path就是ip 如果是file，path就是文件路径，最后一个是app的名称
  bool Enable(const RecType_e&   rec_type = RecType_e::ONLINE,
              const std::string& path = "",
              const std::string& name = "ne_vision");

  bool inline IsEnabled() const { return is_enabled_ && rec_uptr_ != nullptr; }

  inline bool Disable()
  {
    if (!is_enabled_)
      return true;

    if (rec_uptr_ != nullptr)
      rec_uptr_.reset();

    is_enabled_ = false;
    return true;
  }

  // stamp 默认使用当前时间，也可传入数据采集时的 steady_clock 时间点。
  inline void LogF64(const std::string&                    path,
                     const double&                         val,
                     std::chrono::steady_clock::time_point stamp =
                         std::chrono::steady_clock::now())
  {
    if (!IsEnabled())
      return;

    rec_uptr_->set_time_duration(timeline_name_, stamp.time_since_epoch());
    rec_uptr_->log(path, rerun::Scalars(val));
  }

  // 支持 CV_8UC1（灰度）、CV_8UC3（BGR）、CV_8UC4（BGRA），含非连续 ROI。
  void LogCvMat(const std::string&                    path,
                const cv::Mat&                        mat,
                std::chrono::steady_clock::time_point stamp =
                    std::chrono::steady_clock::now());

  // 画线
  // * path需要为图像路径的子路径，stamp需要与图像一致：如 camera/frame/lines
  // * groups的结构为[线1、[点1、点2、...]、...]，多个点依次为一条折线
  // * color采用RGB或RGBA，默认绿色，建议使用宏NV_RERUN_COLOR(r, g, b, a)
  // * 线宽单位是像素
  void LogLines2D(const std::string&                               path,
                  const std::vector<std::vector<Eigen::Vector2d>>& groups,
                  rerun::Color color = rerun::Color(0, 255, 0),
                  float        line_width = 3.0f,
                  std::chrono::steady_clock::time_point stamp =
                      std::chrono::steady_clock::now());

  // 画3D盒子
  // * path 不用说
  // * boxes 采用结构体vector，去填结构体
  // * stamp 不用说
  void LogBoxes3D(const std::string&                    path,
                  const std::vector<NeRerunBox>&        boxes,
                  std::chrono::steady_clock::time_point stamp =
                      std::chrono::steady_clock::now());

  // 图像去畸变发送
  // RERUN的view不支持畸变参数，所以需要在发送前去畸变
  void LogCvMatUndistorted(const std::string&                    path,
                           const cv::Mat&                        mat,
                           const Eigen::Matrix3d&                K,
                           const std::vector<double>&            distortion,
                           std::chrono::steady_clock::time_point stamp =
                               std::chrono::steady_clock::now());

  // 静态发布相机信息
  // path应该与图像相同
  // K是相机内参
  // width和height是图像的宽高
  void LogStaticPinhole(const std::string&     path,
                        const Eigen::Matrix3d& K,
                        int                    width,
                        int                    height);

  // 发布动态TF信息，该消息标明parent -> child的TF
  // RERUN中path决定tftree的关系，变换描述的是父节点到子节点直接的变换如world/robot
  // world/robot/imu imu这个子节点的变换是相对于robot系的而不是world系
  //
  // parent 父节点全称 eg：world/robot
  // child 子节点名，不含父节点路径 eg：img
  void LogTF(const std::string&                    parent,
             const std::string&                    child,
             const Eigen::Quaterniond&             rotation,
             const Eigen::Vector3d&                translation,
             std::chrono::steady_clock::time_point stamp =
                 std::chrono::steady_clock::now());

  // 静态变换对所有时间生效；同一子路径不要混用静态和动态 TF。
  void LogStaticTF(const std::string&        parent,
                   const std::string&        child,
                   const Eigen::Quaterniond& rotation,
                   const Eigen::Vector3d&    translation);

  NeRerunToolkit(const NeRerunToolkit&) = delete;
  NeRerunToolkit& operator=(const NeRerunToolkit&) = delete;
  NeRerunToolkit(NeRerunToolkit&&) = delete;
  NeRerunToolkit& operator=(NeRerunToolkit&&) = delete;

private:
  NeRerunToolkit();
  ~NeRerunToolkit();

  bool        is_enabled_ = false;
  std::string timeline_name_; // Enable 时生成 name + "_clock"。

  std::unique_ptr<rerun::RecordingStream> rec_uptr_;
};

} // namespace ne_vision
