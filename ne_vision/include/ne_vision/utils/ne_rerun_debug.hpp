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
// 基于 Rerun 的调试可视化 + 日志记录功能
// 可以方便你去发现问题

#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <memory>
#include <chrono>

#include "rerun/recording_stream.hpp"

namespace ne_vision
{

class NeRerunDebug final
{

public:
  ~NeRerunDebug() = default;

  // 单例
  const NeRerunDebug& operator=(const NeRerunDebug&) = delete;
  NeRerunDebug&       operator=(NeRerunDebug&&) = delete;
  NeRerunDebug(const NeRerunDebug&) = delete;
  NeRerunDebug(NeRerunDebug&&) = delete;

  static NeRerunDebug& GetInstance()
  {
    static NeRerunDebug instance;
    return instance;
  }

  // 初始化 Rerun
  void Init(const std::string& app_name = "ne_vision_debug",
            const std::string& tcp_url = "127.0.0.1:9876")
  {
    std::lock_guard<std::mutex> lock(mtx_);
    app_name_ = app_name;
    tcp_url_ = tcp_url;

    // 构造rerun对象
  }

private:
  NeRerunDebug() = default;

  std::mutex                            mtx_;
  std::string                           app_name_ = "ne_vision_debug";
  std::string                           tcp_url_ = "127.0.0.1:9876";
  std::optional<rerun::RecordingStream> rec_;
};

} // namespace ne_vision
