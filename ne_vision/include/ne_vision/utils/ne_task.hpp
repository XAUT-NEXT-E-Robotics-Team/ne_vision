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
// Muti-task(multi-thread).
// 1. Wait for channel data.
// 2. Wait until a specified time interval.

#pragma once

#include "gtest/gtest.h"
#include <chrono>
#include <functional>
#include <string>
#include <condition_variable>
#include <memory>
#include <string_view>
#include <thread>

#include "ne_vision/utils/ne_channel.hpp"
#include "ne_vision/utils/ne_debug.hpp"

namespace ne_vision
{

// You can use ms or s to describ time interval, such as 100ms or 1s.
using namespace std::chrono_literals;

enum class NeTaskType_e
{
  WAIT_FOR_CHANNEL_DATA = 0,
  WAIT_FOR_INTERVAL = 1,
};

class NeTask final
{

public:
  ~NeTask();

  template <typename T, typename Func, typename... Args>
  explicit NeTask(const std::string&        name,
                  NeTaskType_e              type,
                  std::chrono::milliseconds time_interval,
                  T*                        obj,
                  Func                      func,
                  Args&&... args)
  {
    if (type == NeTaskType_e::WAIT_FOR_CHANNEL_DATA)
    {
      NV_ASSERT(false &&
                "Channel must be provided for WAIT_FOR_CHANNEL_DATA type");
    }
    init(name,
         type,
         time_interval,
         nullptr,
         obj,
         func,
         std::forward<Args>(args)...);
  }

  template <typename T, typename Func, typename... Args>
  explicit NeTask(const std::string&                    name,
                  NeTaskType_e                          type,
                  const std::shared_ptr<NeChannelBase>& channel_sPtr,
                  T*                                    obj,
                  Func                                  func,
                  Args&&... args)
  {
    if (type == NeTaskType_e::WAIT_FOR_INTERVAL)
    {
      NV_ASSERT(false &&
                "Time interval must be provided for WAIT_FOR_INTERVAL type");
    }

    init(name, type, 0ms, channel_sPtr, obj, func, std::forward<Args>(args)...);
  }

  void Start();
  void WakeUp();
  void Stop();

  inline std::string GetName() const { return name_; }

private:
  using task_t = std::function<void()>;

  template <typename T, typename Func, typename... Args>
  void init(const std::string&                    name,
            NeTaskType_e                          type,
            std::chrono::milliseconds             time_interval,
            const std::shared_ptr<NeChannelBase>& channel_sPtr,
            T*                                    obj,
            Func                                  func,
            Args&&... args)
  {
    name_ = name;
    task_type_ = type;

    cv_pair_sPtr_ =
        std::make_shared<std::pair<std::condition_variable, bool>>();
    cv_pair_sPtr_->second = false;

    // If the type is equal to WAIT_FOR_CHANNEL_DATA
    // Create a shared pointer to a pair of condition variable and
    // notification status.
    // Then register the condition variable to the channel
    if (type == NeTaskType_e::WAIT_FOR_CHANNEL_DATA)
    {
      NV_ASSERT(channel_sPtr != nullptr &&
                "Channel must be provided for WAIT_FOR_CHANNEL_DATA type");

      channel_sPtr_ = channel_sPtr;

      channel_sPtr_->RegisterCv(cv_pair_sPtr_);
    }
    // If the type is equal to WAIT_FOR_INTERVAL, store the time interval.
    else if (type == NeTaskType_e::WAIT_FOR_INTERVAL)
    {
      time_interval_ = time_interval;
    }

    task_ =
        [obj, func, ... captured_args = std::forward<Args>(args)]() mutable {
          std::invoke(func, obj, captured_args...);
        };
    NV_ASSERT(task_ != nullptr);
  }

  std::shared_ptr<NeChannelBase> channel_sPtr_;
  CvPairSPtr_t                   cv_pair_sPtr_ = nullptr;
  std::string                    name_ = "Unnamed Task";
  NeTaskType_e                   task_type_;
  task_t                         task_;
  std::chrono::milliseconds      time_interval_ = 1000ms;

  std::jthread task_thread_;

  // Monitor the runtime of the task and the interval between two executions.
  std::chrono::steady_clock::time_point last_execution_time_ =
      std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point execution_start_time_ =
      std::chrono::steady_clock::now();
  double task_runtime_s_ = 0.0;
  double task_interval_s_ = 0.0;
};

}; // namespace ne_vision
