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

#include "ne_vision/utils/ne_task.hpp"
#include "ne_vision/utils/ne_log.hpp"

namespace ne_vision
{

NeTask::~NeTask() { Stop(); }

void NeTask::Start()
{
  if (task_thread_.joinable())
  {
    NV_ERROR("[{}] you have already started the task", name_);
    return;
  }

  if (task_type_ == NeTaskType_e::WAIT_FOR_CHANNEL_DATA)
  {
    if (!cv_pair_sPtr_)
    {
      NV_ERROR("you have to register a condition variable to the channel "
               "before starting the task");
      return;
    }

    task_thread_ = std::jthread([this](std::stop_token stoken) {
      NV_INFO("task started, waiting for channel: {}",
              channel_sPtr_->GetName());

      while (!stoken.stop_requested())
      {
        // Wait for the channel to notify that new data has been transmitted.
        // This Method is thread safty
        channel_sPtr_->WaitForData(cv_pair_sPtr_, stoken);
        if (stoken.stop_requested())
          break;

        // Calculate the task interval and runtime.
        execution_start_time_ = std::chrono::steady_clock::now();
        task_interval_s_ = std::chrono::duration<double>(execution_start_time_ -
                                                         last_execution_time_)
                               .count();
        last_execution_time_ = execution_start_time_;

        task_();

        auto execution_end_time = std::chrono::steady_clock::now();
        task_runtime_s_ = std::chrono::duration<double>(execution_end_time -
                                                        execution_start_time_)
                              .count();
      }
      NV_INFO("task stopped.");
    });
  }
  else if (task_type_ == NeTaskType_e::WAIT_FOR_INTERVAL)
  {
    task_thread_ = std::jthread([this](std::stop_token stoken) {
      NV_INFO("task started, task will run every: {} ms",
              time_interval_.count());

      std::mutex mtx;

      auto next_tick = std::chrono::steady_clock::now() + time_interval_;

      while (!stoken.stop_requested())
      {
        next_tick += time_interval_;

        // Calculate the task interval and runtime.
        execution_start_time_ = std::chrono::steady_clock::now();
        task_interval_s_ = std::chrono::duration<double>(execution_start_time_ -
                                                         last_execution_time_)
                               .count();
        last_execution_time_ = execution_start_time_;

        task_();

        auto execution_end_time = std::chrono::steady_clock::now();
        task_runtime_s_ = std::chrono::duration<double>(execution_end_time -
                                                        execution_start_time_)
                              .count();

        std::unique_lock<std::mutex> lock(mtx);
        cv_pair_sPtr_->first.wait_until(
            lock, next_tick, [&stoken] { return stoken.stop_requested(); });
      }
      NV_INFO("task stopped.");
    });
  }
  else
  {
    NV_ASSERT(false && "Invalid task type");
  }
}

void NeTask::WakeUp()
{
  if (task_thread_.joinable() && cv_pair_sPtr_)
  {
    cv_pair_sPtr_->first.notify_all();
  }
}

void NeTask::Stop()
{
  if (task_thread_.joinable())
  {
    task_thread_.request_stop();
    WakeUp();
    task_thread_.join();
  }
}

} // namespace ne_vision
