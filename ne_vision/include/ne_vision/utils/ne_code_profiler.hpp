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
// 用于多线程分析代码运行时间的工具
// RALL + 多例模式

#include <memory>
#include <functional>
#include <chrono>
#include <string>
#include <mutex>
#include <deque>
#include <unordered_map>

#pragma once

namespace ne_vision
{

class NeCodeProfiler final
{
public:
  struct ResultData_t
  {
    inline double GetAvgDurationS() const { return avg_duration; }
    inline double GetAvgIntervalS() const { return avg_interval; }
    inline double GetLastDurationS() const { return last_duration; }
    inline double GetLastIntervalS() const { return last_interval; }

    double avg_duration  = 0.0;
    double avg_interval  = 0.0;
    double last_duration = 0.0;
    double last_interval = 0.0;
  };

  static std::shared_ptr<NeCodeProfiler> GetInstance(const std::string& name)
  {
    std::lock_guard<std::mutex> lock(creation_mtx_);
    auto                        it = profiler_map_.find(name);

    if (it == profiler_map_.end())
    {
      auto profiler = std::shared_ptr<NeCodeProfiler>(new NeCodeProfiler(name));
      profiler_map_[name] = profiler;
      return profiler;
    }
    return it->second;
  }

  static void ForEach(
      const std::function<void(const std::string&, NeCodeProfiler&)>& fn)
  {
    std::lock_guard<std::mutex> lock(creation_mtx_);
    for (auto& [name, profiler] : profiler_map_)
      fn(name, *profiler);
  }

  NeCodeProfiler(const NeCodeProfiler&) = delete;
  NeCodeProfiler& operator=(const NeCodeProfiler&) = delete;
  NeCodeProfiler(NeCodeProfiler&&) = delete;
  NeCodeProfiler& operator=(NeCodeProfiler&&) = delete;

  inline void
  RecordStart(std::chrono::steady_clock::time_point current_start_time)
  {
    std::lock_guard<std::mutex> lock(data_mtx_);

    if (last_start_time_.time_since_epoch().count() != 0)
    {
      double interval = std::chrono::duration<double>(
                            current_start_time - last_start_time_).count();
      last_interval_ = interval;
      interval_window_.push_back(interval);
    }
    last_start_time_ = current_start_time;
  }

  inline void RecordEnd(std::chrono::duration<double> elapsed_time)
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    last_duration_ = elapsed_time.count();
    duration_window_.push_back({std::chrono::steady_clock::now(), last_duration_});
  }

  inline ResultData_t GetResult()
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    auto now = std::chrono::steady_clock::now();

    while (!duration_window_.empty() &&
           std::chrono::duration<double>(now - duration_window_.front().first).count() > window_s_)
      duration_window_.pop_front();

    while (!interval_window_.empty() && interval_window_.size() > duration_window_.size() + 1)
      interval_window_.pop_front();

    ResultData_t r;
    r.last_duration = last_duration_;
    r.last_interval = last_interval_;

    if (!duration_window_.empty())
    {
      double sum = 0;
      for (auto& [t, v] : duration_window_) sum += v;
      r.avg_duration = sum / duration_window_.size();
    }
    if (!interval_window_.empty())
    {
      double sum = 0;
      for (auto v : interval_window_) sum += v;
      r.avg_interval = sum / interval_window_.size();
    }
    return r;
  }

private:
  explicit NeCodeProfiler(const std::string& name) : name_(name) {}

  inline static std::unordered_map<std::string, std::shared_ptr<NeCodeProfiler>>
                           profiler_map_;
  inline static std::mutex creation_mtx_;

  double window_s_ = 1.0;

  double last_duration_ = 0.0;
  double last_interval_ = 0.0;

  std::deque<std::pair<std::chrono::steady_clock::time_point, double>> duration_window_;
  std::deque<double> interval_window_;

  std::string name_;
  std::mutex  data_mtx_;

  std::chrono::steady_clock::time_point last_start_time_{};
};

class NeProfileScope final
{
public:
  NeProfileScope(const std::string& name)
      : profiler_(NeCodeProfiler::GetInstance(name)),
        start_time_(std::chrono::steady_clock::now())
  {
    profiler_->RecordStart(start_time_);
  }

  ~NeProfileScope()
  {
    auto end_time = std::chrono::steady_clock::now();
    profiler_->RecordEnd(end_time - start_time_);
  }

private:
  std::shared_ptr<NeCodeProfiler>       profiler_;
  std::chrono::steady_clock::time_point start_time_;
};

#define NV_PROFILE_BLOCK(name)                                                 \
  ne_vision::NeProfileScope profile_scope_##__LINE__(name)
#define NV_PROFILE_INSTANCE(name) ne_vision::NeCodeProfiler::GetInstance(name)

} // namespace ne_vision
