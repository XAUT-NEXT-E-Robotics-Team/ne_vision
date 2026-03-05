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
#include <chrono>
#include <string>
#include <mutex>
#include <unordered_map>

#pragma once

namespace ne_vision
{

class NeCodeProfiler final
{
public:
  struct ResultData
  {
    inline double GetAvgS() const
    {
      return count > 0 ? total_time.count() / count : 0.0;
    }

    inline double GetAvgPeriodS() const
    {
      return period_count > 0 ? total_period.count() / period_count : 0.0;
    }

    inline double GetCurrentS() const { return current_time.count(); }

    // 运行时间分析
    std::chrono::duration<double> total_time{0};
    size_t                        count{0};
    std::chrono::duration<double> current_time{0};

    // 运行间隔分析
    std::chrono::duration<double> total_period{0};
    size_t                        period_count{0};
    std::chrono::duration<double> current_period{0};
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

  NeCodeProfiler(const NeCodeProfiler&) = delete;
  NeCodeProfiler& operator=(const NeCodeProfiler&) = delete;
  NeCodeProfiler(NeCodeProfiler&&) = delete;
  NeCodeProfiler& operator=(NeCodeProfiler&&) = delete;

  inline void
  RecordStart(std::chrono::steady_clock::time_point current_start_time)
  {
    std::lock_guard<std::mutex> lock(data_mtx_);

    // 第一次不计算周期
    if (result_data_.period_count > 0)
    {
      result_data_.current_period = current_start_time - last_start_time_;
      result_data_.total_period += result_data_.current_period;
    }

    result_data_.period_count++;
    last_start_time_ = current_start_time;
  }

  inline void RecordEnd(std::chrono::duration<double> elapsed_time)
  {
    std::lock_guard<std::mutex> lock(data_mtx_);

    result_data_.current_time = elapsed_time;
    result_data_.total_time += elapsed_time;
    result_data_.count++;
  }

  inline ResultData GetResult()
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    return result_data_;
  }

private:
  explicit NeCodeProfiler(const std::string& name) : name_(name) {}

  inline static std::unordered_map<std::string, std::shared_ptr<NeCodeProfiler>>
                           profiler_map_;
  inline static std::mutex creation_mtx_;

  ResultData result_data_;

  std::string name_;
  std::mutex  data_mtx_;

  std::chrono::steady_clock::time_point last_start_time_;
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
