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
// 整个自瞄系统是通过channel通信的
// 每一个channel可以理解为一个ros的topic

#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <deque>
#include <mutex>
#include <optional>
#include <type_traits>
#include <vector>
#include <algorithm>
#include <tuple>
#include <utility>
#include <concepts>

#include "ne_vision/utils/ne_debug.hpp"
#include "ne_vision/utils/ne_tools.hpp"

namespace ne_vision
{

/// @brief A pair containing a condition variable and a boolean notification
/// flag.
using CvBracket_t = std::pair<std::condition_variable, bool>;
/// @brief A shared pointer to a CvBracket_t, used to manage CVs for waiting
/// tasks.
using CvPairSPtr_t = std::shared_ptr<CvBracket_t>;
using NeChannelStamp_t = std::chrono::steady_clock::time_point;

// 这是读取数据的两种方式，阅后保留或阅后即焚
enum class NeChannelType_e
{
  POP_ON_READ = 0,
  KEEP_ON_READ = 1,
};

struct NeChannelHeader_t
{
  NeChannelHeader_t()
      : frame("default"), stamp(std::chrono::steady_clock::now())
  {
  }

  std::string      frame;
  NeChannelStamp_t stamp;
};

template <typename T>
struct NeChannelData_t
{
  NeChannelHeader_t header;
  T                 data;
};

// 时间戳查询结果。目标时间位于缓存范围内时，first 和 second 包围
// 目标时间；精确命中时二者相同。目标时间越界时，二者均为最近的
// 边界数据。
template <typename T>
struct NeChannelBracket_t
{
  enum
  {
    NO_DATA = 0,              // 缓存中没有数据，first 和 second 无效
    TARGET_BEFORE_OLDEST = 1, // 目标早于最旧数据，二者均为最旧数据
    TARGET_AFTER_NEWEST = 2,  // 目标晚于最新数据，二者均为最新数据
    IN_RANGE = 3 // 目标在缓存范围内；精确命中时 first 和 second 相同
  } status = NO_DATA;

  NeChannelData_t<T> first;
  NeChannelData_t<T> second;
};

/**
 * @brief Base class for channels, providing a mechanism for tasks to wait for
 * data using condition variables. This class encapsulates the synchronization
 * logic.
 */
class NeChannelBase
{
public:
  NeChannelBase() = default;
  virtual ~NeChannelBase() = default;

  void RegisterCv(const CvPairSPtr_t& cv_pair_sPtr)
  {
    std::lock_guard<std::mutex> lock(mtx__);
    NV_ASSERT(cv_pair_sPtr != nullptr);

    // Initialize the notification status to false.
    cv_pair_sPtr->second = false;

    cv_pair_sPtrs_.push_back(cv_pair_sPtr);
  }

  // GetName用于获取当前channel的名称
  // 这个方法是个MAGIC方法，log会识别调用命名空间下是否有GetName方法，
  // 并将对应的名字打印出来，方便调试
  inline std::string GetName() { return name__; }

  void WaitForData(const CvPairSPtr_t& cv_pair_sPtr, std::stop_token& stoken)
  {
    std::unique_lock<std::mutex> lock(mtx__);

    // Wait until the notification status is true, which means new data has been
    // transmitted, or a stop is requested.
    cv_pair_sPtr->first.wait(lock, [&cv_pair_sPtr, &stoken] {
      return cv_pair_sPtr->second || stoken.stop_requested();
    });

    // Reset the notification status to false after being notified.
    cv_pair_sPtr->second = false;
  }

protected:
  /// @brief A list of all registered condition variable pairs to be notified.
  std::vector<CvPairSPtr_t> cv_pair_sPtrs_;
  /// @brief The name of the channel, used for logging and debugging.
  std::string name__ = "UnnamedChannel";
  /// @brief The internal mutex protecting both the data queue and the CV list.
  mutable std::mutex mtx__;
};

template <typename T>
class NeChannel : public NeChannelBase
{
public:
  // Channel名称，类型，大小
  explicit NeChannel(const std::string& name, NeChannelType_e type, size_t size)
      : type_(type), channel_size_(size)
  {
    name__ = name;
    NV_ASSERT(size > 0 && "Channel size must be greater than 0");
  }

  ~NeChannel() = default;

  // 朴实无华的发送函数
  // 标准发送函数
  void Transmit(const T& data, const NeChannelHeader_t& header)
  {
    std::unique_lock<std::mutex> lock(mtx__);

    // 发送数据

    // 拼接数据
    NeChannelData_t<T> data_raw;
    data_raw.header = header;
    data_raw.data = data;

    // 判断当前时间是否符合顺序
    if (!data_queue_.empty() &&
        data_raw.header.stamp >= data_queue_.back().header.stamp)
    {
      // 如果是，直接发送
      data_queue_.push_back(data_raw);
    }
    else
    {
      // 如果不是，进行排序插入
      auto it = std::upper_bound(
          data_queue_.begin(),
          data_queue_.end(),
          data_raw.header.stamp,
          [](const NeChannelStamp_t& stamp, const NeChannelData_t<T>& item) {
            return stamp < item.header.stamp;
          });
      data_queue_.insert(it, data_raw);
    }

    // 维护队列长度
    for (; data_queue_.size() > channel_size_;)
      data_queue_.pop_front();

    // 如果设置了CV，则通知所有等待的线程有新数据到来
    if (cv_pair_sPtrs_.empty())
      return;
    else
      for (const auto& cv_pair_sPtr : cv_pair_sPtrs_)
        if (cv_pair_sPtr != nullptr)
        {
          // Set the notification status to true before notifying.
          cv_pair_sPtr->second = true;
          cv_pair_sPtr->first.notify_all();
        }
  }

  // 简化版发送函数
  void Transmit(const T&         data,
                NeChannelStamp_t stamp = std::chrono::steady_clock::now())
  {
    NeChannelHeader_t header;
    header.stamp = stamp;
    Transmit(data, header);
  }

  // 接收原始数据
  bool ReceiveRaw(NeChannelData_t<T>& data_raw, bool newest = true)
  {
    std::unique_lock<std::mutex> lock(mtx__);

    if (data_queue_.empty())
      return false;

    // 根据 newest 参数决定是取最新的数据还是最旧的数据
    if (newest)
    {
      data_raw = data_queue_.back();
      if (type_ == NeChannelType_e::POP_ON_READ)
        data_queue_.pop_back();
    }
    else
    {
      data_raw = data_queue_.front();
      if (type_ == NeChannelType_e::POP_ON_READ)
        data_queue_.pop_front();
    }
    return true;
  }

  // 接收数据
  bool Receive(T& data, bool newest = true)
  {

    NeChannelData_t<T> data_raw;
    if (!ReceiveRaw(data_raw, newest))
      return false;

    data = data_raw.data;
    return true;
  }

  // 临时兼容
  template <typename Predicate>
  bool Find(T& data, Predicate predicate)
  {
    std::unique_lock<std::mutex> lock(mtx__);

    NV_ASSERT(type_ == NeChannelType_e::KEEP_ON_READ &&
              "Channel type must be KEEP_ON_READ");

    for (const auto& item : data_queue_)
    {
      if (predicate(item.data))
      {
        data = item.data;
        return true;
      }
    }

    return false;
  }

  // 临时兼容
  template <typename TimestampExtractor>
  bool FindClosestPair(const std::chrono::steady_clock::time_point& time,
                       std::pair<T, T>&                             result,
                       TimestampExtractor get_timestamp)
  {
    auto re = FindBracket(time);

    if (re.status != NeChannelBracket_t<T>::NO_DATA)
    {
      result.first = re.first.data;
      result.second = re.second.data;
      return true;
    }

    return false;
  }

  // 寻找两个数据包含特定的时间戳
  NeChannelBracket_t<T> FindBracket(const NeChannelStamp_t& stamp)
  {
    using Bracket_t = NeChannelBracket_t<T>;

    std::unique_lock<std::mutex> lock(mtx__);

    Bracket_t result;

    if (data_queue_.empty())
    {
      result.status = Bracket_t::NO_DATA;
      return result;
    }

    // 如果只有一个数据而且时间不等于，在这里已经解决掉了
    if (stamp < data_queue_.front().header.stamp)
    {
      // 如果所有数据中最老的都比目标大
      result.status = Bracket_t::TARGET_BEFORE_OLDEST;
      result.first = data_queue_.front();
      result.second = data_queue_.front();
    }
    else if (stamp > data_queue_.back().header.stamp)
    {
      // 如果所有数据中最新的都比目标小
      result.status = Bracket_t::TARGET_AFTER_NEWEST;
      result.first = data_queue_.back();
      result.second = data_queue_.back();
    }
    else
    {
      result.status = Bracket_t::IN_RANGE;

      // lower_bound返回第一个大于等于目标的迭代器
      auto it = std::lower_bound(
          data_queue_.begin(),
          data_queue_.end(),
          stamp,
          [](const NeChannelData_t<T>& item, const NeChannelStamp_t& t) {
            return item.header.stamp < t;
          });

      if (it->header.stamp == stamp)
      {
        // 这里只能是等于
        result.first = *it;
        result.second = *it;
      }
      else
      {
        // 这里 it - 1就是小的
        result.first = *(it - 1);
        result.second = *it;
      }
    }

    return result;
  }

  std::deque<NeChannelData_t<T>> GetAllDataRaw()
  {
    std::unique_lock<std::mutex> lock(mtx__);
    return data_queue_;
  }

  std::deque<T> GetAllData()
  {
    std::unique_lock<std::mutex> lock(mtx__);

    std::deque<T> data_only;
    for (const auto& item : data_queue_)
      data_only.push_back(item.data);
    return data_only;
  }

  // 获取最新数据的时间戳
  std::optional<NeChannelStamp_t> GetNewestStamp()
  {
    std::unique_lock<std::mutex> lock(mtx__);
    if (data_queue_.empty())
      return std::nullopt;
    return data_queue_.back().header.stamp;
  }

  // 临时兼容
  template <typename TimestampExtractor>
  std::vector<T> GetDataSince(const std::chrono::steady_clock::time_point& time,
                              TimestampExtractor get_timestamp)
  {
    std::unique_lock<std::mutex> lock(mtx__);
    std::vector<T>               result;

    if (data_queue_.empty())
    {
      return result;
    }

    auto it =
        std::upper_bound(data_queue_.begin(),
                         data_queue_.end(),
                         time,
                         [&](const std::chrono::steady_clock::time_point& t,
                             const NeChannelData_t<T>& item) {
                           return t < get_timestamp(item.data);
                         });

    result.reserve(static_cast<size_t>(std::distance(it, data_queue_.end())));
    for (; it != data_queue_.end(); ++it)
      result.push_back(it->data);
    return result;
  }

  // 当前大小
  inline size_t Size() const
  {
    std::unique_lock<std::mutex> lock(mtx__);
    return data_queue_.size();
  }

  // 判空
  inline bool Empty() const
  {
    std::unique_lock<std::mutex> lock(mtx__);
    return data_queue_.empty();
  }

  // 获取channel的类型
  inline NeChannelType_e GetType() const { return type_; }

  // 公开数据类型
  using MessageType = T;

private:
  // 数据的处理方式
  NeChannelType_e type_;

  // channel的最大容量
  size_t channel_size_ = 0;

  // 数据队列，存储传输的数据
  std::deque<NeChannelData_t<T>> data_queue_;
};

// === 同步器相关 === //

// 这里可能会牵扯一点复杂模版操作

// 对应数据对齐状态
enum class NeChannelSyncMatchStatus_e
{
  FAIL = 0,                 // 失败，一般是没有数据
  IS_TARGET = 1,            // 本数据最新，为时间基准
  SUCCESS = 2,              // 成功，数据已经同步
  TARGET_BEFORE_OLDEST = 3, // 基准数据早于最旧数据，同步数据不准
  TARGET_AFTER_NEWEST = 4,  // 基准数据晚于最新数据，同步数据不准
};

// 对应同步器状态
enum class NeChannelSyncResult_e
{
  SUCCESS,              // 成功，所有数据对齐
  SUCCESS_WITH_WARNING, // 成功但存在未完全对齐的数据
  NOT_READY,            // 部分或全部数据不足
};

// 外部不要使用detail命名空间的东西
namespace detail
{

// 如果有差值函数(如针对ImuData_t)，则应该定义为
// static ImuData_t Interpolate(...)

template <typename T>
concept Interpolatable = requires(const T&                a1,
                                  const T&                a2,
                                  const NeChannelStamp_t& t1,
                                  const NeChannelStamp_t& t2,
                                  const NeChannelStamp_t& target_time) {
  { T::Interpolate(a1, a2, t1, t2, target_time) } -> std::same_as<T>;
};
} // namespace detail

// 记录同步结果和各类同步信息
template <typename T>
struct NeChannelSyncSlot
{

  explicit NeChannelSyncSlot(std::shared_ptr<NeChannel<T>> channel_sptr_)
      : channel_sptr(std::move(channel_sptr_))
  {
  }

  using MsgType = T;
  using RawDataType = NeChannelData_t<T>;

  // 记录对应的哪个channel的数据
  std::shared_ptr<NeChannel<T>> channel_sptr = nullptr;
  // 记录同步结果
  T result;
  // 记录同步状态
  NeChannelSyncMatchStatus_e status = NeChannelSyncMatchStatus_e::FAIL;
  // 数据同步偏差，单位ms，本数据为基数据或数据为插值同步时严格为0
  double time_diff_ms = 0;
  // 是否是基准时间
  bool is_target = false;
  // 是否有差值函数
  bool has_interpolation = false;
};

template <typename... MsgType>
class NeChannelSynchronizer
{
private:
  using SlotTuple_t = std::tuple<NeChannelSyncSlot<MsgType>...>;

public:
  explicit NeChannelSynchronizer(
      std::shared_ptr<NeChannel<MsgType>>... channel_sptr)
      : slots_(NeChannelSyncSlot<MsgType>{std::move(channel_sptr)}...)
  {
    // 遍历tuple，检查指针并初始化数据
    detail::NeTraverseTuple(slots_, [&]<typename T>(const T& slot) {
      // 检查指针是否存在
      NV_ASSERT(slot.channel_sptr && "Channel is invaild! (nullptr)");

      // 检查channel类型是否为keep on read
      NV_ASSERT(slot.channel_sptr->GetType() == NeChannelType_e::KEEP_ON_READ &&
                "Channel type must be KEEP_ON_READ!");
    });
  }

  // 按照最旧的最新时间戳同步（水位线--木桶效应）
  NeChannelSyncResult_e Sync()
  {
    // 本轮同步完成前保持未就绪，防止失败后读取上一轮结果
    curr_sync_result_ = NeChannelSyncResult_e::NOT_READY;

    // 遍历tuple，获取所有数据中最旧的最新时间戳
    // A: .....|..
    // B: .....|
    // C: .....|....
    // 同步目标是这条线（木桶效应）
    curr_target_stamp_ = std::nullopt;
    bool all_channels_ready = true;
    detail::NeTraverseTuple(slots_, [&]<typename T>(const T& slot) {
      if (!all_channels_ready)
        return;

      auto newest_stamp = slot.channel_sptr->GetNewestStamp();
      if (newest_stamp)
      {
        if (curr_target_stamp_ == std::nullopt ||
            *newest_stamp < *curr_target_stamp_)
          curr_target_stamp_ = newest_stamp;
      }
      else
        all_channels_ready = false;
    });
    if (!all_channels_ready || curr_target_stamp_ == std::nullopt)
      return curr_sync_result_;

    // 基于基准时间，遍历并进行同步处理
    bool is_fail = false;
    bool is_warn = false;
    detail::NeTraverseTuple(slots_, [&]<typename Slot>(Slot& slot) {
      if (is_fail)
        return;

      using DataType = typename Slot::MsgType;

      // 就是基准本人
      if (slot.channel_sptr->GetNewestStamp() == curr_target_stamp_)
      {
        // 直接获取最新数据
        slot.channel_sptr->Receive(slot.result, true);
        // 更新同步状态
        slot.status = NeChannelSyncMatchStatus_e::IS_TARGET;
        // 同步偏差
        slot.time_diff_ms = 0;
        // 是基准时间
        slot.is_target = true;
      }
      else
      {
        slot.is_target = false;

        // 先按照时间寻找最近的两个数据
        NeChannelBracket_t<DataType> bracket =
            slot.channel_sptr->FindBracket(*curr_target_stamp_);
        // 找不到数据
        if (bracket.status == NeChannelBracket_t<DataType>::NO_DATA)
        {
          is_fail = true;
          slot.status = NeChannelSyncMatchStatus_e::FAIL;
        }
        // 目标时间在最晚时间之前
        else if (bracket.status ==
                 NeChannelBracket_t<DataType>::TARGET_BEFORE_OLDEST)
        {
          slot.status = NeChannelSyncMatchStatus_e::TARGET_BEFORE_OLDEST;
          slot.result = bracket.first.data;
          slot.time_diff_ms =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  bracket.first.header.stamp - *curr_target_stamp_)
                  .count();
          is_warn = true;
        }
        // 目标时间在最晚时间之后
        else if (bracket.status ==
                 NeChannelBracket_t<DataType>::TARGET_AFTER_NEWEST)
        {
          slot.status = NeChannelSyncMatchStatus_e::TARGET_AFTER_NEWEST;
          slot.result = bracket.first.data;
          slot.time_diff_ms =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  bracket.first.header.stamp - *curr_target_stamp_)
                  .count();
          is_warn = true;
        }
        // IN range
        else
        {
          slot.status = NeChannelSyncMatchStatus_e::SUCCESS;
          if constexpr (detail::Interpolatable<DataType>)
          {
            // 插值
            slot.result = DataType::Interpolate(bracket.first.data,
                                                bracket.second.data,
                                                bracket.first.header.stamp,
                                                bracket.second.header.stamp,
                                                *curr_target_stamp_);
            slot.time_diff_ms = 0;
          }
          else
          {
            // 使用最近邻
            const auto first_diff =
                *curr_target_stamp_ - bracket.first.header.stamp;
            const auto second_diff =
                bracket.second.header.stamp - *curr_target_stamp_;

            // 距离相同时选择时间更早的数据，保证结果稳定
            const auto& nearest =
                first_diff <= second_diff ? bracket.first : bracket.second;
            slot.result = nearest.data;
            slot.time_diff_ms = std::chrono::duration<double, std::milli>(
                                    nearest.header.stamp - *curr_target_stamp_)
                                    .count();
          }
        }
      }
    });

    if (is_fail)
      return curr_sync_result_;

    if (is_warn)
    {
      curr_sync_result_ = NeChannelSyncResult_e::SUCCESS_WITH_WARNING;
      return curr_sync_result_;
    }

    curr_sync_result_ = NeChannelSyncResult_e::SUCCESS;
    return curr_sync_result_;
  }

  // 获取当前同步时间戳
  inline NeChannelStamp_t GetTargetStamp() const
  {
    NV_ASSERT(curr_sync_result_ != NeChannelSyncResult_e::NOT_READY &&
              "Synchronizer is not ready! Call Sync() successfully first.");
    return *curr_target_stamp_;
  }

  // 获取同步详细信息，需要提前构造slot，不直接，通常不建议使用
  template <typename DataType>
  NeChannelSyncMatchStatus_e
  GetResultSlot(const std::shared_ptr<NeChannel<DataType>>& channel_sptr,
                NeChannelSyncSlot<DataType>&                result) const
  {
    NV_ASSERT(curr_sync_result_ != NeChannelSyncResult_e::NOT_READY &&
              "Synchronizer is not ready! Call Sync() successfully first.");

    NeChannelSyncMatchStatus_e ms = NeChannelSyncMatchStatus_e::FAIL;
    bool                       found = false;
    detail::NeTraverseTuple(slots_, [&]<typename Slot>(const Slot& slot) {
      if (ms != NeChannelSyncMatchStatus_e::FAIL)
        return;
      if constexpr (std::is_same_v<typename Slot::MsgType, DataType>)
      {
        if (slot.channel_sptr == channel_sptr)
        {
          ms = slot.status;
          result = slot;
          found = true;
        }
      }
    });
    NV_ASSERT(found && "The channel not include in the Synchronizer!");
    return ms;
  }

  // 获取同步结果：最直接
  template <typename DataType>
  NeChannelSyncMatchStatus_e
  GetResultData(const std::shared_ptr<NeChannel<DataType>>& channel_sptr,
                DataType&                                   result) const
  {
    NV_ASSERT(curr_sync_result_ != NeChannelSyncResult_e::NOT_READY &&
              "Synchronizer is not ready! Call Sync() successfully first.");

    NeChannelSyncMatchStatus_e ms = NeChannelSyncMatchStatus_e::FAIL;
    bool                       found = false;
    detail::NeTraverseTuple(slots_, [&]<typename Slot>(const Slot& slot) {
      if (ms != NeChannelSyncMatchStatus_e::FAIL)
        return;
      if constexpr (std::is_same_v<typename Slot::MsgType, DataType>)
      {
        if (slot.channel_sptr == channel_sptr)
        {
          ms = slot.status;
          result = slot.result;
          found = true;
        }
      }
    });
    NV_ASSERT(found && "The channel not include in the Synchronizer!");
    return ms;
  }

private:
  SlotTuple_t                     slots_;
  std::optional<NeChannelStamp_t> curr_target_stamp_;
  NeChannelSyncResult_e curr_sync_result_ = NeChannelSyncResult_e::NOT_READY;
};

} // namespace ne_vision
