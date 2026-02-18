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

/**
 * @file ne_channel.hpp
 * @author ziyedeyuu@163.com (Zhaoyu Chen)
 * @brief A thread-safe channel mechanism for CSP-style inter-thread
 * communication.
 *
 * Provides a generic channel class for message passing between threads.
 * It supports different behaviors on read (pop or keep) and allows tasks
 * to wait efficiently for new data using condition variables.
 */

#pragma once

#include <cmath>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <deque>
#include <mutex>
#include <vector>

#include "ne_vision/utils/ne_debug.hpp"

namespace ne_vision
{

/**
 * @brief Defines the behavior of the channel when data is read.
 */
enum class NeChannelType_e
{
  POP_ON_READ = 0,  ///< The data will be popped after being read.
  KEEP_ON_READ = 1, ///< The data will be kept after being read.
};

/// @brief A pair containing a condition variable and a boolean notification
/// flag.
using CvPair_t = std::pair<std::condition_variable, bool>;
/// @brief A shared pointer to a CvPair_t, used to manage CVs for waiting tasks.
using CvPairSPtr_t = std::shared_ptr<CvPair_t>;

/**
 * @brief Base class for channels, providing a mechanism for tasks to wait for
 * data using condition variables. This class encapsulates the synchronization
 * logic.
 */
class NeChannelBase
{
public:
  /**
   * @brief Default constructor.
   */
  NeChannelBase() = default;
  /**
   * @brief Default virtual destructor.
   */
  virtual ~NeChannelBase() = default;

  /**
   * @brief Registers a condition variable pair to be notified when new data is
   * transmitted.
   * @note This method is thread-safe.
   * @param cv_pair_sPtr A shared pointer to the condition variable pair to
   * register.
   */
  void RegisterCv(const CvPairSPtr_t& cv_pair_sPtr)
  {
    std::lock_guard<std::mutex> lock(mtx__);
    NV_ASSERT(cv_pair_sPtr != nullptr);

    // Initialize the notification status to false.
    cv_pair_sPtr->second = false;

    cv_pair_sPtrs_.push_back(cv_pair_sPtr);
  }

  /**
   * @brief Gets the name of the channel.
   * @return The name of the channel.
   */
  inline std::string GetName() { return name__; }

  /**
   * @brief Blocks the calling thread until notified or a stop is requested.
   *
   * This method encapsulates the condition variable waiting logic. It locks the
   * channel's internal mutex and waits on the condition variable associated
   * with the provided cv_pair_sPtr. The thread will unblock when another thread
   * calls Transmit() or when the stop_token is triggered. After waiting, it
   * resets the notification flag.
   *
   * @note This method is thread-safe.
   * @param cv_pair_sPtr The specific condition variable pair this thread should
   * wait on.
   * @param stoken A stop_token to allow for early exit from waiting.
   */
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
  std::mutex mtx__;
};

/**
 * @brief A thread-safe, single-producer, multi-consumer channel for message
 * passing.
 *
 * This class provides a fixed-size buffer for communication between threads.
 * When the buffer is full, transmitting new data will cause the oldest data to
 * be dropped.
 * @tparam T The type of data to be transmitted through the channel.
 */
template <typename T>
class NeChannel : public NeChannelBase
{
public:
  /**
   * @brief Construct a new NeChannel object.
   *
   * @param name The name of the channel, used for logging/debugging.
   * @param type The behavior of the channel on receive (pop or keep).
   * @param size The maximum size of the internal data buffer.
   */
  explicit NeChannel(const std::string& name, NeChannelType_e type, size_t size)
      : type_(type), channel_size_(size)
  {
    name__ = name;
  }
  /**
   * @brief Default destructor.
   */
  ~NeChannel() = default;

  /**
   * @brief Transmits data into the channel.
   *
   * If the channel's buffer is full, the oldest data will be removed to make
   * space. If any tasks are waiting on a registered condition variable, they
   * will be notified.
   * @note This method is thread-safe.
   * @param data The data to be transmitted.
   */
  void Transmit(const T& data)
  {
    std::unique_lock<std::mutex> lock(mtx__);

    for (; data_queue_.size() >= channel_size_;)
    {
      data_queue_.pop_front();
    }
    data_queue_.push_back(data);

    // If cv is set, notify the waiting task to read data immediately.
    if (cv_pair_sPtrs_.empty())
    {
      return;
    }
    else
    {
      for (const auto& cv_pair_sPtr : cv_pair_sPtrs_)
      {
        if (cv_pair_sPtr != nullptr)
        {
          // Set the notification status to true before notifying.
          cv_pair_sPtr->second = true;
          cv_pair_sPtr->first.notify_all();
        }
      }
    }
  }

  /**
   * @brief Receives data from the channel.
   *
   * @note This method is thread-safe.
   * @param[out] data A reference to store the received data.
   * @param newest If true, retrieves the newest data without popping it (only
   * applicable for KEEP_ON_READ channels). If false, retrieves the oldest data
   * (default behavior).
   * @return true if data was successfully received, false if the channel was
   * empty.
   */
  bool Receive(T& data, bool newest = false)
  {
    std::unique_lock<std::mutex> lock(mtx__);

    if (data_queue_.empty())
    {
      return false;
    }

    // Copy the data, and pop it if the channel type is POP_ON_READ.
    if (newest && type_ == NeChannelType_e::KEEP_ON_READ)
    {
      data = data_queue_.back();
      if (type_ == NeChannelType_e::POP_ON_READ)
      {
        data_queue_.pop_back();
      }
    }
    else
    {
      data = data_queue_.front();
      if (type_ == NeChannelType_e::POP_ON_READ)
      {
        data_queue_.pop_front();
      }
    }
    return true;
  }

  /**
   * @brief Finds an element in the channel that satisfies a predicate.
   *
   * This method is only available for channels of type KEEP_ON_READ.
   * It iterates through the data queue and returns the first element
   * for which the predicate returns true.
   *
   * @note This method is thread-safe.
   * @param[out] data A reference to store the found data.
   * @param predicate A unary predicate that takes an element of type const T&
   *                  and returns true if it's the desired element.
   * @return true if an element was found, false otherwise.
   */
  template <typename Predicate>
  bool Find(T& data, Predicate predicate)
  {
    std::unique_lock<std::mutex> lock(mtx__);

    NV_ASSERT(type_ == NeChannelType_e::KEEP_ON_READ &&
              "Channel type must be KEEP_ON_READ");

    for (const auto& item : data_queue_)
    {
      if (predicate(item))
      {
        data = item;
        return true;
      }
    }

    return false;
  }

  /**
   * @brief Finds the two elements in the channel with timestamps closest to the
   * given time.
   *
   * This method is only available for channels of type KEEP_ON_READ.
   * It performs a binary search to find the two closest elements.
   * The elements in the channel are expected to be sorted by timestamp.
   *
   * @note This method is thread-safe.
   * @param time The time point to search for.
   * @param[out] result A pair to store the two closest elements.
   * @return true if a pair was found (i.e., channel has >= 2 elements), false
   * otherwise.
   * @tparam TimestampExtractor A callable that takes a const T& and returns a
   * std::chrono::steady_clock::time_point.
   */
  template <typename TimestampExtractor>
  bool FindClosestPair(const std::chrono::steady_clock::time_point& time,
                       std::pair<T, T>&                             result,
                       TimestampExtractor get_timestamp)
  {
    std::unique_lock<std::mutex> lock(mtx__);

    NV_ASSERT(type_ == NeChannelType_e::KEEP_ON_READ &&
              "Channel type must be KEEP_ON_READ");

    if (data_queue_.size() < 2)
    {
      return false;
    }

    auto it = std::lower_bound(
        data_queue_.begin(),
        data_queue_.end(),
        time,
        [&](const T& element, const std::chrono::steady_clock::time_point& t) {
          return get_timestamp(element) < t;
        });

    if (it == data_queue_.begin())
    {
      result = {data_queue_[0], data_queue_[1]};
    }
    else if (it == data_queue_.end())
    {
      result = {data_queue_[data_queue_.size() - 2],
                data_queue_[data_queue_.size() - 1]};
    }
    else
    {
      result = {*(it - 1), *it};
    }

    return true;
  }

  /**
   * @brief Gets the current number of elements in the channel.
   * @return The number of elements.
   */
  inline size_t Size() const { return data_queue_.size(); }

  /**
   * @brief Checks if the channel is empty.
   * @return true if the channel is empty, false otherwise.
   */
  inline bool Empty() const { return data_queue_.empty(); }

private:
  /// @brief The behavior of the channel on receive.
  NeChannelType_e type_;
  /// @brief The maximum size of the data buffer.
  size_t channel_size_ = 0;
  /// @brief The internal deque used as a data buffer.
  std::deque<T> data_queue_;
};

} // namespace ne_vision
