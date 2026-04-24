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
// SPSC无锁环形缓冲区

#pragma once

#include <atomic>
namespace ne_vision
{

namespace ne_serial
{

template <typename T>
class NeRingBuffer
{
public:
  // 真实容量为capacity-1
  NeRingBuffer(size_t capacity) : capacity_(capacity)
  {
    buffer_.resize(capacity_);
  }

  // Push 满了返回false
  bool Push(const T& item)
  {
    size_t t = tail_.load(std::memory_order_relaxed);
    size_t next = (t + 1) % capacity_;
    if (next == head_.load(std::memory_order_acquire))
      return false;
    buffer_[t] = item;
    tail_.store(next, std::memory_order_release);
    return true;
  }

  // Pop 空了返回false
  bool Pop(T& item)
  {
    size_t h = head_.load(std::memory_order_relaxed);
    if (h == tail_.load(std::memory_order_acquire))
      return false;
    item = buffer_[h];
    head_.store((h + 1) % capacity_, std::memory_order_release);
    return true;
  }

private:
  size_t capacity_ = 0;
  // 防止伪共享，head和tail分开缓存行
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};

  std::vector<T> buffer_;
};

} // namespace ne_serial

} // namespace ne_vision