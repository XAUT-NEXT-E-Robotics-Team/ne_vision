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
// 串口驱动
// 改自NJQ版的串口驱动
//
// 一个标准的串口协议应该为:
// [Header] 2 bytes: eg: 0xAA 0x55
// [Cmd] 1 byte: 区分不同类型的包
// [Payload] N bytes: N个字节的包，直接memcpy
// [CRC16] 2 bytes: 从Header开始到Payload结束的所有字节的CRC16校验码
//

#pragma once

#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <thread>
#include <type_traits>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "ne_serial_driver/ne_ring_buffer.hpp"
#include "ne_serial_driver/crc.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_debug.hpp"

namespace ne_vision
{
namespace drivers
{

enum class BaudRate_e : int
{
  B_9600 = 9600,
  B_57600 = 57600,
  B_115200 = 115200,
  B_230400 = 230400,
  B_460800 = 460800,
  B_921600 = 921600,
};

class NeSerialDriver;

namespace detail
{
// 判断是否这个结构体协议能被memcpy
template <typename T>
concept ProtocolStruct = std::is_trivially_copyable_v<T>;

// 用来进行类型擦除
class NeProtocolHandlerBase
{
  friend class ne_vision::drivers::NeSerialDriver;

public:
  explicit NeProtocolHandlerBase(uint8_t cmd, size_t size)
      : size_(size), cmd_(cmd)
  {
  }

  uint8_t Cmd() const { return cmd_; }
  size_t  Size() const { return size_; }

protected:
  virtual bool copyFrame(uint8_t* data, size_t size) = 0;
  virtual void notify() = 0;
  virtual void startReceive() = 0;
  virtual void stopReceive() = 0;
  virtual bool isTimeout() const = 0;

private:
  // 能用就行，别管太多
  bool CopyFrame(uint8_t* data, size_t size) { return copyFrame(data, size); }
  void Notify() { notify(); }
  void StartReceive() { startReceive(); }
  void StopReceive() { stopReceive(); }
  bool Istimeout() const { return isTimeout(); }

  size_t  size_ = 0;
  uint8_t cmd_ = 0;
};

} // namespace detail

template <detail::ProtocolStruct T>
class NeProtocolHandler : public detail::NeProtocolHandlerBase
{
private:
  using Callback_t = std::function<void(const T&)>;

public:
  NeProtocolHandler(uint8_t cmd, int timeout_ms = -1)
      : pack_(T{}), size_(sizeof(T)), cmd_(cmd),
        detail::NeProtocolHandlerBase(cmd, sizeof(T)), timeout_ms_(timeout_ms)
  {
  }

  ~NeProtocolHandler() { stopReceive(); }

  T GetPack() const { return pack_.load(std::memory_order_acquire); }

  void AddCallBack(const Callback_t& callback) { callback_ = callback; }

private:
  bool copyFrame(uint8_t* data, size_t size) override
  {
    // 将会在解开包线程中被调用
    // 该函数只保留最新数据
    NV_ASSERT(data && "Data must not be nullptr");
    if (size != size_)
      return false;
    T tmp;
    std::memcpy(&tmp, data, size_);
    pack_.store(tmp, std::memory_order_release);
    last_recv_ns_.store(
        std::chrono::steady_clock::now().time_since_epoch().count(),
        std::memory_order_release);
    return true;
  }

  void notify() override
  {
    notified_.store(true, std::memory_order_release);
    notified_.notify_one();
  }

  void startReceive() override
  {
    if (!callback_ || running_.load())
      return;
    running_.store(true);

    // 专门给毁回调执行准备的线程
    callback_thread_ = std::jthread([this](std::stop_token st) {
      while (!st.stop_requested())
      {
        notified_.wait(false, std::memory_order_acquire);
        if (st.stop_requested())
          break;
        notified_.store(false, std::memory_order_relaxed);
        callback_(pack_.load(std::memory_order_acquire));
      }
    });
  }

  void stopReceive() override
  {
    running_.store(false);
    callback_thread_.request_stop();
    notified_.store(true);
    notified_.notify_all();
  }

  bool isTimeout() const override
  {
    if (timeout_ms_ < 0)
      return false;
    int64_t last = last_recv_ns_.load(std::memory_order_acquire);
    if (last == 0)
      return true;
    int64_t now = std::chrono::steady_clock::now().time_since_epoch().count();
    return (now - last) > timeout_ms_ * 1'000'000;
  }

  std::jthread         callback_thread_;
  std::atomic<T>       pack_;
  std::atomic<bool>    notified_{false};
  std::atomic<bool>    running_{false};
  std::atomic<int64_t> last_recv_ns_{0};
  Callback_t           callback_;
  size_t               size_ = 0;
  uint8_t              cmd_ = 0;
  int64_t              timeout_ms_ = -1;
};

class NeSerialDriver
{
public:
  explicit NeSerialDriver(const std::string& port,
                          BaudRate_e         baud_rate = BaudRate_e::B_115200,
                          uint16_t           header = 0xAA55,
                          size_t             ring_buffer_size = 1024);
  ~NeSerialDriver();

  // cmd是包的标识符
  // timeout_ms是包的超时时间，单位毫秒，负数表示不检测超时
  template <detail::ProtocolStruct T>
  std::shared_ptr<NeProtocolHandler<T>>
  NewProtocolHandler(uint8_t cmd, int64_t timeout_ms = -1)
  {
    auto handle = std::make_shared<NeProtocolHandler<T>>(cmd, timeout_ms);
    protocol_handlers_.push_back(handle);
    return handle;
  }

  // 本函数不是打开串口，而是启动一个串口接收线程来管理串口，因此能实现断线重连
  // interval_ms是重试打开串口的时间间隔，单位毫秒
  // timeout_ms是打开串口的超时时间（尝试的超时时间）
  // 如果timeout_ms为负数，则会一直尝试打开串口直到成功
  void Open(int interval_ms = 1000, int timeout_ms = -1);

  void Stop();

  bool IsOpen() const;

  bool Transmit(uint8_t* buffer, int write_size);

  template <detail::ProtocolStruct T>
  bool TransmitProtocol(uint8_t cmd, const T& protocol)
  {
    // frame: [hdr_hi][hdr_lo][cmd][payload(N)][crc16_lo][crc16_hi]
    constexpr size_t payload_size = sizeof(T);
    constexpr size_t frame_size = 3 + payload_size + 2;
    uint8_t          frame[frame_size];
    frame[0] = (header_ >> 8) & 0xFF;
    frame[1] = header_ & 0xFF;
    frame[2] = cmd;
    std::memcpy(frame + 3, &protocol, payload_size);
    ne_io::Append_CRC16_Check_Sum(frame, frame_size);
    return Transmit(frame, frame_size);
  }

  // 给日志库用的
  std::string GetName() { return "ne_serial_driver"; }

private:
  uint16_t    header_ = 0xAA55;
  BaudRate_e  baud_rate_ = BaudRate_e::B_115200;
  std::string port_;

  std::vector<std::shared_ptr<detail::NeProtocolHandlerBase>>
      protocol_handlers_;

  std::jthread receive_thread_;
  std::jthread decode_thread_;

  size_t buffer_size_ = 1024;

  std::atomic<int>  fd_{-1};
  std::atomic<bool> is_open_{false};

  std::shared_ptr<ne_serial::NeRingBuffer<uint8_t>> receive_buffer_sPtr_;
};

} // namespace drivers
} // namespace ne_vision
