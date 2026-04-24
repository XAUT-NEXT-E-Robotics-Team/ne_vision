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
// 串口驱动实现

#include "ne_serial_driver/ne_serial_driver.hpp"
#include "ne_serial_driver/crc.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <vector>
#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#endif

namespace ne_vision
{
namespace drivers
{

NeSerialDriver::NeSerialDriver(const std::string& port,
                               BaudRate_e         baud_rate,
                               uint16_t           header,
                               size_t             buffer_size)
    : port_(port), baud_rate_(baud_rate), header_(header),
      buffer_size_(buffer_size)
{
  receive_buffer_sPtr_ =
      std::make_shared<ne_serial::NeRingBuffer<uint8_t>>(buffer_size);
}

NeSerialDriver::~NeSerialDriver() { Stop(); }

bool NeSerialDriver::IsOpen() const
{
  return is_open_.load(std::memory_order_acquire);
}

static int openPort(const std::string& port, BaudRate_e baud_rate);

void NeSerialDriver::Open(int interval_ms, int timeout_ms)
{
  for (auto& h : protocol_handlers_)
    h->StartReceive();
  // 接收线程：打开串口 + 读字节 → ring buffer，支持断线重连
  receive_thread_ = std::jthread([this, interval_ms, timeout_ms](
                                     std::stop_token st) {
    auto deadline = timeout_ms >= 0
                        ? std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms)
                        : std::chrono::steady_clock::time_point::max();

    // 尝试打开串口
    auto tryOpen = [&]() -> bool {
      while (!st.stop_requested())
      {
        int fd = openPort(port_, baud_rate_);
        if (fd >= 0)
        {
          // 成功打开，更新状态
          fd_.store(fd, std::memory_order_release);
          is_open_.store(true, std::memory_order_release);
          NV_INFO("Serial port {} opened successfully", port_);
          return true;
        }
        // 超时退出
        if (std::chrono::steady_clock::now() >= deadline)
        {
          NV_ERROR("Serial port {} open timed out", port_);
          return false;
        }

        NV_WARN("Serial port {} not available, retrying in {}ms...",
                port_,
                interval_ms);
        // 间隔一段时间后再试试
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
      }
      return false;
    };

    // 第一次打开
    if (!tryOpen())
      return;

    std::vector<uint8_t> buf(buffer_size_);
    while (!st.stop_requested())
    {
      // 获取当前fd
      int fd = fd_.load(std::memory_order_acquire);

      // 读取并全部写入缓冲区
      ssize_t n = ::read(fd, buf.data(), buf.size());
      if (n > 0)
      {
        for (ssize_t i = 0; i < n; ++i)
          receive_buffer_sPtr_->Push(buf[i]);
      }

      // 此时是串口断掉了
      else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
      {
        // 更新状态并且关闭
        is_open_.store(false, std::memory_order_release);
        ::close(fd);
        fd_.store(-1, std::memory_order_release);

        NV_WARN("Serial port {} disconnected, retrying every {}ms...",
                port_,
                interval_ms);

        // 重新尝试打开
        if (!tryOpen())
          return;
      }

      // 检查是否有协议超时，超时则重启串口
      else
      {
        for (auto& h : protocol_handlers_)
        {
          if (h->Istimeout())
          {
            NV_WARN(
                "Protocol cmd {} receive timeout, restarting serial port...",
                h->Cmd());
            is_open_.store(false, std::memory_order_release);
            ::close(fd_.load(std::memory_order_acquire));
            fd_.store(-1, std::memory_order_release);
            if (!tryOpen())
              return;
            break;
          }
        }
      }
    }

    // 这里是调用了停止函数了
    int fd = fd_.exchange(-1, std::memory_order_acq_rel);
    if (fd >= 0)
      ::close(fd);
    is_open_.store(false, std::memory_order_release);
  });

  // 解包线程：ring buffer → 帧同步 → CRC校验 → 分发 handler
  decode_thread_ = std::jthread([this](std::stop_token st) {
    // 协议格式: [hdr_hi][hdr_lo][cmd(1)][payload(N)][crc16_lo][crc16_hi]

    // 包头拆分，方便使用
    const uint8_t hdr_hi = (header_ >> 8) & 0xFF;
    const uint8_t hdr_lo = header_ & 0xFF;

    // 串口解包状态机
    enum class State_e
    {
      WAIT_H1,
      WAIT_H2,
      READ_CMD,
      READ_PAYLOAD,
      READ_CRC
    } state = State_e::WAIT_H1;

    uint8_t              current_cmd = 0;
    size_t               payload_size = 0;
    std::vector<uint8_t> frame; // [hdr_hi, hdr_lo, cmd, payload...]
    frame.reserve(256);

    while (!st.stop_requested())
    {
      // 开始读取一个字节

      uint8_t byte;
      if (!receive_buffer_sPtr_->Pop(byte))
      {
        // 此时没数据，先让出时间片
        std::this_thread::yield();
        continue;
      }

      switch (state)
      {
      case State_e::WAIT_H1:
        if (byte == hdr_hi)
        {
          // 持续寻找第一个包头字节
          frame.clear();
          frame.push_back(byte);
          state = State_e::WAIT_H2;
        }
        break;

      case State_e::WAIT_H2:
        if (byte == hdr_lo)
        {
          // 同理寻找第二个
          frame.push_back(byte);
          state = State_e::READ_CMD;
        }
        else
        {
          // 这里是第一个匹配上了第二个没匹配
          frame.clear();
          state = State_e::WAIT_H1;
        }
        break;

      case State_e::READ_CMD:
      {
        // 读取包CMD
        current_cmd = byte;
        payload_size = 0;
        // 找找看当前CMD有没有对应的协议
        for (auto& h : protocol_handlers_)
        {
          if (h->Cmd() == current_cmd)
          {
            // 如果有的话就获取这个协议体长度
            payload_size = h->Size();
            break;
          }
        }
        if (payload_size == 0)
        {
          // 没有这个CMD的协议，丢掉这个包头，等待下一个
          state = State_e::WAIT_H1;
          break;
        }
        // 读取CMD成功，将这个字节放入帧缓冲区并继续读
        frame.push_back(byte);
        state = State_e::READ_PAYLOAD;
        break;
      }

      case State_e::READ_PAYLOAD:
        frame.push_back(byte);
        // payload 前面有四个字节，所以索引从3+1开始
        // 如果读满了，就准备CRC
        if (frame.size() == 3 + payload_size)
          state = State_e::READ_CRC;
        break;

      case State_e::READ_CRC:
      {
        frame.push_back(byte);
        if (frame.size() == 3 + payload_size + 2)
        {
          // CRC16 校验整帧
          // 沿用之前的DJI CRC，连命名空间都没改
          if (ne_io::Verify_CRC16_Check_Sum(frame.data(), frame.size()))
          {
            // 看看对应的哪个协议，分发数据
            for (auto& h : protocol_handlers_)
            {
              if (h->Cmd() == current_cmd)
              {
                h->CopyFrame(frame.data() + 3, payload_size);
                h->Notify(); // 让协议自己的回调去处理数据
                // NV_DEBUG("Frame received: cmd=0x{:02X} size={}", current_cmd,
                //          payload_size);
                break;
              }
            }
          }
          // 重新开始
          state = State_e::WAIT_H1;
        }
        break;
      }
      }
    }
  });
}

void NeSerialDriver::Stop()
{
  for (auto& h : protocol_handlers_)
    h->StopReceive();
  receive_thread_.request_stop();
  decode_thread_.request_stop();
}

bool NeSerialDriver::Transmit(uint8_t* buffer, int write_size)
{
  if (!is_open_.load(std::memory_order_acquire))
    return false;
  int     fd  = fd_.load(std::memory_order_acquire);
  ssize_t ret = ::write(fd, buffer, write_size);
  return ret == write_size;
}

// Private methods
static int openPort(const std::string& port, BaudRate_e baud_rate)
{
  int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    return -1;

  termios tty{};
  if (tcgetattr(fd, &tty) != 0)
  {
    ::close(fd);
    return -1;
  }

  speed_t speed = B115200;
  switch (baud_rate)
  {
  case BaudRate_e::B_9600: speed = B9600; break;
  case BaudRate_e::B_57600: speed = B57600; break;
  case BaudRate_e::B_115200: speed = B115200; break;
  case BaudRate_e::B_230400: speed = B230400; break;
#ifndef __APPLE__
  case BaudRate_e::B_460800: speed = B460800; break;
  case BaudRate_e::B_921600: speed = B921600; break;
#else
  case BaudRate_e::B_460800:
  case BaudRate_e::B_921600: break; // 由 IOSSIOSPEED 处理，跳过标准设置
#endif
  default: speed = B115200; break;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  cfmakeraw(&tty);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    ::close(fd);
    return -1;
  }

#ifdef __APPLE__
  if (baud_rate == BaudRate_e::B_460800 || baud_rate == BaudRate_e::B_921600)
  {
    speed_t custom = static_cast<speed_t>(baud_rate);
    if (ioctl(fd, IOSSIOSPEED, &custom) < 0)
    {
      ::close(fd);
      return -1;
    }
  }
#endif
  return fd;
}

} // namespace drivers
} // namespace ne_vision
