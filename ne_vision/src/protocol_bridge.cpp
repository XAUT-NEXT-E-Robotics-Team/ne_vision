///////////////////////////////////////////////////////////
//                                                       //
//                        .                .:-:          //
//                        :-:              :-::          //
//                      -----          .:---.            //
//                    .-------.     .:-----:             //
//                   :---------. .:-------.              //
//                  :--------------------.               //
//                 ---------------------                 //
//                .-------:. :---------:                 //
//               :-----:.     .-------.                  //
//              .:---:         .-----.                   //
//            .:-:.              :-:                     //
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
// 串口协议，NJQ写的
//
// 跨平台，想不到吧

#include "ne_vision/serial/protocol_bridge.hpp"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/ioctl.h>

#ifdef __APPLE__
#include <termios.h>
#include <IOKit/serial/ioss.h> // macOS ：用于设置自定义波特率
#else
#include <termios.h> // 先包含标准库
// 解决 Linux 下内核头文件与标准库头文件的冲突
#define termios asmtermios
#define winsize asmwinsize
#define termio  asmtermio
#include <asm/termios.h>
#undef termios
#undef winsize
#undef termio
#endif

// macOS 不支持 CMSPAR(Mark/Space Parity)，这里做宏兼容兜底
#ifndef CMSPAR
#define CMSPAR 0
#endif

namespace SerialToNode
{

NePort::NePort(std::shared_ptr<SerialConfig> SerialConfig_ptr)
{
  config = SerialConfig_ptr;
}

bool NePort::init()
{
  struct termios newtio;
  struct termios oldtio;
  bzero(&newtio, sizeof(newtio));
  bzero(&oldtio, sizeof(oldtio));
  if (tcgetattr(fd, &oldtio) != 0)
  {
    perror("tcgetattr");
    return false;
  }
  newtio.c_cflag |= CLOCAL | CREAD;
  switch (config->databit)
  {
  case 5: newtio.c_cflag |= CS5; break;
  case 6: newtio.c_cflag |= CS6; break;
  case 7: newtio.c_cflag |= CS7; break;
  case 8: newtio.c_cflag |= CS8; break;
  default: fprintf(stderr, "unsupported data size\n"); return false;
  }

  switch (config->parity)
  {
  case Parity::NONE:
    newtio.c_cflag &= ~PARENB;
    newtio.c_iflag &= ~INPCK;
    break;
  case Parity::ODD:
    newtio.c_cflag |= (PARODD | PARENB);
    newtio.c_iflag |= INPCK;
    break;
  case Parity::EVEN:
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    newtio.c_iflag |= INPCK;
    break;
  case Parity::MARK:
    newtio.c_cflag |= PARENB;
    newtio.c_cflag |= CMSPAR;
    newtio.c_cflag |= PARODD;
    newtio.c_iflag |= INPCK;
    break;
  case Parity::SPACE:
    newtio.c_cflag |= PARENB;
    newtio.c_cflag |= CMSPAR;
    newtio.c_cflag &= ~PARODD;
    newtio.c_iflag |= INPCK;
    break;
  default: fprintf(stderr, "unsupported parity\n"); return false;
  }

  switch (config->stopbit)
  {
  case Stopbit::ONE: newtio.c_cflag &= ~CSTOPB; break;
  case Stopbit::TWO: newtio.c_cflag |= CSTOPB; break;
  default: perror("unsupported stop bits\n"); return false;
  }

  if (config->flowcontrol)
    newtio.c_cflag |= CRTSCTS;
  else
    newtio.c_cflag &= ~CRTSCTS;

  newtio.c_cc[VTIME] = 10; // Time-out value (tenths of a second) 1s
  newtio.c_cc[VMIN] = 0;   // Minimum number of bytes read at once
  tcflush(fd, TCIOFLUSH);

  if (tcsetattr(fd, TCSANOW, &newtio) != 0)
  {
    perror("tcsetattr");
    return false;
  }

  // === 跨平台处理自定义波特率 ===
#ifdef __APPLE__
  // macOS 平台设置波特率
  speed_t speed = config->baundrate;
  if (ioctl(fd, IOSSIOSPEED, &speed) == -1)
  {
    printf(
        "[WARN] IOSSIOSPEED failed (virtual port?), using default baud rate\n");
  }
#else
  // Linux 平台设置波特率
  struct termios2 tio;
  if (ioctl(fd, TCGETS2, &tio) == 0)
  {
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER; // 使用自定义波特率
    tio.c_ispeed = config->baundrate;
    tio.c_ospeed = config->baundrate;
    if (ioctl(fd, TCSETS2, &tio) != 0)
    {
      printf(
          "[WARN] TCSETS2 failed (virtual port?), using default baud rate\n");
    }
  }
  else
  {
    printf("[WARN] TCGETS2 not supported (virtual port?), skipping custom baud "
           "rate\n");
  }
#endif

  isinit = true;
  return true;
}

int NePort::openport()
{
  fd = open(config->portname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0)
  {
    perror("open device failed");
    isopen = false;
    return fd;
  }
  // 获取并修改文件状态标志
  flags = fcntl(fd, F_GETFL, 0);
  flags &= ~O_NONBLOCK;
  if (fcntl(fd, F_SETFL, flags) < 0)
  {
    printf("[WARN] fcntl F_SETFL failed.\n");
  }

  // 虚拟串口(pts)不是 tty 设备，isatty 会返回 0，这里仅做提示不阻断
  if (isatty(fd) == 0)
  {
    printf("[INFO] Device is not a tty (virtual serial port?), continuing.\n");
  }
  else
  {
    printf("[INFO] tty device test ok.\n");
  }

  isopen = true;
  init();
  return fd;
}

int NePort::transmit(uint8_t* buffer, int write_size)
{
  num_per_write = write(fd, buffer, write_size);
  if (num_per_write > 0)
  {
    return num_per_write;
  }
  else
  {
    return -1;
  }
}

int NePort::receive(uint8_t* buffer)
{
  num_per_read = read(fd, buffer, 64);
  return num_per_read;
}

bool NePort::closePort()
{
  isopen = false;
  return close(fd) == 0;
}

bool NePort::reopen()
{
  if (PortisOpen())
    closePort();
  if (openport() >= 0)
    return true;
  return false;
}

bool NePort::PortisInit() { return isinit; }

bool NePort::PortisOpen() { return isopen; }

NePort::~NePort()
{
  if (isopen)
    closePort();
}

SerialConfig::~SerialConfig() {}

} // namespace SerialToNode
