/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-03-22 23:34:29
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-03-22 23:34:32
 * @FilePath: /ne_vision/ne_serial/include/protocol_bridge.hpp
 * @Description: 我永远喜欢雪之下雪乃
 * 
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved. 
 */
#ifndef PROTOCOL_BRIDGE_HPP
#define PROTOCOL_BRIDGE_HPP

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#define Sentry

#ifndef Sentry
#include "protocol.hpp"
#endif

#ifdef Sentry
#include "sentry_protocol.hpp"
#endif

namespace SerialToNode
{
enum PkgState : uint8_t
{
    COMPLETE = 0,          //可编译的，即完整的
    HEADER_INCOMPLETE,     //包头有问题
    PAYLOAD_INCOMPLETE,    //
    CRC_HEADER_ERROR,
    CRC_PKG_ERROR,
    OTHER,
};
enum Stopbit : uint8_t
{
    ONE = 0,
    ONE_POINT_FIVE,
    TWO,
};
enum Parity : uint8_t
{
    NONE = 0,
    ODD,
    EVEN,
    MARK,
    SPACE,
};
class SerialConfig
{
public:
    SerialConfig() = default;
    SerialConfig(const std::string& port, int baundrate, int databit, bool flow, Stopbit stopbit, Parity parity)
        : portname(port), baundrate(baundrate), databit(databit), flowcontrol(flow), stopbit(stopbit), parity(parity)
    {
    }
    ~SerialConfig();
    std::string portname = "/dev/ttyACM0";
    int baundrate = 200000;
    int databit = 8;
    bool flowcontrol = false;
    Stopbit stopbit = Stopbit::ONE;
    Parity parity = Parity::NONE;
};
class NePort
{
public:
    NePort(std::shared_ptr<SerialConfig> SerialConfig_ptr);
    ~NePort();
    int openport();
    bool closePort();
    bool init();
    bool reopen();
    bool PortisInit();
    bool PortisOpen();
    int fd;
    int transmit(uint8_t *buffer, int write_size);
    int receive(uint8_t *buffer);

private:
    std::shared_ptr<SerialConfig> config;
    int flags = 0;
    int num_per_read = 0;
    int num_per_write = 0;
    bool isinit = false;
    bool isopen = false;
    PkgState frameState;
};
} // namespace SerialToNode
#endif
