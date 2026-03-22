/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-03-22 23:34:45
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-03-22 23:34:55
 * @FilePath: /ne_vision/ne_serial/include/protocol.hpp
 * @Description: 我永远喜欢雪之下雪乃
 * 
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved. 
 */
#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>
#include <map>
#include <variant>
#include <iostream>
namespace general_protocol
{
    constexpr uint8_t PACK_HEADER = 0xA5;
    struct Protocol_write
    {
        uint8_t header;
        uint8_t state;//自瞄运行状态
        float pitch;
        float yaw;
        uint8_t fire;

    } __attribute__ ((packed));

    struct Protocol_read
    {
        uint8_t header;
        int8_t our_color;
        float pitch;
        float yaw;
        uint8_t cmd;
        float muzzle_v;
    } __attribute__ ((packed));
    #define SET_AUTO_AIM_STATE_ENABLE(__X__)       (__X__ |= 0x01)    // 0000 0001 使能
    #define SET_AUTO_AIM_STATE_DISABLE(__X__)      (__X__ &= 0xFE)    // 1111 1110 
    #define SET_AUTO_AIM_STATE_WE_ARE_RED(__X__)   (__X__ |= 0x02)    // 0000 0010
    #define SET_AUTO_AIM_STATE_WE_ARE_BLUE(__X__)  (__X__ &= 0xFD)    // 1111 1101
    #define SET_AUTO_AIM_STATE_TRACKING(__X__)     (__X__ |= 0x04)    // 0000 0100
    #define SET_AUTO_AIM_STATE_NO_TRACKING(__X__)  (__X__ &= 0xFB)    // 1111 1011

    // 说人话的CMD
    #define CMD_IS_WATER_MODE(__CMD__) (__CMD__ & 0x01) // 0000 0001 这个是设置泼水模式
    enum AutoaimFireStatus
    {
        AUTO_AIM_FIRE = 0xff,
        AUTO_AIM_NO_FIRE = 0x00,
    };

    using ProtocoType = std::variant<Protocol_read,Protocol_write>;
    template <typename T>
    inline T arraytostruct(uint8_t *buffer)
    {
        T result;
        std::memcpy(&result,buffer,sizeof(T));
        return result;
    }
    template <typename T>
    inline void structToarray(const T& inputstruct, uint8_t* outputArray)
    {
        std::memcpy(outputArray, reinterpret_cast<const uint8_t*>(&inputstruct), sizeof(T));
    }
    template <typename T>
    inline T fromVector(const std::vector<uint8_t>&data)
    {
        T received_packet;
        std::copy(data.begin(),data.end(),reinterpret_cast<uint8_t *>(&received_packet));
        return received_packet;
    }
    template <typename T>
    inline std::vector<uint8_t> tovector(const T& data)
    {
        std::vector<uint8_t> send_packet(sizeof(T));
        std::copy(
         reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
        send_packet.begin());
        return send_packet;
    }
}
#endif
