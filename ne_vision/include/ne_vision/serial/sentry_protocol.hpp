#ifndef SENTRY_PROTOCOL_HPP
#define SENTRY_PROTOCOL_HPP
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <vector>
#include <map>
#include <variant>
#include <iostream>
namespace sentry_protocol
{
    enum RobotMoveMode_e
    {
        /**
         * 说明：
         * 1. 该模式下，车完全站桩
         * 2. 该模式下，底盘速度跟随最大速度方向，用于过洞
         * 其他模式字如其名
         */
        ROBOT_STOP                  = 0, // 停止
        ROBOT_CHASSIS_TOUGHT_PATH   = 1, // 底盘跟随路径模式模式
        ROBOT_CHASSIS_TOUGHT_GIMBAL = 2, // 底盘跟随云台模式
        ROBOT_SLOW_CONSTANT_TOP     = 3, // 底盘慢速匀速陀螺模式
        ROBOT_FAST_CONSTANT_TOP     = 4, // 底盘快速匀速陀螺模式
        ROBOT_VARIABLE_TOP          = 5, // 底盘变速陀螺模式
    };

    struct NavInfo
    {
        std::chrono::steady_clock::time_point receive_time_point;
        struct
        {
            float velocity_x  = 0.0f;
            float velocity_y  = 0.0f;
            float omega_z     = 0.0f;
            uint8_t move_mode = 0;
            float speed       = 0.0f;
        } aim;
    };

    constexpr uint8_t PACK_HEADER = 0xA5;
    constexpr uint8_t NAV_PACK_HEADER = 0xF5;
    struct Sentry_write
    {
        uint8_t header;
        uint8_t state;
        float pitch;
        float yaw;
        uint8_t fire;
    } __attribute__ ((packed));

    struct Sentry_read
    {
      uint8_t header;
      float pitch;
      float yaw;
      int8_t our_color;
      uint8_t cmd;
      float muzzle_v;
    } __attribute__ ((packed));

    struct NavWrite
    {
        uint8_t header;
        float v_x;
        float v_y;
        uint8_t move_mode;
    } __attribute__ ((packed));


    #define SET_AUTO_AIM_STATE_ENABLE(__X__)       (__X__ |= 0x01)    // 0000 0001
    #define SET_AUTO_AIM_STATE_DISABLE(__X__)      (__X__ &= 0xFE)    // 1111 1110
    #define SET_AUTO_AIM_STATE_WE_ARE_RED(__X__)   (__X__ |= 0x02)    // 0000 0010
    #define SET_AUTO_AIM_STATE_WE_ARE_BLUE(__X__)  (__X__ &= 0xFD)    // 1111 1101
    #define SET_AUTO_AIM_STATE_TRACKING(__X__)     (__X__ |= 0x04)    // 0000 0100
    #define SET_AUTO_AIM_STATE_NO_TRACKING(__X__)  (__X__ &= 0xFB)    // 1111 1011

    enum AutoaimFireStatus
    {
        AUTO_AIM_FIRE = 0xff,
        AUTO_AIM_NO_FIRE = 0x00,
    };
    using Sentry_protocol = std::variant<Sentry_read,Sentry_write,NavWrite>;
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
