#ifndef NE_SERIAL_DRIVER_HPP
#define NE_SERIAL_DRIVER_HPP

#define READER_BUFFER_SIZE 64   // do not change this
#define MAX_BUFFER_SIZE 2048
#define DECODE_BUFFER_SIZE 128
#define TRANSMIT_BUFFER_SIZE 128

#include "protocol_bridge.hpp"
#include "crc.hpp"
#include "param.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#define LOG_INFO(fmt, ...)  printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  printf("[WARN] " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)

namespace ne_serial
{


struct GimbalState
{
    float pitch = 0.0f;
    float yaw = 0.0f;
    int8_t our_color = 0;
    uint8_t cmd = 0;
    float muzzle_v = 0.0f;
};

struct AutoAimStatus
{
    uint8_t state = 0; // tracking ,state,color的使能位
};

struct GimbalOrientation
{
    float pitch = 0.0f;
    float yaw = 0.0f;
    uint8_t fire = 0x00;
    uint8_t state = 0x00;
};

struct Latency
{
    double latency_ms = 0.0;
};

// 回调类型定义
using GimbalStateCallback   = std::function<void(const GimbalState&)>;//这是云台状态
using AutoAimStatusCallback = std::function<void(const AutoAimStatus&)>;//这是自瞄的回调
using LatencyCallback       = std::function<void(double latency_ms)>;//这是延时的分析

class NeSerialDriver
{
public:
    NeSerialDriver(const std::string& port, int baudrate = 200000, bool is_sentry = true);
    ~NeSerialDriver();

    // 生命周期
    bool start();  // 打开串口，启动接收线程和定时器线程
    void stop();   // 停止所有线程

    // 注册回调
    void onGimbalState(GimbalStateCallback cb);
    void onAutoAimStatus(AutoAimStatusCallback cb);
    void onLatency(LatencyCallback cb);

    // 发送数据
    void sendGimbalOrientation(const GimbalOrientation& orient);
    //导航的数据
    void sendNavVelocity(float vx, float vy);
    void sendNavCommand(float velocity, uint8_t mode);

private:
    // 线程
    std::thread receive_thread_;//接收的线程
    std::thread auto_aim_timer_thread_;//自瞄的线程
    std::thread nav_timer_thread_;//导航定时器的线程
    std::atomic<bool> running_{false};
    bool is_sentry_;

    // 回调
    GimbalStateCallback   gimbal_state_cb_;
    AutoAimStatusCallback auto_aim_status_cb_;
    LatencyCallback       latency_cb_;

    // 串口
    std::shared_ptr<SerialToNode::SerialConfig> config_;
    std::shared_ptr<SerialToNode::NePort> port_;

    // 协议状态
    std::deque<uint8_t> receive_buffer_;//fifo
    std::mutex transmit_mutex_;
    AutoAimStatus auto_aim_status_;

    uint8_t decodeBuffer_[DECODE_BUFFER_SIZE];
    uint8_t receiveBuffer_[READER_BUFFER_SIZE];

    std::chrono::high_resolution_clock::time_point start_time_ =
        std::chrono::high_resolution_clock::now();

    // Sentry 导航信息
    sentry_protocol::NavInfo nav_info_;

    // 统计
    int error_sum_payload_ = 0;
    int error_sum_header_ = 0;
    int read_sum_ = 0;
    int write_num_ = 0;
    int pkg_sum_ = 0;
    int classify_pkg_sum_ = 0;
    int trans_pkg_sum_ = 0;

    // 内部方法
    void receiveLoop();
    int doReceive();
    int doTransmit(uint8_t *data, int size);
    void classify();
    SerialToNode::PkgState decode();
    void autoAimTimerLoop();
    void navTimerLoop();
    void navTimerCallback();
};

} // namespace ne_serial
#endif