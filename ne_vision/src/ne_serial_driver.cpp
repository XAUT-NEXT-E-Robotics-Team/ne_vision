#include "ne_vision/serial/ne_serial_driver.hpp"
#include <cstring>

namespace ne_serial
{

NeSerialDriver::NeSerialDriver(const std::string& port, int baudrate, bool is_sentry)
    : is_sentry_(is_sentry),
      config_(std::make_shared<SerialToNode::SerialConfig>(
          port, baudrate, 8, false, SerialToNode::Stopbit::TWO, SerialToNode::Parity::NONE)),
      port_(std::make_shared<SerialToNode::NePort>(config_))
{
    LOG_INFO("NeSerialDriver created: port=%s baudrate=%d sentry=%d",
             port.c_str(), baudrate, is_sentry);
}

NeSerialDriver::~NeSerialDriver()
{
    stop();
}

bool NeSerialDriver::start()
{
    // 打开串口
    int retry = 0;
    while (!port_->PortisOpen())
    {
        int fd = port_->openport();
        if (fd >= 0) break;
        retry++;
        if (retry > 5)
        {
            LOG_ERROR("Failed to open serial port after %d retries", retry);
            return false;
        }
        LOG_WARN("Open port failed, retry %d ...", retry);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    LOG_INFO("Serial port opened successfully (fd=%d)", port_->fd);

    running_ = true;

    // 启动接收线程
    receive_thread_ = std::thread(&NeSerialDriver::receiveLoop, this);

    // 启动自瞄状态定时器 (20Hz)
    auto_aim_timer_thread_ = std::thread(&NeSerialDriver::autoAimTimerLoop, this);

    // Sentry 模式启动导航定时器 (20Hz)
    if (is_sentry_)
    {
        nav_timer_thread_ = std::thread(&NeSerialDriver::navTimerLoop, this);
    }

    LOG_INFO("NeSerialDriver started");
    return true;
}

void NeSerialDriver::stop()
{
    if (!running_) return;
    running_ = false;

    if (receive_thread_.joinable()) receive_thread_.join();
    if (auto_aim_timer_thread_.joinable()) auto_aim_timer_thread_.join();
    if (nav_timer_thread_.joinable()) nav_timer_thread_.join();

    LOG_INFO("NeSerialDriver stopped. stats: read=%d write=%d pkgs=%d err_hdr=%d err_payload=%d",
             read_sum_, write_num_, pkg_sum_, error_sum_header_, error_sum_payload_);
}

// 注册回调
void NeSerialDriver::onGimbalState(GimbalStateCallback cb)
{
    gimbal_state_cb_ = std::move(cb);
}

void NeSerialDriver::onAutoAimStatus(AutoAimStatusCallback cb)
{
    auto_aim_status_cb_ = std::move(cb);
}

void NeSerialDriver::onLatency(LatencyCallback cb)
{
    latency_cb_ = std::move(cb);
}



void NeSerialDriver::sendGimbalOrientation(const GimbalOrientation& orient)
{
    sentry_protocol::Sentry_write pkt;
    pkt.header = sentry_protocol::PACK_HEADER;
    pkt.state  = orient.state;
    pkt.pitch  = orient.pitch;
    pkt.yaw    = orient.yaw;
    pkt.fire   = orient.fire;

    // 更新自瞄状态
    auto_aim_status_.state = orient.state;

    // 序列化: [struct bytes][CRC16 lo][CRC16 hi]
    constexpr int total_size = sizeof(pkt) + 2; // + 2 bytes CRC16
    uint8_t buf[total_size];
    std::memcpy(buf, &pkt, sizeof(pkt));
    ne_io::Append_CRC16_Check_Sum(buf, total_size);

    doTransmit(buf, total_size);
}

void NeSerialDriver::sendNavVelocity(float vx, float vy)
{
    nav_info_.receive_time_point = std::chrono::steady_clock::now();
    nav_info_.aim.velocity_x = vx;
    nav_info_.aim.velocity_y = vy;
}

void NeSerialDriver::sendNavCommand(float velocity, uint8_t mode)
{
    nav_info_.receive_time_point = std::chrono::steady_clock::now();
    nav_info_.aim.speed     = velocity;
    nav_info_.aim.move_mode = mode;
}



void NeSerialDriver::receiveLoop()
{
    while (running_.load())
    {
        int bytes = doReceive();
        if (bytes > 0)
        {
            // 追加到接收缓冲区
            for (int i = 0; i < bytes; i++)
            {
                receive_buffer_.push_back(receiveBuffer_[i]);
            }
            // 防止缓冲区溢出
            while (receive_buffer_.size() > MAX_BUFFER_SIZE)
            {
                receive_buffer_.pop_front();
            }
            // 尝试解码所有完整包
            while (receive_buffer_.size() >= 3) // 最小包: header + crc16(2)
            {
                classify();
                if (receive_buffer_.empty()) break;
                SerialToNode::PkgState state = decode();
                if (state == SerialToNode::PkgState::COMPLETE)
                {
                    pkg_sum_++;
                    continue; // 尝试解码下一个包
                }
                else if (state == SerialToNode::PkgState::PAYLOAD_INCOMPLETE)
                {
                    break; // 等待更多数据
                }
                else
                {
                    // CRC 错误或其他，跳过一个字节继续
                    if (!receive_buffer_.empty())
                        receive_buffer_.pop_front();
                }
            }
        }
        else if (bytes == 0)
        {
            // 无数据，短暂休眠避免忙等
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        else
        {
            // 读取错误，尝试重新打开
            LOG_ERROR("Serial read error, attempting reopen");
            port_->reopen();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

int NeSerialDriver::doReceive()
{
    if (!port_ || !port_->PortisOpen()) return -1;
    int n = port_->receive(receiveBuffer_);
    if (n > 0)
    {
        read_sum_ += n;
    }
    return n;
}

int NeSerialDriver::doTransmit(uint8_t *data, int size)
{
    std::lock_guard<std::mutex> lock(transmit_mutex_);
    if (!port_ || !port_->PortisOpen()) return -1;
    int written = port_->transmit(data, size);
    if (written > 0)
    {
        write_num_ += written;
        trans_pkg_sum_++;
    }
    return written;
}

void NeSerialDriver::classify()
{
    while (!receive_buffer_.empty())
    {
        uint8_t front = receive_buffer_.front();
        if (front == sentry_protocol::PACK_HEADER || front == sentry_protocol::NAV_PACK_HEADER)
        {
            break;
        }
        receive_buffer_.pop_front();
    }
    classify_pkg_sum_++;
}

SerialToNode::PkgState NeSerialDriver::decode()
{
    if (receive_buffer_.empty())
        return SerialToNode::PkgState::OTHER;

    uint8_t header = receive_buffer_.front();

    if (header == sentry_protocol::PACK_HEADER)
    {
        // Sentry_read 包: header(1) + pitch(4) + yaw(4) + color(1) + cmd(1) + muzzle_v(4) = 15 bytes
        // 加上 CRC16(2) = 17 bytes 总长
        constexpr size_t pkt_size = sizeof(sentry_protocol::Sentry_read);
        constexpr size_t total_size = pkt_size + 2; // + CRC16

        if (receive_buffer_.size() < total_size)
            return SerialToNode::PkgState::PAYLOAD_INCOMPLETE;

        // 复制到解码缓冲区
        for (size_t i = 0; i < total_size; i++)
        {
            decodeBuffer_[i] = receive_buffer_[i];
        }

        // 验证 CRC16
        if (!ne_io::Verify_CRC16_Check_Sum(decodeBuffer_, total_size))
        {
            error_sum_payload_++;
            // CRC 失败，丢弃第一个字节
            receive_buffer_.pop_front();
            return SerialToNode::PkgState::CRC_PKG_ERROR;
        }

        // 解码
        sentry_protocol::Sentry_read read_pkt =
            sentry_protocol::arraytostruct<sentry_protocol::Sentry_read>(decodeBuffer_);

        // 计算延迟
        auto now = std::chrono::high_resolution_clock::now();
        double latency_ms = std::chrono::duration<double, std::milli>(now - start_time_).count();
        start_time_ = now;

        // 调用回调
        if (gimbal_state_cb_)
        {
            GimbalState state;
            state.pitch     = read_pkt.pitch;
            state.yaw       = read_pkt.yaw;
            state.our_color = read_pkt.our_color;
            state.cmd       = read_pkt.cmd;
            state.muzzle_v  = read_pkt.muzzle_v;
            gimbal_state_cb_(state);
        }

        if (latency_cb_)
        {
            latency_cb_(latency_ms);
        }

        // 消费掉已解码的字节
        for (size_t i = 0; i < total_size; i++)
        {
            receive_buffer_.pop_front();
        }

        return SerialToNode::PkgState::COMPLETE;
    }
    else if (header == sentry_protocol::NAV_PACK_HEADER)
    {
        // 导航包: header(1) + v_x(4) + v_y(4) + move_mode(1) = 10 bytes + CRC16(2) = 12 bytes
        // 注: 导航包通常是写出去的，如果收到则跳过
        constexpr size_t pkt_size = sizeof(sentry_protocol::NavWrite);
        constexpr size_t total_size = pkt_size + 2;

        if (receive_buffer_.size() < total_size)
            return SerialToNode::PkgState::PAYLOAD_INCOMPLETE;

        // 跳过导航包（通常不会收到）
        for (size_t i = 0; i < total_size; i++)
        {
            receive_buffer_.pop_front();
        }
        return SerialToNode::PkgState::COMPLETE;
    }

    // 未知包头
    receive_buffer_.pop_front();
    return SerialToNode::PkgState::OTHER;
}

void NeSerialDriver::autoAimTimerLoop()
{
    while (running_.load())
    {
        auto next_tick = std::chrono::steady_clock::now() + std::chrono::milliseconds(50); // 20Hz

        if (auto_aim_status_cb_)
        {
            auto_aim_status_cb_(auto_aim_status_);
        }

        std::this_thread::sleep_until(next_tick);
    }
}

void NeSerialDriver::navTimerLoop()
{
    while (running_.load())
    {
        auto next_tick = std::chrono::steady_clock::now() + std::chrono::milliseconds(50); // 20Hz

        navTimerCallback();

        std::this_thread::sleep_until(next_tick);
    }
}

void NeSerialDriver::navTimerCallback()
{
    // 检查导航数据是否过期 (>500ms)
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - nav_info_.receive_time_point).count();
    if (elapsed > 500)
    {
        // 过期 - 发送零速度
        nav_info_.aim.velocity_x = 0;
        nav_info_.aim.velocity_y = 0;
    }

    sentry_protocol::NavWrite nav_pkt;
    nav_pkt.header    = sentry_protocol::NAV_PACK_HEADER;
    nav_pkt.v_x       = nav_info_.aim.velocity_x;
    nav_pkt.v_y       = nav_info_.aim.velocity_y;
    nav_pkt.move_mode = nav_info_.aim.move_mode;

    constexpr int total_size = sizeof(nav_pkt) + 2; // + CRC16
    uint8_t buf[total_size];
    std::memcpy(buf, &nav_pkt, sizeof(nav_pkt));
    ne_io::Append_CRC16_Check_Sum(buf, total_size);

    doTransmit(buf, total_size);
}

} // namespace ne_serial
