#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <memory>
#include <atomic>
#include <chrono>

#include "rerun.hpp"
#include "opencv2/opencv.hpp"

#include "ne_vision/utils/ne_log.hpp"

// 配置宏
#define NV_RERUN_DEFAULT_APP_NAME "ne_vision_debug"
#define NV_RERUN_SAVE_INTERVAL_S  5 // 每 5 秒自动保存一次文件

namespace ne_vision
{

class NeRerunDebug final
{
public:
  ~NeRerunDebug() = default;

  // 单例防拷贝
  const NeRerunDebug& operator=(const NeRerunDebug&) = delete;
  NeRerunDebug&       operator=(NeRerunDebug&&) = delete;
  NeRerunDebug(const NeRerunDebug&) = delete;
  NeRerunDebug(NeRerunDebug&&) = delete;

  static NeRerunDebug& GetInstance()
  {
    static NeRerunDebug instance;
    return instance;
  }

  // --- 核心控制接口 ---

  /**
   * @brief 开启实时调试（连接 Rerun Viewer）
   * @param tcp_url Viewer 的地址，如 "127.0.0.1:9876"
   */
  void EnableRealtimeDebug(const std::string& tcp_url = "NULL")
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (is_connected_.load())
      return;

    EnsureRecInitialized();

    rerun::Error error;
    if (tcp_url != "NULL")
      error = rec_->connect_grpc(tcp_url.c_str());
    else
      error = rec_->connect_grpc();
    if (error.is_ok())
    {
      is_connected_.store(true);
      NV_INFO("Rerun Realtime Debug Enabled: Connected to {}", tcp_url);
    }
    else
    {
      NV_WARN("Rerun Realtime Debug Failed: Could not connect to {}", tcp_url);
    }
  }

  /**
   * @brief 开启日志文件记录
   * @param path .rrd 文件路径
   */
  void EnableLogToFile(const std::string& path = "debug.rrd")
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (is_file_enabled_.load())
      return;

    EnsureRecInitialized();
    save_path_ = path;
    is_file_enabled_.store(true);

    // 立即执行一次初始保存
    rec_->save(save_path_.c_str()).exit_on_failure();
    last_save_stamp_ = std::chrono::steady_clock::now();

    NV_INFO("Rerun File Logging Enabled: Saving to {}", path);
  }

  // --- 日志接口 ---

  template <typename... Args>
  void Log(const std::string& entity_path, const Args&... args)
  {
    // 如果两个开关都没开，直接返回
    if (!is_connected_.load() && !is_file_enabled_.load())
      return;

    std::lock_guard<std::mutex> lock(mtx_);
    if (!rec_)
      return;

    // 执行发送
    rec_->log(entity_path.c_str(), args...);

    // 如果开启了文件记录，检查是否需要更新文件
    if (is_file_enabled_.load())
    {
      HandleAutoSave();
    }
  }

  void LogFrame(const std::string& entity_path, const cv::Mat& frame)
  {
    if (!is_connected_.load() && !is_file_enabled_.load())
      return;

    if (frame.empty())
    {
      NV_WARN("Attempted to log an empty frame to {}", entity_path);
      return;
    }

    rerun::Image rr_img;
    // 根据通道处理图片
    if (frame.channels() == 3)
    {
      cv::Mat rgb_frame;
      cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);
      rr_img = rerun::Image::from_rgb24(
          std::vector<uint8_t>(rgb_frame.data,
                               rgb_frame.data + rgb_frame.total() * 3),
          rerun::WidthHeight(static_cast<uint32_t>(rgb_frame.cols),
                             static_cast<uint32_t>(rgb_frame.rows)));
    }
    else if (frame.channels() == 1)
    {
      rr_img = rerun::Image::from_grayscale8(
          std::vector<uint8_t>(frame.data, frame.data + frame.total()),
          rerun::WidthHeight(static_cast<uint32_t>(frame.cols),
                             static_cast<uint32_t>(frame.rows)));
    }
    else
      return;

    Log(entity_path, rr_img);
  }

private:
  NeRerunDebug() = default;

  // 确保 RecordingStream 存在
  void EnsureRecInitialized()
  {
    if (!rec_)
    {
      rec_.emplace(NV_RERUN_DEFAULT_APP_NAME);
    }
  }

  // 处理定时更新文件
  void HandleAutoSave()
  {
    auto now = std::chrono::steady_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::seconds>(now - last_save_stamp_)
            .count();

    if (elapsed >= NV_RERUN_SAVE_INTERVAL_S)
    {
      // Rerun 的 save 是增量的，重复调用会更新文件内容
      rec_->save(save_path_.c_str()).exit_on_failure();
      last_save_stamp_ = now;
      // 注意：NV_TRACE 可能太频繁，这里不打印日志
    }
  }

  std::mutex                            mtx_;
  std::optional<rerun::RecordingStream> rec_;

  std::atomic<bool> is_connected_{false};    // 实时连接状态
  std::atomic<bool> is_file_enabled_{false}; // 文件保存状态

  std::string                           save_path_;
  std::chrono::steady_clock::time_point last_save_stamp_;
};

// 宏封装
#define NV_REC_ENABLE_REALTIME(url)                                            \
  ne_vision::NeRerunDebug::GetInstance().EnableRealtimeDebug(url)
#define NV_REC_ENABLE_FILE(path)                                               \
  ne_vision::NeRerunDebug::GetInstance().EnableLogToFile(path)
#define NV_REC_LOG(...) ne_vision::NeRerunDebug::GetInstance().Log(__VA_ARGS__)
#define NV_REC_LOG_FRAME(path, img)                                            \
  ne_vision::NeRerunDebug::GetInstance().LogFrame(path, img)

} // namespace ne_vision
