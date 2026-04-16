
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
// 使用opencv，拓展的绘图工具
// 全AI写的 一个字没动

#pragma once

#include "opencv2/opencv.hpp"
#include <chrono>
#include <deque>
#include <limits>
#include <vector>
#include <string>
#include <algorithm>

namespace ne_vision
{

class NeDataScope
{
public:
  NeDataScope() : title_("untitle"), width_(100), height_(50), time_window_(5)
  {
  }

  NeDataScope(const std::string& title,
              int                width,
              int                height,
              double             time_window_sec = 5.0)
      : title_(title), width_(width), height_(height),
        time_window_(time_window_sec)
  {
  }

  void SetPosition(int x, int y)
  {
    x_ = x;
    y_ = y;
  }

  void SetSize(int width, int height)
  {
    width_ = width;
    height_ = height;
  }

  void SetTimeWindow(double time_window_sec) { time_window_ = time_window_sec; }

  /**
   * @brief 添加数据点
   * @param time      时间戳
   * @param value     数值
   * @param series_idx 曲线索引（0-based），不同索引对应不同颜色的曲线
   */
  void AddPoint(std::chrono::steady_clock::time_point time,
                double                                value,
                size_t                                series_idx = 0)
  {
    if (series_idx >= series_.size())
      series_.resize(series_idx + 1);
    series_[series_idx].data.push_back({time, value});
  }

  void Draw(cv::Mat& frame)
  {
    if (series_.empty() || frame.empty())
      return;

    // 找到所有系列中最新的时间戳
    std::chrono::steady_clock::time_point latest_time;
    bool                                  has_data = false;
    for (auto& s : series_)
    {
      if (!s.data.empty())
      {
        if (!has_data || s.data.back().time > latest_time)
          latest_time = s.data.back().time;
        has_data = true;
      }
    }
    if (!has_data)
      return;

    // 清理超出时间窗口的数据，并计算全局 min/max
    double min_val   = std::numeric_limits<double>::max();
    double max_val   = std::numeric_limits<double>::lowest();
    double sum       = 0.0;
    size_t total     = 0;
    double current_val = 0.0;

    for (auto& s : series_)
    {
      while (!s.data.empty())
      {
        double dt =
            std::chrono::duration<double>(latest_time - s.data.front().time)
                .count();
        if (dt > time_window_)
          s.data.pop_front();
        else
          break;
      }
      for (const auto& pt : s.data)
      {
        min_val = std::min(min_val, pt.value);
        max_val = std::max(max_val, pt.value);
        sum += pt.value;
        ++total;
      }
      if (!s.data.empty())
        current_val = s.data.back().value;
    }

    if (total == 0)
      return;

    double avg_val = sum / total;

    // 防止除数为0，设置合理的显示范围余量
    double margin = (max_val - min_val) * 0.1;
    if (margin == 0)
      margin = 1.0;
    double y_min   = min_val - margin;
    double y_max   = max_val + margin;
    double y_range = y_max - y_min;

    // 绘制背景
    cv::Rect roi(x_, y_, width_, height_);
    roi &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (roi.width <= 0 || roi.height <= 0)
      return;

    cv::Mat graph_roi = frame(roi);
    cv::Mat overlay;
    graph_roi.copyTo(overlay);
    cv::rectangle(overlay,
                  cv::Rect(0, 0, roi.width, roi.height),
                  cv::Scalar(60, 20, 20),
                  -1);
    cv::addWeighted(overlay, 0.4, graph_roi, 0.6, 0, graph_roi);

    cv::rectangle(frame, roi, cv::Scalar(200, 200, 200), 1);

    std::string info = cv::format(
        "%s | Cur: %.2f | Avg: %.2f", title_.c_str(), current_val, avg_val);
    cv::putText(frame,
                info,
                cv::Point(x_ + 5, y_ + 12),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(0, 255, 255),
                1);

    double x_interval = time_window_ / 4.0;

    int graph_top    = y_ + 18;
    int graph_bottom = y_ + height_ - 15;
    int graph_h      = graph_bottom - graph_top;
    if (graph_h <= 0)
      graph_h = 10;

    int num_lines = 4;
    for (int i = 0; i <= num_lines; ++i)
    {
      int y_pos = graph_top + i * graph_h / num_lines;

      if (i > 0 && i < num_lines)
      {
        cv::line(frame,
                 cv::Point(x_, y_pos),
                 cv::Point(x_ + width_, y_pos),
                 cv::Scalar(100, 100, 100),
                 1,
                 cv::LINE_AA);
      }

      double y_val  = y_max - i * (y_range / num_lines);
      int    text_y = (i == 0) ? (y_pos + 11) : (y_pos - 2);
      cv::putText(frame,
                  cv::format("%.2f", y_val),
                  cv::Point(x_ + 2, text_y),
                  cv::FONT_HERSHEY_PLAIN,
                  0.65,
                  cv::Scalar(255, 200, 200),
                  1);
    }

    for (int i = 0; i <= num_lines; ++i)
    {
      int    x_pos = x_ + width_ - i * width_ / num_lines;
      double x_val = i * x_interval;
      cv::putText(frame,
                  cv::format("-%.1fs", x_val),
                  cv::Point(x_pos - 35, y_ + height_ - 3),
                  cv::FONT_HERSHEY_PLAIN,
                  0.65,
                  cv::Scalar(255, 200, 200),
                  1);
    }

    // 每条曲线用不同颜色绘制
    static const cv::Scalar SERIES_COLORS[] = {
        cv::Scalar(0, 255, 0),    // 绿
        cv::Scalar(0, 100, 255),  // 橙
        cv::Scalar(255, 50, 50),  // 蓝
        cv::Scalar(255, 0, 255),  // 品红
        cv::Scalar(0, 255, 255),  // 黄
        cv::Scalar(255, 255, 0),  // 青
    };
    constexpr size_t NUM_COLORS = sizeof(SERIES_COLORS) / sizeof(SERIES_COLORS[0]);

    for (size_t si = 0; si < series_.size(); ++si)
    {
      const auto& s         = series_[si];
      size_t      data_size = s.data.size();
      if (data_size < 2)
        continue;

      std::vector<cv::Point> pts(data_size);

#pragma omp parallel for if (data_size > 100)
      for (size_t i = 0; i < data_size; ++i)
      {
        const auto& pt = s.data[i];

        double t_offset =
            std::chrono::duration<double>(latest_time - pt.time).count();
        int px =
            x_ + width_ - static_cast<int>((t_offset / time_window_) * width_);
        int py = graph_bottom -
                 static_cast<int>((pt.value - y_min) / y_range * graph_h);

        px = std::max(x_, std::min(px, x_ + width_));
        py = std::max(graph_top, std::min(py, graph_bottom));

        pts[i] = cv::Point(px, py);
      }

      const cv::Point* pt_ptr = pts.data();
      int              npt    = static_cast<int>(pts.size());
      cv::polylines(frame,
                    &pt_ptr,
                    &npt,
                    1,
                    false,
                    SERIES_COLORS[si % NUM_COLORS],
                    1,
                    cv::LINE_AA);
    }
  }

  const std::string& GetTitle() const { return title_; }
  int                GetWidth() const { return width_; }
  int                GetHeight() const { return height_; }

private:
  struct DataPoint_t
  {
    std::chrono::steady_clock::time_point time;
    double                                value;
  };

  struct Series_t
  {
    std::deque<DataPoint_t> data;
  };

  std::string          title_;
  int                  x_           = 0;
  int                  y_           = 0;
  int                  width_       = 300;
  int                  height_      = 150;
  double               time_window_ = 5.0;
  std::vector<Series_t> series_;
};

class NeDataScopeManager
{
public:
  NeDataScopeManager() = default;

  NeDataScope& GetOrCreateScope(const std::string& title,
                                int                width          = 260,
                                int                height         = 120,
                                double             time_window_sec = 5.0)
  {
    for (auto& scope : scopes_)
    {
      if (scope.GetTitle() == title)
        return scope;
    }
    scopes_.emplace_back(title, width, height, time_window_sec);
    return scopes_.back();
  }

  /**
   * @brief 快捷添加数据点
   * @param series_idx 曲线索引，同一 title 下不同 series_idx 绘制不同曲线
   */
  void AddPoint(const std::string&                    title,
                std::chrono::steady_clock::time_point time,
                double                                value,
                size_t                                series_idx = 0)
  {
    GetOrCreateScope(title).AddPoint(time, value, series_idx);
  }

  void AddText(const std::string& key, const std::string& value)
  {
    for (auto& item : texts_)
    {
      if (item.first == key)
      {
        item.second = value;
        return;
      }
    }
    texts_.emplace_back(key, value);
  }

  void AddText(const std::string& key, double value, const char* fmt = "%.3f")
  {
    AddText(key, cv::format(fmt, value));
  }

  void AddText(const std::string& key, int value)
  {
    AddText(key, std::to_string(value));
  }

  void DrawAll(cv::Mat& frame)
  {
    if (frame.empty())
      return;

    if (!texts_.empty())
    {
      int start_x    = 10;
      int start_y    = 15;
      int line_height = 20;

      cv::Mat overlay;
      frame.copyTo(overlay);

      for (const auto& item : texts_)
      {
        std::string display_text = item.first + ": " + item.second;

        int      baseline  = 0;
        cv::Size text_size = cv::getTextSize(
            display_text, cv::FONT_HERSHEY_PLAIN, 0.8, 1, &baseline);

        cv::Rect text_bg(start_x - 3,
                         start_y - text_size.height - 3,
                         text_size.width + 6,
                         text_size.height + baseline + 3);
        text_bg &= cv::Rect(0, 0, overlay.cols, overlay.rows);
        if (text_bg.area() > 0)
          cv::rectangle(overlay, text_bg, cv::Scalar(0, 0, 0), -1);

        start_y += line_height;
        if (start_y > frame.rows - 10)
        {
          start_y = 15;
          start_x += 160;
        }
      }

      cv::addWeighted(overlay, 0.4, frame, 0.6, 0, frame);

      start_x = 10;
      start_y = 15;
      for (const auto& item : texts_)
      {
        std::string display_text = item.first + ": " + item.second;
        cv::putText(frame,
                    display_text,
                    cv::Point(start_x, start_y),
                    cv::FONT_HERSHEY_PLAIN,
                    0.8,
                    cv::Scalar(0, 255, 255),
                    1,
                    cv::LINE_AA);

        start_y += line_height;
        if (start_y > frame.rows - 10)
        {
          start_y = 15;
          start_x += 160;
        }
      }
    }

    if (scopes_.empty())
      return;

    int padding_x = 15;
    int padding_y = 15;

    int start_x = frame.cols - padding_x;
    int start_y = padding_y;

    int current_x    = start_x;
    int current_y    = start_y;
    int max_w_in_col = 0;

    for (auto& scope : scopes_)
    {
      int w = scope.GetWidth();
      int h = scope.GetHeight();

      if (current_y + h > frame.rows - padding_y && current_y != start_y)
      {
        current_x -= (max_w_in_col + padding_x);
        current_y    = start_y;
        max_w_in_col = 0;
      }

      if (current_x - w < 0)
        break;

      scope.SetPosition(current_x - w, current_y);
      scope.Draw(frame);

      current_y += (h + padding_y);
      if (w > max_w_in_col)
        max_w_in_col = w;
    }
  }

private:
  std::vector<NeDataScope>                         scopes_;
  std::vector<std::pair<std::string, std::string>> texts_;
};

} // namespace ne_vision
