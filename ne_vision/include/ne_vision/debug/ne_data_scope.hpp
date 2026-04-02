
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

  void AddPoint(std::chrono::steady_clock::time_point time, double value)
  {
    data_.push_back({time, value});
  }

  void Draw(cv::Mat& frame)
  {
    if (data_.empty() || frame.empty())
    {
      return;
    }

    auto latest_time = data_.back().time;

    // 清理超出时间窗口的数据
    while (!data_.empty())
    {
      double dt =
          std::chrono::duration<double>(latest_time - data_.front().time)
              .count();
      if (dt > time_window_)
      {
        data_.pop_front();
      }
      else
      {
        break;
      }
    }

    if (data_.empty())
      return;

    // 计算最大值、最小值、当前值、平均值
    double min_val = data_.front().value;
    double max_val = data_.front().value;
    double sum = 0.0;

    for (const auto& pt : data_)
    {
      if (pt.value < min_val)
        min_val = pt.value;
      if (pt.value > max_val)
        max_val = pt.value;
      sum += pt.value;
    }

    double current_val = data_.back().value;
    double avg_val = sum / data_.size();

    // 防止除数为0，设置合理的显示范围余量
    double margin = (max_val - min_val) * 0.1;
    if (margin == 0)
      margin = 1.0;
    double y_min = min_val - margin;
    double y_max = max_val + margin;
    double y_range = y_max - y_min;

    // 绘制背景
    cv::Rect roi(x_, y_, width_, height_);
    // 保证ROI不超出图像边界
    roi &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (roi.width <= 0 || roi.height <= 0)
      return;

    cv::Mat graph_roi = frame(roi);
    cv::Mat overlay;
    graph_roi.copyTo(overlay);
    cv::rectangle(overlay,
                  cv::Rect(0, 0, roi.width, roi.height),
                  cv::Scalar(60, 20, 20), // 使用较明显的深蓝色/暗紫色系背景
                  -1);
    // 调整为更高的透明度，使背景清晰：融合比例改为 0.4(overlay) :
    // 0.6(graph_roi)
    cv::addWeighted(overlay, 0.4, graph_roi, 0.6, 0, graph_roi);

    // 绘制外边框，更为显眼
    cv::rectangle(frame, roi, cv::Scalar(200, 200, 200), 1);

    // 绘制标题和信息，去掉最大最小值展示
    std::string info = cv::format(
        "%s | Cur: %.2f | Avg: %.2f", title_.c_str(), current_val, avg_val);
    cv::putText(frame,
                info,
                cv::Point(x_ + 5, y_ + 12), // 放置在绘图区最上方
                cv::FONT_HERSHEY_PLAIN,     // 使用 Plain 字体以显得更细更紧凑
                0.8,
                cv::Scalar(0, 255, 255),
                1);

    // 绘制横纵坐标刻度预留参数
    double x_interval = time_window_ / 4.0;

    // 定义图形实际绘制区域（预留上方18像素和下方15像素给文本区域，防止折线或文本覆盖）
    int graph_top = y_ + 18;
    int graph_bottom = y_ + height_ - 15;
    int graph_h = graph_bottom - graph_top;
    if (graph_h <= 0)
      graph_h = 10; // 防止高度过小引发除零异常

    // 绘制网格辅助线（简单画几条横线）和纵轴坐标值
    int num_lines = 4;
    for (int i = 0; i <= num_lines; ++i)
    {
      int y_pos = graph_top + i * graph_h / num_lines;

      // 避免上下边界线与外框重叠时显得不自然，可以不画顶底边网格
      if (i > 0 && i < num_lines)
      {
        cv::line(frame,
                 cv::Point(x_, y_pos),
                 cv::Point(x_ + width_, y_pos),
                 cv::Scalar(100, 100, 100),
                 1,
                 cv::LINE_AA);
      }

      // 绘制 y 轴数值
      double y_val = y_max - i * (y_range / num_lines);
      // y轴文本避免与标题粘连，i=0 时画在线下，其余画在线上
      int text_y = (i == 0) ? (y_pos + 11) : (y_pos - 2);

      cv::putText(frame,
                  cv::format("%.2f", y_val),
                  cv::Point(x_ + 2, text_y),
                  cv::FONT_HERSHEY_PLAIN,
                  0.65, // 字体再略缩一点
                  cv::Scalar(255, 200, 200),
                  1);
    }

    // 绘制横轴坐标值（时间，从当前时刻回推）
    for (int i = 0; i <= num_lines; ++i)
    {
      int    x_pos = x_ + width_ - i * width_ / num_lines;
      double x_val = i * x_interval; // 距离当前时间多久 (s)
      cv::putText(frame,
                  cv::format("-%.1fs", x_val),
                  cv::Point(x_pos - 35, y_ + height_ - 3), // 底部最下沿
                  cv::FONT_HERSHEY_PLAIN,
                  0.65,
                  cv::Scalar(255, 200, 200),
                  1);
    }

    // 绘制数据对应的折线
    size_t data_size = data_.size();
    if (data_size > 1)
    {
      std::vector<cv::Point> pts(data_size);

// 使用 OpenMP
// 优化坐标映射的过程，虽然数据量小但可以在数据频率极高或分辨率大时减少延时
#pragma omp parallel for if (data_size > 100)
      for (size_t i = 0; i < data_size; ++i)
      {
        const auto& pt = data_[i];

        // 将时间映射到 [x_, x_ + width_]
        double t_offset =
            std::chrono::duration<double>(latest_time - pt.time).count();
        int px =
            x_ + width_ - static_cast<int>((t_offset / time_window_) * width_);

        // 将值映射到 [graph_bottom, graph_top]
        int py = graph_bottom -
                 static_cast<int>((pt.value - y_min) / y_range * graph_h);

        // 限制在实际显示的范围内
        px = std::max(x_, std::min(px, x_ + width_));
        py = std::max(graph_top, std::min(py, graph_bottom));

        pts[i] = cv::Point(px, py);
      }

      // 使用polylines绘制，改用更显眼的亮绿色表示曲线
      const cv::Point* pt_ptr = pts.data();
      int              npt = static_cast<int>(pts.size());
      cv::polylines(frame,
                    &pt_ptr,
                    &npt,
                    1,
                    false,
                    cv::Scalar(0, 255, 0),
                    1,
                    cv::LINE_AA);
    }
  }

  const std::string& GetTitle() const { return title_; }
  int                GetWidth() const { return width_; }
  int                GetHeight() const { return height_; }

private:
  struct DataPoint
  {
    std::chrono::steady_clock::time_point time;
    double                                value;
  };

  std::string           title_;
  int                   x_ = 0;
  int                   y_ = 0;
  int                   width_ = 300;
  int                   height_ = 150;
  double                time_window_ = 5.0; // 显示的时间窗口（秒）
  std::deque<DataPoint> data_;
};

class NeDataScopeManager
{
public:
  NeDataScopeManager() = default;

  // 注册或获取一个示波器
  NeDataScope& GetOrCreateScope(const std::string& title,
                                int                width = 260,
                                int                height = 120,
                                double             time_window_sec = 5.0)
  {
    for (auto& scope : scopes_)
    {
      if (scope.GetTitle() == title)
      {
        return scope;
      }
    }
    scopes_.emplace_back(title, width, height, time_window_sec);
    return scopes_.back();
  }

  // 快捷添加数据点
  void AddPoint(const std::string&                    title,
                std::chrono::steady_clock::time_point time,
                double                                value)
  {
    GetOrCreateScope(title).AddPoint(time, value);
  }

  // 添加仅供显示的文本或不带图形的数据（将在左上角排列）
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

  // 统一绘制，按从右上角向下、向左的方式自动排列波形图，并在左上角绘制文本
  void DrawAll(cv::Mat& frame)
  {
    if (frame.empty())
      return;

    // 1. 绘制纯文本信息（左上角）
    if (!texts_.empty())
    {
      int start_x = 10;
      int start_y = 15;
      int line_height = 20; // 调小行间距

      // 预先计算所有文本块所在的大致区域来进行整体半透明融合（可选），
      // 为了简单，对每一行采用单行独立半透明背景覆盖
      cv::Mat overlay;
      frame.copyTo(overlay);

      for (const auto& item : texts_)
      {
        std::string display_text = item.first + ": " + item.second;

        // 计算当前文字大小
        int      baseline = 0;
        cv::Size text_size = cv::getTextSize(
            display_text, cv::FONT_HERSHEY_PLAIN, 0.8, 1, &baseline);

        // 绘制当前文本行的黑色半透明背景框
        cv::Rect text_bg(start_x - 3,
                         start_y - text_size.height - 3,
                         text_size.width + 6,
                         text_size.height + baseline + 3);
        // 区域防止超出图片边框
        text_bg &= cv::Rect(0, 0, overlay.cols, overlay.rows);
        if (text_bg.area() > 0)
        {
          cv::rectangle(overlay, text_bg, cv::Scalar(0, 0, 0), -1);
        }

        start_y += line_height;
        // 自动折列（如果高度超出了画面底部）
        if (start_y > frame.rows - 10)
        {
          start_y = 15;
          start_x += 160;
        }
      }

      // 应用半透明
      cv::addWeighted(overlay, 0.4, frame, 0.6, 0, frame);

      // 背景画好后重新在 frame 上刷上文字（去除黑边，纯黄色字）
      start_x = 10;
      start_y = 15;
      for (const auto& item : texts_)
      {
        std::string display_text = item.first + ": " + item.second;
        cv::putText(frame,
                    display_text,
                    cv::Point(start_x, start_y),
                    cv::FONT_HERSHEY_PLAIN,
                    0.8, // 缩小字体且换作更细致明快的 PLAIN
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

    // 初始位置：图像右上角
    int start_x = frame.cols - padding_x;
    int start_y = padding_y;

    int current_x = start_x;
    int current_y = start_y;
    int max_w_in_col = 0;

    for (auto& scope : scopes_)
    {
      int w = scope.GetWidth();
      int h = scope.GetHeight();

      // 如果当前列排不下这个波形窗口（且不是列首），就换到左边一列
      if (current_y + h > frame.rows - padding_y && current_y != start_y)
      {
        current_x -= (max_w_in_col + padding_x);
        current_y = start_y;
        max_w_in_col = 0;
      }

      // 排列时如果当前窗口还是超出了左侧边界则不画了（超出屏幕范围）
      if (current_x - w < 0)
      {
        break;
      }

      scope.SetPosition(current_x - w, current_y);
      scope.Draw(frame);

      current_y += (h + padding_y);
      if (w > max_w_in_col)
      {
        max_w_in_col = w;
      }
    }
  }

private:
  std::vector<NeDataScope>                         scopes_;
  std::vector<std::pair<std::string, std::string>> texts_;
};

} // namespace ne_vision