/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-04-01 15:38:01
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-04-03 19:17:13
 * @FilePath: /ne_vision/ne_vision/include/ne_vision/detector/infer_base.hpp
 * @Description: 我永远喜欢雪之下雪乃
 * 
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved. 
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
//推理的基类，以同时适配于openvino和tensorrt
namespace ne_vision
{
namespace infer
{

struct Object
{
  cv::Rect_<float> rect;
  float            landmarks[8];
  int              label;
  float            prob;
  int              color;
  double           length;
  double           width;
  double           ratio;
};

class InferBase
{
public:
  virtual ~InferBase() = default;
  // virtual void infer(cv::Mat img, int detect_color) = 0;
  virtual bool initModule(const std::string engine_or_plan_file,const int batch_size,const int num_classes) = 0;
  // 返回每一帧的检测结果，使用基类内定义的 Object 类型以统一不同后端
  virtual std::vector<std::vector<Object>> dointerfence(std::vector<cv::Mat> &frames,float nms_thresold,float conf_thresold) = 0;
  std::vector<Object> tmp_objects;
  
};

} // namespace infer
} // namespace ne_vision
