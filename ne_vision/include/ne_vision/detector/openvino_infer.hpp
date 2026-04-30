#pragma once

#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>

#include "ne_vision/detector/infer_base.hpp"

namespace ne_vision
{
namespace infer
{

using namespace cv;
using namespace std;

class OpenvinoInfer : public InferBase
{
public:
  OpenvinoInfer() = default;
  OpenvinoInfer(const string& model_path_xml,
                const string& model_path_bin,
                const string& device);

  bool initModule(const std::string engine_or_plan_file,
                  int               batch_size,
                  int               num_classes) override;

  std::vector<std::vector<Object>> dointerfence(std::vector<cv::Mat>& frames,
                                                float                 nms_threshold,
                                                float                 conf_threshold) override;

private:
  const int IMAGE_HEIGHT = 640;
  const int IMAGE_WIDTH  = 640;

  double              ans = 0;
  std::vector<double> ious;
  std::vector<Object> objects;

  std::shared_ptr<ov::Model>        model;
  ov::Core                          core;
  ov::preprocess::PrePostProcessor* ppp = nullptr;
  ov::CompiledModel                 compiled_model;
  ov::Shape                         input_shape;
  std::string                       device_;

  double sigmoid(double x)
  {
    return x > 0 ? 1.0 / (1.0 + exp(-x)) : exp(x) / (1.0 + exp(x));
  }
};

} // namespace infer
} // namespace ne_vision
