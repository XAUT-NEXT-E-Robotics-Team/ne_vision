#include "ne_vision/detector/openvino_infer.hpp"

namespace ne_vision
{
namespace infer
{

OpenvinoInfer::OpenvinoInfer(const string& model_path_xml,
                             const string& model_path_bin,
                             const string& device)
  : device_(device)
{
  input_shape = {1,
                 static_cast<unsigned long>(IMAGE_HEIGHT),
                 static_cast<unsigned long>(IMAGE_WIDTH),
                 3};
  model = core.read_model(model_path_xml, model_path_bin);
  ppp   = new ov::preprocess::PrePostProcessor(model);
  ppp->input()
      .tensor()
      .set_element_type(ov::element::u8)
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::BGR);
  ppp->input()
      .preprocess()
      .convert_element_type(ov::element::f32)
      .convert_color(ov::preprocess::ColorFormat::RGB)
      .scale({255., 255., 255.});
  ppp->input().model().set_layout("NCHW");
  ppp->output().tensor().set_element_type(ov::element::f32);
  model          = ppp->build();
  compiled_model = core.compile_model(model, device);
}

bool OpenvinoInfer::initModule(const std::string /*engine_or_plan_file*/,
                               int /*batch_size*/,
                               int /*num_classes*/)
{
  return compiled_model.operator bool();
}

std::vector<std::vector<Object>> OpenvinoInfer::dointerfence(
    std::vector<cv::Mat>& frames,
    float                 nms_threshold,
    float                 conf_threshold)
{
  std::vector<std::vector<Object>> batch_results;

  for (auto& img : frames)
  {
    objects.clear();
    tmp_objects.clear();

    cv::resize(img, img, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));

    ov::Tensor input_tensor(compiled_model.input().get_element_type(),
                            compiled_model.input().get_shape(),
                            img.data);
    auto infer_request = compiled_model.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    auto          output        = infer_request.get_output_tensor(0);
    ov::Shape     output_shape  = output.get_shape();
    const cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F,
                                (void*)output.data<float>());

    std::vector<cv::Rect> boxes;
    std::vector<float>    confidences;

    for (int i = 0; i < output_buffer.rows; i++)
    {
      float confidence = sigmoid(output_buffer.at<float>(i, 8));
      if (confidence < conf_threshold)
        continue;

      cv::Mat   color_scores   = output_buffer.row(i).colRange(9, 13);
      cv::Mat   classes_scores = output_buffer.row(i).colRange(13, 22);
      cv::Point class_id, color_id;
      double    score_num, score_color;
      cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
      cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);

      if (color_id.x == 3)
        continue;

      Object obj;
      obj.prob  = confidence;
      obj.color = color_id.x;
      obj.label = class_id.x;
      for (int k = 0; k < 8; ++k)
        obj.landmarks[k] = output_buffer.at<float>(i, k);

      obj.length = cv::norm(cv::Point2f(obj.landmarks[0] - obj.landmarks[6],
                                        obj.landmarks[1] - obj.landmarks[7]));
      obj.width  = cv::norm(cv::Point2f(obj.landmarks[0] - obj.landmarks[2],
                                        obj.landmarks[1] - obj.landmarks[3]));
      obj.ratio  = obj.length / obj.width;

      float min_x = obj.landmarks[0], max_x = obj.landmarks[0];
      float min_y = obj.landmarks[1], max_y = obj.landmarks[1];
      for (int k = 1; k < 4; ++k)
      {
        min_x = std::min(min_x, obj.landmarks[k * 2]);
        max_x = std::max(max_x, obj.landmarks[k * 2]);
        min_y = std::min(min_y, obj.landmarks[k * 2 + 1]);
        max_y = std::max(max_y, obj.landmarks[k * 2 + 1]);
      }
      obj.rect = cv::Rect_<float>(min_x, min_y, max_x - min_x, max_y - min_y);

      objects.push_back(obj);
      boxes.push_back(obj.rect);
      confidences.push_back(static_cast<float>(score_num));
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);
    for (int idx : indices)
      if (idx < static_cast<int>(objects.size()))
        tmp_objects.push_back(objects[idx]);

    batch_results.push_back(tmp_objects);
  }
  return batch_results;
}

} // namespace infer
} // namespace ne_vision
