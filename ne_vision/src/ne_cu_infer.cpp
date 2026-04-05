#include "ne_vision/detector/ne_cu/include/ne_cu_infer.h"
#include "ne_vision/detector/infer_base.hpp"
#include <algorithm>
#include <cmath>
namespace ne_cu
{
    
    NeCudaInfer::NeCudaInfer(const int device)
    {
        CHECK(cudaSetDevice(device));
    }
    NeCudaInfer::~NeCudaInfer()
    {
        uninitmodel();
    }
    
    double NeCudaInfer::sigmoid(double x)
    {
    if (x > 0) { return 1.0 / (1.0 + exp(-x)); }
    else
    {
      return exp(x) / (1.0 + exp(x));
    }
    }

    double NeCudaInfer::cal_iou(const cv::Rect &r1,const cv::Rect &r2)
    {
    float x_left   = std::fmax(r1.x, r2.x);
    float y_top    = std::fmax(r1.y, r2.y);
    float x_right  = std::fmin(r1.x + r1.width, r2.x + r2.width);
    float y_bottom = std::fmin(r1.y + r1.height, r2.y + r2.height);

    if (x_right < x_left || y_bottom < y_top) { return 0.0; }

    double in_area = (x_right - x_left) * (y_bottom - y_top);
    double un_area = r1.area() + r2.area() - in_area;

    return in_area / un_area;
    }

    bool NeCudaInfer::initModule(const std::string engine_or_plan_file,const int batch_size,const int num_classes)
    {
        this -> batch_size = batch_size;
        this -> num_classes = num_classes;
        char *trtModelStream{nullptr};
        size_t size{0};
        std::ifstream file(engine_or_plan_file, std::ios::binary);
        if (file.good())
        {
            file.seekg(0, std::ios::end);
            size = file.tellg();
            file.seekg(0, std::ios::beg);
            trtModelStream = new char[size];
            assert(trtModelStream);
            file.read(trtModelStream, size);
            file.close();
        }
        else
        {
            this->ne_logger.log(ILogger::Severity::kERROR, "Engine bad file");
            return false;
        }
        this->run_time = createInferRuntime(this->ne_logger);
        assert(run_time != nullptr);
        //反序列化
        this->engine = this->run_time->deserializeCudaEngine(trtModelStream, size);
        assert(this->engine != nullptr);

        this->context = this->engine->createExecutionContext();
        assert(context != nullptr);

        delete trtModelStream;
        //显式batch
        this -> input_dim = this -> engine -> getTensorShape(kInputTensorName);//[1,3,640,640]
        this -> input_dim.d[0] = batch_size;

        this->output_dim = this->engine->getTensorShape(kOutputTensorName);
        this->context->setInputShape(kInputTensorName, input_dim);
        this -> output_size = output_dim.d[1] * output_dim.d[2];
        int IOtensorsNum = engine -> getNbIOTensors();
        assert(IOtensorsNum == 2);
        for(int i = 0; i < IOtensorsNum;i++)
        {
        if (!strcmp(this->engine->getIOTensorName(i), kInputTensorName))
        {
        this->inputIndex = i;
        assert(this->engine->getTensorDataType(kInputTensorName) == nvinfer1::DataType::kFLOAT);
        }
        else if (!strcmp(this->engine->getIOTensorName(i), kOutputTensorName))
        {
        this->outputIndex = i;
        assert(this->engine->getTensorDataType(kOutputTensorName) == nvinfer1::DataType::kFLOAT);
        }
        }
        if (this->inputIndex == -1 || this->outputIndex == -1)
        {
            this->ne_logger.log(ILogger::Severity::kERROR, "Uncorrect Input/Output tensor name");
            delete context;
            delete engine;
            return false;
        }
        CHECK(cudaMalloc(&buffer[inputIndex],batch_size * this -> input_dim.d[1] * this -> input_dim.d[2] * this -> input_dim.d[3] * sizeof(float)));
        CHECK(cudaMalloc(&buffer[outputIndex], batch_size * this->output_size * sizeof(float)));
        CHECK(cudaMallocHost((void **)&this->image_host, MAX_IMAGE_INPUT_SIZE_THRESH * 3 * sizeof(float)));
        CHECK(cudaMalloc((void **)&this->image_device, MAX_IMAGE_INPUT_SIZE_THRESH * 3 * sizeof(float)));
        this->output = (float *)malloc(batch_size * this->output_size * sizeof(float));
        this->_is_inited = true;
        return true;
    }

    void NeCudaInfer::uninitmodel()
    {
        this->_is_inited = false;
        delete this->context;
        delete this->run_time;
        delete this->engine;
        CHECK(cudaFree(image_device));
        CHECK(cudaFreeHost(image_host));
        CHECK(cudaFree(this->buffer[this->inputIndex]));
        CHECK(cudaFree(this->buffer[this->outputIndex]));
    }
    std::vector<std::vector<Object>> NeCudaInfer::dointerfence(std::vector<cv::Mat> &frames,float nms_thresold,float conf_thresold)
    {
        if(!this -> _is_inited)
        {
            this -> ne_logger.log(ILogger::Severity::kERROR,"Module not inited !");
            return {};
        }
        if(frames.size() == 0 || int(frames.size()) > this -> input_dim.d[0])
        {
            this->ne_logger.log(ILogger::Severity::kWARNING, "Invalid frames size");
            return {};
        }
    std::vector<std::vector<Object>>batch_res(frames.size());
        cudaStream_t stream = nullptr;
        CHECK(cudaStreamCreate(&stream));
        float *buffer_idx = (float*)buffer[this -> inputIndex];
        for (size_t b = 0; b < frames.size(); ++b)
        {
            cv::Mat &img = frames[b];
            if (img.empty())
                continue;
            size_t size_image = img.cols * img.rows * 3;
            size_t size_image_dst = this->input_dim.d[3] * this->input_dim.d[2] * 3;
            memcpy(image_host, img.data, size_image);
            CHECK(cudaMemcpyAsync(image_device, image_host, size_image, cudaMemcpyHostToDevice, stream));
            preprocess_kernel_img(image_device, img.cols, img.rows, buffer_idx, this->input_dim.d[3], this->input_dim.d[2], stream);
            buffer_idx += size_image_dst;
        }
        this->context->setOptimizationProfileAsync(0, stream);
        this -> context -> setTensorAddress(kInputTensorName, this->buffer[this->inputIndex]);
        this -> context -> setTensorAddress(kOutputTensorName, this->buffer[this->outputIndex]);

        bool success = this -> context -> enqueueV3(stream);
        if(!success)
        {
            this->ne_logger.log(ILogger::Severity::kERROR, "DoInference failed");
            CHECK(cudaStreamDestroy(stream));
            return {};
        }
        CHECK(cudaMemcpyAsync(this->output, buffer[this->outputIndex], frames.size() * this->output_size * sizeof(float), cudaMemcpyDeviceToHost, stream));
        CHECK(cudaStreamSynchronize(stream));
        CHECK(cudaStreamDestroy(stream));
        this -> postprocess(batch_res,frames,conf_thresold,nms_thresold);
        return batch_res;
    }
    void NeCudaInfer::postprocess(std::vector<std::vector<Object>> &batch_res,std::vector<cv::Mat> &frames,float &conf_threshold,float &nms_threshold)
    {
        for (int b = 0; b < int(frames.size()); ++b)
        {
            auto &res = batch_res[b];
            this->decoder(res, frames[b], &this->output[b * this->output_size],conf_threshold, nms_threshold);
        }
    }

    void NeCudaInfer::decoder(std::vector<Object> &res, cv::Mat &frame, float *pdata, float &conf_thresold, float &nms_thresold)
    {
        res.clear();
        std::vector<cv::Rect> boxes;
    std::vector<Object> tmp_objects;
        std::vector<int>      class_ids;
        std::vector<float>    class_scores;
        std::vector<float>    confidences;
        if (!frame.data || pdata == nullptr)
        {
            return;
        }
        if (this->output_dim.nbDims < 3 || this->output_dim.d[2] < 22)
        {
            return;
        }

        const int num_boxes = this->output_dim.d[1];
        const int elem = this->output_dim.d[2];
        cv::Mat output_buffer(num_boxes, elem, CV_32F, pdata);
        for(int i=0;i < output_buffer.rows;i++)
        {
            float confidence = output_buffer.at<float>(i,8);
            confidence = sigmoid(confidence);
            if(confidence < conf_thresold)
            {
                continue;
            }
        cv::Mat color_scores = output_buffer.row(i).colRange(9,13);
        cv::Mat class_scores = output_buffer.row(i).colRange(13,22);
        cv::Point class_id, color_id;
        int _class_id,_color_id;
        double score_color,score_num;
        cv::minMaxLoc(class_scores, NULL, &score_num,NULL,&class_id);//类别分数的最大值以及位置
        cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);//颜色分数的最大值以及其位置
        if(color_id.x ==2 || color_id.x == 3)//保留红蓝
        {
            continue;
        }
        _class_id = class_id.x;
        _color_id = color_id.x;
    Object obj;
        obj.prob = confidence;
        obj.color = _color_id;
        obj.label = _class_id;
        obj.landmarks[0] = output_buffer.at<float>(i, 0);
        obj.landmarks[1] = output_buffer.at<float>(i, 1);
        obj.landmarks[2] = output_buffer.at<float>(i, 2);
        obj.landmarks[3] = output_buffer.at<float>(i, 3);
        obj.landmarks[4] = output_buffer.at<float>(i, 4);
        obj.landmarks[5] = output_buffer.at<float>(i, 5);
        obj.landmarks[6] = output_buffer.at<float>(i, 6);
        obj.landmarks[7] = output_buffer.at<float>(i, 7);
        obj.length       = cv::norm(cv::Point2f(obj.landmarks[0] - obj.landmarks[6]) -
                          cv::Point2f(obj.landmarks[1] - obj.landmarks[7]));
        obj.width        = cv::norm(cv::Point2f(obj.landmarks[0] - obj.landmarks[2]) -
                         cv::Point2f(obj.landmarks[1] - obj.landmarks[3]));
        obj.ratio        = obj.length / obj.width;

        std::vector<cv::Point2f> points;
        points.push_back(cv::Point2f(obj.landmarks[0], obj.landmarks[1]));
        points.push_back(cv::Point2f(obj.landmarks[6], obj.landmarks[7]));
        points.push_back(cv::Point2f(obj.landmarks[4], obj.landmarks[5]));
        points.push_back(cv::Point2f(obj.landmarks[2], obj.landmarks[3]));

        float min_x = points[0].x;
        float max_x = points[0].x;
        float min_y = points[0].y;
        float max_y = points[0].y;

        for (size_t i = 1; i < points.size(); i++)
        {
        if (points[i].x < min_x) min_x = points[i].x;
        if (points[i].x > max_x) max_x = points[i].x;
        if (points[i].y < min_y) min_y = points[i].y;
        if (points[i].y > max_y) max_y = points[i].y;
        }
        cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);
        obj.rect = rect;
        res.push_back(obj);
        boxes.push_back(rect);
        confidences.push_back(score_num);
        }
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_thresold, nms_thresold, indices);
        // int index = 0, index_indices = 0;
        for (size_t valid_index : indices)
        {
        if (valid_index <= res.size()) { tmp_objects.push_back(res[valid_index]); }
        }
        return;
    }

  
    //别管，debug用的
    int NeCudaInfer::get_width()
    {
        return this -> input_dim.d[3];
    }

    int NeCudaInfer::get_height()
    {
        return this -> input_dim.d[2];
    }
}