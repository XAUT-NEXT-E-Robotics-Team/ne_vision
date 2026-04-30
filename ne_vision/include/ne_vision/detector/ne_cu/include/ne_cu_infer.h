#ifndef NE_INFER_H
#define NE_INFER_H
#include "./ne_log_cu.h"
#include "./ne_public.h"
#include "ne_vision/detector/infer_base.hpp"
namespace ne_cu
{
    // Use the common Object defined in the InferBase to keep interface consistent
    using Object = ne_vision::infer::Object;
    class NeCudaInfer: public ne_vision::infer::InferBase
    {
        public:
        NeCudaInfer() = default;
        NeCudaInfer(const int device);
        ~NeCudaInfer();
        bool initModule(const std::string engine_or_plan_file,const int batch_size,const int num_classes);
        void uninitmodel();
        std::vector<std::vector<Object>> dointerfence(std::vector<cv::Mat> &frames,float nms_thresold,float conf_thresold);
        
        //拿netron看看
        inline static constexpr const char* kInputTensorName  = "images";
        inline static constexpr const char* kOutputTensorName = "output";

        int num_classes = 22;
        int batch_size = 1;
        const static int kInputH = 640;
        const static int kInputW = 640;
        private:
        TRTLogger ne_logger;
        IRuntime *run_time;
        ICudaEngine *engine;
        IExecutionContext *context;
        void *buffer[2];
        Dims input_dim;
        Dims output_dim;
        uint8_t *image_host = nullptr;
        uint8_t *image_device = nullptr;
        float *output;
        int inter_frame_compensation = 0;
        bool _is_inited = false;
        int inputIndex = -1;
        int outputIndex = -1;
        int output_size = -1;
        private:
    //void nms(std::vector<Object> &input_boxes, float &nms_threshold);
    void postprocess(std::vector<std::vector<Object>> &batch_res,
                 std::vector<cv::Mat> &frames,
                 float &conf_threshold,
                 float &nms_threshold);
    void decoder(std::vector<Object> &res, cv::Mat &frame, float *pdata, float &conf_thresold, float &nms_thresold);
        double sigmoid(double x);
        double cal_iou(const cv::Rect &r1,const cv::Rect &r2);
        int get_width();
        int get_height();
        
    };
}
#endif