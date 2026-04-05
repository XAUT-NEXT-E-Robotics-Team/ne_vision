#ifndef NE_PUBLIC_H
#define NE_PUBLIC_H
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <numeric>
#include <chrono>
#include <vector>
#include <dirent.h>
#include <thread>

#include "NvInfer.h"
#include "cuda_runtime_api.h"
#include "ne_log_cu.h"
#include "ne_preprocess.h"
#include "cuda_runtime.h"
#include "NvOnnxParser.h"
using namespace nvonnxparser;
#define CHECK(status)                                          \
    do                                                         \
    {                                                          \
        auto ret = (status);                                   \
        if (ret != 0)                                          \
        {                                                      \
            std::cerr << "Cuda failure: " << ret << std::endl; \
            abort();                                           \
        }                                                      \
    } while (0)

#define MAX_IMAGE_INPUT_SIZE_THRESH 10000 * 10000
#define MAX_OUTPUT_BBOX_COUNT 1000

using namespace nvinfer1;
#endif