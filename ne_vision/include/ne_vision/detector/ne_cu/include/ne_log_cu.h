#ifndef NE_LOG_CU_H
#define NE_LOG_CU_H
#include "./ne_public.h"
class TRTLogger : public nvinfer1::ILogger
{
public:
    explicit TRTLogger(nvinfer1::ILogger::Severity severity = nvinfer1::ILogger::Severity::kWARNING) : severity_(severity) {}
    void log(nvinfer1::ILogger::Severity severity, const char *msg) noexcept override
    {
        if (severity <= severity_)
        {
            std::cerr << msg << std::endl;
        }
    }
    nvinfer1::ILogger::Severity severity_;
};
#endif