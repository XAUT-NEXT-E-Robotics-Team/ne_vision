/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-03-22 20:40:06
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-03-22 20:57:07
 * @FilePath: /ne_vision/ne_vision/include/ne_vision/ne_hikcam.hpp
 * @Description: 我永远喜欢雪之下雪乃
 * 
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved. 
 */
#ifndef _HIK_CAM_H_
#define _HIK_CAM_H_


#include <cmath>
#include <opencv2/opencv.hpp>
#include <MvCameraControl.h>
#include <chrono>

typedef enum GAIN_MODE_{
    R_CHANNEL,
    G_CHANNEL,
    B_CHANNEL
} GAIN_MODE;


class HikCam
{
    int nRet = MV_OK;
    void* handle = nullptr;
    MVCC_FLOATVALUE FrameRate;
    MVCC_FLOATVALUE *pFrameRate = &FrameRate;
    MV_FRAME_OUT pFrame = {nullptr};
    int timestamp_offset = 0;

public:
    //相机库初始化
    HikCam();

    //打开设备
    int StartDevice(int serial_number);

    //开始采集
    bool SetStreamOn();

    //设置相机图像格式
    bool SetPixelFormat(int format);

    //设置图像分辨率
    bool SetResolution(int width = 1280, int height = 1024);

    //设置曝光时间
    bool SetExposureTime(float ExposureTime);

    //设置帧率
    bool SetFrameRate(float FrameRate);

    //读取实际帧率
    float GetFrameRate();

    //设置曝光增益
    bool SetGAIN(float ExpGain);

    //自动白平衡
    bool Set_Auto_BALANCE();

    //手动白平衡
    bool Set_BALANCE(int value, unsigned int value_number);

    //Gamma校正
    bool Set_Gamma(bool set_status,double dGammaParam);

    //色彩校正
    bool Color_Correct(bool value);

    //对比度调整
    bool Set_Contrast(bool set_status,int dContrastParam);

    //采集图像（原图像转rgb）
    void GetMat(cv::Mat &dst);

    //读取相机时间戳
    int Get_TIMESTAMP() const;

    ~HikCam();
};
#endif