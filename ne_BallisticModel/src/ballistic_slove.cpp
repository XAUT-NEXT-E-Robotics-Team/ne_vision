/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-03-22 20:26:57
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-03-22 20:30:18
 * @FilePath: /ne_vision/ne_BallisticModel/src/ballistic_slove.cpp
 * @Description: 我永远喜欢雪之下雪乃
 * 
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved. 
 */
#include "ballistic_slove.hpp"
#include <cstdio>

Com_ps g_com_ps;
Com_ps* Com_ptr_ = &g_com_ps;

namespace YUKINO
{
    BallisticModel::BallisticModel()
    {

    }
    BallisticModel::~BallisticModel()
    {

    }
    Com_ps* BallisticModel::judgeK1()
    {
        if(Com_ptr_ -> ball_type =="small")
        {
            Com_ptr_ -> K1 = SMALL_BALL_K1;
        }
        else
        {
            Com_ptr_ -> K1 = BIG_BALL_K1;
        }
        std::cout<<Com_ptr_ -> K1<<std::endl;
        return Com_ptr_;
    }
    float BallisticModel::Cal_OssneHeight(float x,float cal_pitch)
    {
        Com_ptr_ -> ft = (float)(exp(x * Com_ptr_ -> K1) - 1) / (Com_ptr_ -> K1 * cos(cal_pitch) * Com_ptr_ -> muzzle_v);
        //std::cout<<Com_ptr_ -> ft << std::endl;
        float OssneHeight = (float)(sin(cal_pitch)* Com_ptr_ -> muzzle_v * Com_ptr_ -> ft) - 0.5 * GRAVITY * pow(Com_ptr_ -> ft,2);
        //std::cout<<OssneHeight <<std::endl; 
        return OssneHeight;
    }
    float BallisticModel::get_ft(float x,float cal_pitch)
    {
        Com_ptr_ -> ft = (float)(exp(x * Com_ptr_ -> K1) - 1) / (Com_ptr_ -> K1 * cos(cal_pitch) * Com_ptr_ -> muzzle_v);
        return ft;
    }
    float BallisticModel::Cal_TargetposPitch(float x,float OssneHeight,float x_offset,float Ossne_offset)
    {
        //局部变量
        int count = 0;
        float aim_ossne = OssneHeight;
        float cal_pitch = 0;//迭代的初始角度为零
        float Drop_OssneHeight = 0;
        float Actual_error = 0;
        
        for(int i=0;i<MAX_ITERATION_COUNT;i++)
        {
            cal_pitch = atan2(aim_ossne,x);//计算角度
            //考虑偏移
            Drop_OssneHeight = Cal_OssneHeight(x - (cos(cal_pitch) * x_offset - sin(cal_pitch) * Ossne_offset),cal_pitch);//根据模型计算落点高度
            Actual_error = OssneHeight - Drop_OssneHeight;//更新误差
            aim_ossne = aim_ossne + Actual_error * ITERATE_SCALE_FACTOR;//误差补偿
            count++;
            if(fabs(Actual_error) < PRECISION)
            {
                break;
            }
            printf("x = %f,原始pitch = %f,pitch = %f,迭代次数 = %d\n",x,-atan2(OssneHeight,x) * 180 / 3.14,-(cal_pitch * 180 / 3.14),count);
        }
        return -cal_pitch;
    }
}
