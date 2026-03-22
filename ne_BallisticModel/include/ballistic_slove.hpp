/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-03-22 20:26:57
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-03-22 20:28:48
 * @FilePath: /ne_vision/ne_BallisticModel/include/ballistic_slove.hpp
 * @Description: 我永远喜欢雪之下雪乃
 * 
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved. 
 */
#ifndef MODEL_HPP
#define MODEL_HPP
#include "model_param.hpp"
#include <cmath>
#include <memory.h>
#include <functional>
namespace YUKINO
{
class BallisticModel
{
    public:
    BallisticModel();
    ~BallisticModel();
    float Cal_OssneHeight(float x,float cal_pitch);//x 为击打距离，即为aim_distance
    Com_ps* judgeK1();
    float get_ft(float x,float cal_pitch);
    //需要注意的一点是弹丸并不是从云台直接的发射出去的，需要加上从云台到枪口的偏移（x方向为水平方向，Ossne为垂直方向）
    float Cal_TargetposPitch(float x,float OssneHeight,float x_offset,float Ossne_offset);//计算pitch
};
}
#endif
