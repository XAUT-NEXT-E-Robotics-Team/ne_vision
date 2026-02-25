/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-02-24 17:57:36
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-02-25 16:12:31
 * @FilePath: /ne_BallisticModel/include/model_param.hpp
 * @Description: 我永远喜欢雪之下雪乃
 * 
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved. 
 */
#ifndef BALLISTIC_SLOVE_HPP
#define BALLISTIC_SLOVE_HPP
#define PI 3.14159265358
#define GRAVITY 9.8
#define BIG_BALL_K1 0.00556//阻力系数/质量
#define BIG_BALL_LIGHTED_K1 0.00530
#define SMALL_BALL_K1 0.01903
#define MAX_ITERATION_COUNT 30 //最大迭代次数
#define ITERATE_SCALE_FACTOR 0.9
#define PRECISION 0.000001
#include <iostream>
#include <math.h>
#include <string>
//单方向的模型
struct Com_ps 
{
    std::string ball_type = "small";
    float K1;//K0/m
    float muzzle_v;//v0
    float ft;//飞行时长
}; 

extern Com_ps* Com_ptr_;//全局变量
#endif
