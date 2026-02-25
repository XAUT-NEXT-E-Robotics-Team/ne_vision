/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-02-25 16:15:07
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-02-25 16:24:51
 * @FilePath: /ne_BallisticModel/example/main.cpp
 * @Description: 我永远喜欢雪之下雪乃
 * 
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved. 
 */

#include "ballistic_slove.hpp"

#include <chrono>
#include <iostream>
int main()
{
    // Initialize global state
    Com_ptr_->ball_type = "small";
    Com_ptr_->muzzle_v = 25.0f; // muzzle speed (units consistent with model)

    YUKINO::BallisticModel model;
    model.judgeK1();

    float x = 7.0f;
    float OssneHeight = 0.3f;
    float x_offset = 0.05f;
    float Ossne_offset = 0.05f;
    using std::chrono::duration_cast;
    using std::chrono::microseconds;
    using std::chrono::system_clock;

    auto start = system_clock::now();
    float pitch = model.Cal_TargetposPitch(x, OssneHeight, x_offset, Ossne_offset);
    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    std::cout << "pitch(rad) = " << pitch << std::endl;
    std::cout << "time(s) = "
              << double(duration.count()) * microseconds::period::num / microseconds::period::den
              << std::endl;

    return 0;
}
