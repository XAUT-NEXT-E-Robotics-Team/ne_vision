/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-03-22 23:33:40
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-03-22 23:33:58
 * @FilePath: /ne_vision/ne_serial/include/param.hpp
 * @Description: 我永远喜欢雪之下雪乃
 * 
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved. 
 */
#ifndef PARAM_HPP
#define PARAM_HPP
#include <cmath>
#include <cassert>
namespace ne_relative_param
{
    struct gimbal_camera_pose
    {
        float gimbal_to_camera_X     = 0;
        float gimbal_to_camera_Y     = 0;
        float gimbal_to_camera_Z     = 0;
        float gimbal_to_camera_roll  = 0;
        float gimbal_to_camera_pitch = 0;
        float gimbal_to_camera_yaw   = 0;
    };

    struct
    {
        struct
        {
            // 按照预计击打到装甲板上距离中心的距离进行火控的阈值 // 感谢中南大学FYT火控方案
            float Z_threshold     = 0.05; // 5cm
            float X_threshold     = 0.10; // 10cm

            // 为防止较远目标或特殊情况导致无法击打，当使用击打距中心判据计算角度小于下阈值时，将替换为下阈值进行火控判断
            float yaw_threshold   = 0.8; // 角度制
            float pitch_threshold = 0.4; // 角度制
        } general, outpost;

    } fire_judge_param_;

    struct
    {
        float yaw_feedforward_K = 0.0f;
        float pitch_feedforward_K = 0.0f;
        float yaw_dead_band = 0.0f;
        float pitch_dead_band = 0.0f;
    } control_feedforward_;

    struct
    {
        void ToRadian()
        {
            assert(std::abs(pitch_offset) < 30);
            assert(yaw_offset >= 0);
            pitch_offset = pitch_offset * static_cast<float>(M_PI) / 180.0f;
            yaw_offset   = yaw_offset   * static_cast<float>(M_PI) / 180.0f;
        }

        float pitch_offset = 0.0f; // 角度制
        float yaw_offset   = 0.0f; // 角度制
    } gimbal_to_camera_offset_;

}
#endif
