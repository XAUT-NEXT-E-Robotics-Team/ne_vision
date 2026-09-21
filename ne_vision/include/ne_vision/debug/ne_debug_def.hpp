///////////////////////////////////////////////////////////
//                                                       //
//                        .                 .:-:         //
//                       :-:              :-::           //
//                      -----          .:---.            //
//                    .-------.     .:-----:             //
//                   :---------. .:-------.              //
//                  :--------------------.               //
//                ---------------------                  //
//               .-------:. :---------:                  //
//              :-----:.     .-------.                   //
//             .:---:         .-----.                    //
//            .:-:.             :-:                      //
//          .-:.                 .                       //
//         .:                                            //
//                                                       //
//    ███╗   ██╗███████╗██╗  ██╗████████╗    ███████╗    //
//    ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝    ██╔════╝    //
//    ██╔██╗ ██║█████╗   ╚███╔╝    ██║       █████╗      //
//    ██║╚██╗██║██╔══╝   ██╔██╗    ██║       ██╔══╝      //
//    ██║ ╚████║███████╗██╔╝ ██╗   ██║       ███████╗    //
//    ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝       ╚══════╝    //
//                                                       //
///////////////////////////////////////////////////////////
//                                                       //
// Copyright (c) 2026 XAUT NEXT-E. All Rights Reserved.  //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// Description:
//
// 为了统一debug，这里定义一些标准名称

#pragma once

// 缩写说明
// NVD = Ne Vision Debug
// P   = Path
// N   = Name

// 技术坐标系信息，根坐标为imu
#define NVD_IMU_N    "/imu"
#define NVD_IMU_P    "/imu"
#define NVD_GIMBAL_P "/imu/gimbal"
#define NVD_GIMBAL_N "gimbal"
// 相机使用合成后的 IMU -> camera TF，图像和内参必须位于该节点下。
#define NVD_CAMERA_P NVD_GIMBAL_P + "/camera"
#define NVD_CAMERA_N "camera"

// 相机图像信息与内参信息，这两者路径应该一样
// result 为识别标记，应该在frame路径下
#define NVD_CAMERA_FRAME_P         NVD_CAMERA_P + "/frame"
#define NVD_CAMERA_PARAM_P         NVD_CAMERA_FRAME_P
#define NVD_CAMERA_DETECT_RESULT_P NVD_CAMERA_FRAME_P + "/detect_result"

// 3D信息
#define NVD_DETECTED_ARMOR3D_P NVD_IMU_N + "/detected_armor3d"
