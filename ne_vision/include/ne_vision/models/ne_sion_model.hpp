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
// ABAB 你不可能通过名字知道这是什么模型 ABAB
//
// 感谢 Sion 和 Erica 保佑模型正常
//
// 反小陀螺整车模型，文档见DOCS
//
// 注意：请不要直接为该模型创建任务，其实你也创建不了。
//      请将其放在tracker 3d运行，哪里可能还有更多模型再运行。
//
// Tip: 由SO2特殊的交换性质，因此广义加法等同于相加并重映射到PI
//
// 一些名称和符号说明
// state 或 x： 名义状态
// error_state 或 ex： 误差状态

#pragma once

#include "ne_vision/interfaces/ne_armors_3d.hpp"
#include "ne_vision/interfaces/ne_imu_data.hpp"
#include "ne_vision/utils/ne_math.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace ne_vision
{

namespace sion
{

// 定义一些常量
// constexpr 好处是含有命名空间
constexpr size_t STATE_DIM =
    10; // 状态维度：位置(3) + 速度(3) + yaw(1) + omega(1) + z1(1) + z2(1)
constexpr size_t P_X_IDX = 0;   // 位置x
constexpr size_t P_Y_IDX = 1;   // 位置y
constexpr size_t DP_X_IDX = 2;  // 速度x
constexpr size_t DP_Y_IDX = 3;  // 速度y
constexpr size_t YAW_IDX = 4;   // yaw角
constexpr size_t OMEGA_IDX = 5; // yaw角速度
constexpr size_t Z1_IDX = 6;    // 装甲板1高度
constexpr size_t Z2_IDX = 7;    // 装甲板2高度
constexpr size_t R1_IDX = 8;    // 装甲板1半径
constexpr size_t R2_IDX = 9;    // 装甲板2半径

constexpr size_t MEASURE_X_IDX = 0;   // 测量位置x
constexpr size_t MEASURE_Y_IDX = 1;   // 测量位置y
constexpr size_t MEASURE_Z_IDX = 2;   // 测量位置z
constexpr size_t MEASURE_YAW_IDX = 3; // 测量装甲板yaw

// 名义状态结构体
struct NeSionState_t
{
  using Vector10d_t = Eigen::Matrix<double, STATE_DIM, 1>;

  Eigen::Vector2d p;      // 位置
  Eigen::Vector2d v;      // 速度
  double          yaw;    // 整车yaw角
  double          omega;  // 整车yaw角速度
  double          z1, z2; // 两组装甲板高度
  double          R1, R2; // 两组装甲板半径

  // 构造函数和初始化
  NeSionState_t()
      : p(Eigen::Vector2d::Zero()), v(Eigen::Vector2d::Zero()), yaw(0),
        omega(0), z1(0), z2(0), R1(0.3), R2(0.3)
  {
    // 注意：初始半径不要给0
  }

  // 广义加法重载
  // 输入为误差状态或等同的小量
  NeSionState_t operator+(const Vector10d_t ex) const
  {
    // 只有YAW的操作有所区别，其他都是普通的加法
    NeSionState_t res;
    res.p = p + ex.segment<2>(P_X_IDX);
    res.v = v + ex.segment<2>(DP_X_IDX);
    res.yaw = math::WrapToPi(yaw + ex(YAW_IDX));
    res.omega = omega + ex(OMEGA_IDX);
    res.z1 = z1 + ex(Z1_IDX);
    res.z2 = z2 + ex(Z2_IDX);
    res.R1 = R1 + ex(R1_IDX);
    res.R2 = R2 + ex(R2_IDX);
    return res;
  }

  NeSionState_t& operator+=(const Vector10d_t ex)
  {
    *this = *this + ex;
    return *this;
  }

  // 广义减法重载
  // 输出为误差状态或等同的小量
  Vector10d_t operator-(const NeSionState_t& other) const
  {
    // 只有YAW的操作有所区别，其他都是普通的减法
    Vector10d_t ex;
    ex.segment<2>(P_X_IDX) = p - other.p;
    ex.segment<2>(DP_X_IDX) = v - other.v;
    ex(YAW_IDX) = math::WrapToPi(yaw - other.yaw);
    ex(OMEGA_IDX) = omega - other.omega;
    ex(Z1_IDX) = z1 - other.z1;
    ex(Z2_IDX) = z2 - other.z2;
    ex(R1_IDX) = R1 - other.R1;
    ex(R2_IDX) = R2 - other.R2;
    return ex;
  }
};

class NeSionModel final
{

private:
  using ErrorState_t = Eigen::Matrix<double, 10, 1>;
  using ErrorStateMat_t = Eigen::Matrix<double, 10, 10>;
  using MeasurementNoiseCov_t = Eigen::Matrix<double, 4, 4>;
  using Measurement_t = Eigen::Vector4d;
  using AccDate_t = Eigen::Vector2d;
  using NePeriodicNumber_t = NePeriodicNumber<4>;
  using HJac_t = Eigen::Matrix<double, 4, 10>;
  using K_t = Eigen::Matrix<double, 10, 4>;

public:
  explicit NeSionModel(const interfaces::NeArmors3D_t& init_armors);
  ~NeSionModel() = default;

  // 考虑加速度的预测
  void Predict(const interfaces::NeImuData_t& imu_data);

  // 不考虑加速度的预测
  void Predict();

  // 更新状态，输入3D装甲数据
  void Update(const interfaces::NeArmors3D_t& armors);

  Eigen::Vector3d
  PredictAndChoose(const interfaces::NeImuData_t& imu_data,
                   std::vector<Eigen::Vector3d>&  all_pred_armors);

private:
  // 预测状态
  void
  predictState(double dt, NeSionState_t x, AccDate_t& a, NeSionState_t& x_pred);
  // 计算过程噪声协方差矩阵Q
  void computeQ(double dt, ErrorStateMat_t& Q);
  // 计算测量噪声协方差矩阵R
  void computeR(MeasurementNoiseCov_t& R);
  // 计算预测雅克比（误差状态）
  void computeF(double dt, ErrorStateMat_t& F);
  // 观测方程h
  void h(const NePeriodicNumber_t id, const NeSionState_t& x, Measurement_t& z);

  // 计算H 这里的x有两种可能的取值
  // 1. 名义变量预测结果x_hat
  // 2. 迭代更新过程中不断更新的x
  // 其实这俩也没必要分这么清吧
  void computeH(const NePeriodicNumber_t id, const NeSionState_t& x, HJac_t& H);

  // ID匹配
  bool matchID(const interfaces::NeArmors3D_t::Armor3D_t& armor,
               const Measurement_t&                       z,
               NePeriodicNumber_t&                        matched_id);

  struct
  {
    NeSionState_t   x;      // 当前名义状态
    NeSionState_t   x_pred; // 预测状态
    ErrorStateMat_t eP;     // 误差状态预测协方差

  } esikf_data_;
  struct
  {
    double var_a = 0;
    double var_beta = 0;
    double var_z = 0;
    double var_R = 0;

    size_t max_iter = 5;   // ESIKF 最大更新迭代次数
    double epsilon = 1e-4; // ESIKF 更新迭代收敛阈值

    double additional_dt = 0.02; // 预测时额外增加的时间，单位秒
  } params_;

  struct
  {
    std::chrono::steady_clock::time_point last_predict_time;
    std::chrono::steady_clock::time_point last_update_time;

    double predict_dt;
    double update_dt;
  } time;
};

} // namespace sion

} // namespace ne_vision
