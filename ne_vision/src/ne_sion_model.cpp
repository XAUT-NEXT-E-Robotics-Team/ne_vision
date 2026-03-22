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

#include "ne_vision/models/ne_sion_model.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_math.hpp"
#include "ne_vision/utils/ne_param.hpp"

#include "ne_vision/ballistic_compensation/ballistic_slove.hpp"

#include "ne_vision/debug/ne_vision_visualization.hpp"
#include <Eigen/src/Core/Matrix.h>

#define _x      esikf_data_.x
#define _x_pred esikf_data_.x_pred
#define _eP     esikf_data_.eP

#define _I(exp) (decltype(exp)::Identity())

namespace ne_vision
{
namespace sion
{

NeSionModel::NeSionModel(const interfaces::NeArmors3D_t& init_armors)
{
  // 初始化参数和状态信息
  auto param = NV_PARAM["auto_aim"]["tracker_3d"]["sion_model"];

  // 过程噪声
  params_.var_a = param["var_a"].as<double>();       // 加速度噪声方差
  params_.var_beta = param["var_beta"].as<double>(); // 角加速度噪声方差
  params_.var_z = param["var_z"].as<double>();       // 高度噪声方差
  params_.var_R = param["var_R"].as<double>();       // 半径噪声方差

  params_.additional_dt =
      param["additional_dt"].as<double>(); // 预测时额外增加的时间，单位秒

  // 初始化协方差
  _eP = ErrorStateMat_t::Identity() * 1e-3;

  // 初始化状态
  if (!init_armors.armors.empty())
  {
    _x.p.x() = init_armors.armors[0].t.x();
    _x.p.y() = init_armors.armors[0].t.y();
    _x.yaw = init_armors.armors[0].yaw.log();
    _x.v.x() = 0;
    _x.v.y() = 0;
    _x.R1 = 0.3;
    _x.R2 = 0.3;
    _x.z1 = init_armors.armors[0].t.z();
    _x.z2 = init_armors.armors[0].t.z();
  }
}

void NeSionModel::Predict()
{
  // 不使用加速度先验

  // 1. 计算dt
  auto now = std::chrono::steady_clock::now();
  time.predict_dt =
      std::chrono::duration<double>(now - time.last_predict_time).count();
  time.last_predict_time = now;

  // 2. 进行状态预测
  AccDate_t a = AccDate_t::Zero(); // 不使用加速度先验，设为0
  predictState(time.predict_dt, _x, a, _x_pred);

  // 3. 更新误差状态预测协方差
  ErrorStateMat_t Q, F;
  computeQ(time.predict_dt, Q);
  computeF(time.predict_dt, F);
  _eP = F * _eP * F.transpose() + Q;
}

void NeSionModel::Update(const interfaces::NeArmors3D_t& armors)
{
  auto update_one = [&](const interfaces::NeArmors3D_t::Armor3D_t& armor) {
    // 1. 处理观测值和观测协方差
    Measurement_t z;
    z(MEASURE_X_IDX) = armor.t.x();
    z(MEASURE_Y_IDX) = armor.t.y();
    z(MEASURE_Z_IDX) = armor.t.z();
    z(MEASURE_YAW_IDX) = armor.yaw.log();
    auto& R = armor.cov;

    // 2. 处理ID，判断属于车的哪个装甲板
    NePeriodicNumber_t matched_id;
    if (!matchID(armor, z, matched_id))
    {
      // 如果没有成功匹配就放弃本轮更新
      return;
    }

    // 3. 迭代更新
    HJac_t H;
    K_t    K;
    auto   x_pred_start = _x_pred; // 最初的先验
    for (size_t iter = 0; iter < params_.max_iter; ++iter)
    {
      // 1 计算测量预测值
      Measurement_t z_pred;
      h(matched_id, _x_pred, z_pred);

      // 2 计算测量残差
      Measurement_t r = z - z_pred;
      r(MEASURE_YAW_IDX) = math::WrapToPi(r(MEASURE_YAW_IDX));

      // 3 计算测量雅克比
      computeH(matched_id, _x_pred, H);

      // 4 计算卡尔曼增益
      // 相比于文档，这里使用的是其等价形式，可以推出来。这里计算速度更快
      auto S = H * _eP * H.transpose() + R;  // 预测测量协方差
      K = _eP * H.transpose() * S.inverse(); // 卡尔曼增益

      // 5 更新名义状态
      auto ex = K * r - (_I(K * H) - K * H) *
                            (_x_pred - x_pred_start); // 误差状态更新量
      _x_pred += ex;                                  // 更新名义状态
      if (ex.norm() < params_.epsilon)
      {
        // 如果更新量足够小就认为收敛了，提前退出迭代
        break;
      }
    }

    // 4. 更新状态和协方差
    _x = _x_pred;
    _eP = (_I(K * H) - K * H) * _eP;

    // nv_rec_g().log("car_yaw", rerun::Scalars(_x.yaw));
    // nv_rec_g().log("car_R1", rerun::Scalars(_x.R1));
    // nv_rec_g().log("car_R2", rerun::Scalars(_x.R2));
  };

  if (armors.armors.empty())
  {
    // 没有观测到装甲板，直接更新协方差
    return;
  }
  else if (armors.armors.size() == 1)
  {
    // 观测到一个装甲板
    auto& armor = armors.armors[0];

    update_one(armor);
  }
  else if (armors.armors.size() == 2)
  {
    // 观测到两个装甲板

    // 选择最近的丢到上面那个去
    auto&  armor1 = armors.armors[0];
    auto&  armor2 = armors.armors[1];
    double dist1 = armor1.t.norm();
    double dist2 = armor2.t.norm();
    auto&  selected_armor = dist1 < dist2 ? armor1 : armor2;
    update_one(selected_armor);
  }
  else
  {
    // 你不对劲
    NV_WARN("Unexpected number of armors observed: {}", armors.armors.size());
    return;
  }
}

Eigen::Vector3d
NeSionModel::PredictAndChoose(const interfaces::NeImuData_t& imu_data,
                              double                         b_dt,
                              std::vector<Eigen::Vector3d>&  all_pred_armors)
{
  // 预测和选板

  // 1. 预测固定时间
  AccDate_t a;

  double total_dt = params_.additional_dt + b_dt;

  predictState(total_dt, _x, a, _x_pred);

  // 2. 选板
  //    我们选择最正对云台的一块装甲板

  double min_yaw_diff = std::numeric_limits<double>::max();

  Eigen::Vector3d aim_point = {_x_pred.p.x(), _x_pred.p.y(), 0};

  all_pred_armors.clear();
  for (int id = 0; id < 4; ++id)
  {
    Measurement_t z_pred;
    h(id, _x_pred, z_pred);

    double yaw_a = z_pred(MEASURE_YAW_IDX);
    double yaw_b = math::QuaternionToYaw(imu_data.quat);

    double yaw_diff = math::WrapToPi(yaw_a - yaw_b);

    if (yaw_diff < min_yaw_diff)
    {
      min_yaw_diff = yaw_diff;
      aim_point.x() = z_pred(MEASURE_X_IDX);
      aim_point.y() = z_pred(MEASURE_Y_IDX);
      aim_point.z() = z_pred(MEASURE_Z_IDX);
    }
    all_pred_armors.emplace_back(
        z_pred(MEASURE_X_IDX), z_pred(MEASURE_Y_IDX), z_pred(MEASURE_Z_IDX));
  }

  // NV_DEBUG("{} {} {}", aim_point.x(), aim_point.y(), aim_point.z());

  return aim_point;
}

// 名义状态预测函数
void NeSionModel::predictState(double         dt,
                               NeSionState_t  x,
                               AccDate_t&     a,
                               NeSionState_t& x_pred)
{
  // 位置预测：p' = p + v*dt + 0.5*a*dt^2
  x_pred.p = x.p + x.v * dt + 0.5 * a * dt * dt;

  // 速度预测：v' = v + a*dt
  x_pred.v = x.v + a * dt;

  // yaw预测：yaw' = yaw + omega*dt
  x_pred.yaw = x.yaw + x.omega * dt;
  x_pred.yaw = math::WrapToPi(x_pred.yaw); // 保持在[-pi, pi]

  // omega预测：omega' = omega + beta*dt
  x_pred.omega = x.omega;

  // z1和z2保持不变
  x_pred.z1 = x.z1;
  x_pred.z2 = x.z2;

  // R1和R2保持不变
  x_pred.R1 = x.R1;
  x_pred.R2 = x.R2;
}

// 计算过程噪声协方差矩阵Q
void NeSionModel::computeQ(double dt, ErrorStateMat_t& Q)
{
  // 整体结构
  //     [ Q_trans (4x4) |       0       |       0         ]
  // Q = [       0       |  Q_rot (2x2)  |       0         ]
  //     [       0       |       0       |  Q_static(4x4)  ]
  //
  // ---------------------------------------------------------
  // 1. 平移部分 Q_trans (CV模型, 离散化白噪声加速度)
  // 状态: [p_x, p_y, v_x, v_y]^T
  //           [ 1/4*dt^4      0      1/2*dt^3      0      ]
  // Q_trans = [     0     1/4*dt^4      0      1/2*dt^3   ] * var_a
  //           [ 1/2*dt^3      0        dt^2        0      ]
  //           [     0     1/2*dt^3      0        dt^2     ]
  // * ---------------------------------------------------------
  // 2. 旋转部分 Q_rot (CV模型, 离散化白噪声角加速度)
  // 状态: [yaw, omega]^T
  //         [ 1/4*dt^4  1/2*dt^3 ]
  // Q_rot = [                    ] * var_beta
  //         [ 1/2*dt^3    dt^2   ]
  //
  // ---------------------------------------------------------
  // 3. 静态参数部分 Q_static (随机游走模型 Random Walk)
  // 状态: [z1, z2, R1, R2]^T
  //            [ var_z*dt      0          0          0      ]
  //            [     0      var_z*dt      0          0      ]
  // Q_static = [     0         0       var_R*dt      0      ]
  //            [     0         0          0      var_R*dt   ]
  //
  // dt 距离上一次预测的时间间隔
  // Q  输出的 10x10 过程噪声协方差矩阵

  Q.setZero();

  // 提前计算 dt 的高次幂以减少重复计算
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;

  // 1. 平移部分
  double q_a = params_.var_a;
  Q.block<2, 2>(P_X_IDX, P_X_IDX) =
      0.25 * q_a * dt4 * Eigen::Matrix2d::Identity(); // 位置方差
  Q.block<2, 2>(DP_X_IDX, DP_X_IDX) =
      q_a * dt2 * Eigen::Matrix2d::Identity(); // 速度方差
  Q.block<2, 2>(P_X_IDX, DP_X_IDX) =
      0.5 * q_a * dt3 * Eigen::Matrix2d::Identity(); // Pos-Vel
  Q.block<2, 2>(DP_X_IDX, P_X_IDX) =
      0.5 * q_a * dt3 * Eigen::Matrix2d::Identity(); // Vel-Pos

  // 2. 旋转部分
  double q_beta = params_.var_beta;
  Q(YAW_IDX, YAW_IDX) = 0.25 * q_beta * dt4;  // Yaw方差
  Q(OMEGA_IDX, OMEGA_IDX) = q_beta * dt2;     // Omega方差
  Q(YAW_IDX, OMEGA_IDX) = 0.5 * q_beta * dt3; // Yaw-Omega
  Q(OMEGA_IDX, YAW_IDX) = 0.5 * q_beta * dt3; // Omega-Yaw

  // 3. 静态常数部分 (随机游走模型)
  Q(Z1_IDX, Z1_IDX) = params_.var_z * dt;
  Q(Z2_IDX, Z2_IDX) = params_.var_z * dt;
  Q(R1_IDX, R1_IDX) = params_.var_R * dt;
  Q(R2_IDX, R2_IDX) = params_.var_R * dt;
}

// 计算预测雅克比
void NeSionModel::computeF(double dt, ErrorStateMat_t& F)
{
  F = ErrorStateMat_t::Identity();
  F(0, 2) = dt; // 位置 p_x 对 速度 v_x 的偏导
  F(1, 3) = dt; // 位置 p_y 对 速度 v_y 的偏导
  F(4, 5) = dt; // 角度 yaw 对 角速度 omega 的偏导
}

// 观测函数h
void NeSionModel::h(const NePeriodicNumber_t id,
                    const NeSionState_t&     x,
                    Measurement_t&           z)
{
  // 1. 根据id获取对应的装甲板参数
  const double x_z = (id % 2 == 0) ? x.z1 : x.z2; // 根据id选择z1或z2
  const double x_R = (id % 2 == 0) ? x.R1 : x.R2; // 根据id选择R1或R2

  // 该ID的对应装甲板是机体x轴逆时针转过多少度（相对于机体坐标系）
  const double offset_yaw = -id * M_PI / 2.0;

  // 2. 根据状态计算观测值
  z(MEASURE_X_IDX) = x.p.x() + x_R * std::cos(x.yaw + offset_yaw);
  z(MEASURE_Y_IDX) = x.p.y() + x_R * std::sin(x.yaw + offset_yaw);
  z(MEASURE_Z_IDX) = x_z;
  z(MEASURE_YAW_IDX) = x.yaw + offset_yaw;
}

void NeSionModel::computeH(const NePeriodicNumber_t id,
                           const NeSionState_t&     x,
                           HJac_t&                  H)
{
  H = HJac_t::Zero();

  // 1. 根据不同装甲板编号提取对应的参数和索引
  double yaw_offset = -id * M_PI / 2.0;        // 根据ID计算yaw偏移
  int r_idx = (id % 2 == 0) ? R1_IDX : R2_IDX; // 根据ID选择R1或R2的索引
  int z_idx = (id % 2 == 0) ? Z1_IDX : Z2_IDX; // 根据ID选择z1或z2的索引
  double R = (id % 2 == 0) ? x.R1 : x.R2;      // 根据ID选择R1或R2的值

  // 2. 提前计算三角函数
  double theta_total = x.yaw + yaw_offset;
  double sin_theta = std::sin(theta_total);
  double cos_theta = std::cos(theta_total);

  // 3. 填充非零偏导数
  // 对 p_x, p_y 的偏导
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;

  // 对 yaw 的偏导
  H(0, 4) = -R * sin_theta;
  H(1, 4) = R * cos_theta;
  H(3, 4) = 1.0;

  // 对 z (z1或z2) 的偏导
  H(2, z_idx) = 1.0;

  // 对 R (R1或R2) 的偏导
  H(0, r_idx) = cos_theta;
  H(1, r_idx) = sin_theta;
}

// 马氏距离ID匹配
// 可能会出现匹配失败。我们可以合理的设置阈值来判断是否为合理的匹配
// 宁可错杀一千，也不放过一个，不要让错误的匹配污染观测器
bool NeSionModel::matchID(const interfaces::NeArmors3D_t::Armor3D_t& armor,
                          const Measurement_t&                       z,
                          NePeriodicNumber_t&                        matched_id)
{
  // 来自julyfun的思路，让我们把整车模型展开来看，看看这个装甲板花落谁家

  // 打包观测协方差为矩阵
  auto& R = armor.cov;

  // 计算马氏距离
  auto compute_mahalanobis_distance =
      [&](const NePeriodicNumber_t id) -> double {
    // 1. 获取从预测值得到的观测值
    Measurement_t z_pred;
    h(id, _x_pred, z_pred);

    // 2. 计算残差
    Eigen::Vector4d residual = z - z_pred;
    residual(MEASURE_YAW_IDX) = math::WrapToPi(residual(MEASURE_YAW_IDX));

    // 3. 获取新息协方差S
    HJac_t H;
    computeH(id, _x_pred, H);
    Eigen::Matrix4d S = H * _eP * H.transpose() + R;

    // 4. 计算马氏距离
    double mahalanobis_distance =
        residual.transpose() * S.llt().solve(residual);

    return mahalanobis_distance;
  };

  double min_mahalanobis_distance = std::numeric_limits<double>::max();
  int    best_id = -1;
  for (int id = 0; id < 4; ++id)
  {
    double distance = compute_mahalanobis_distance(id);
    // TODO: 这里加入阈值判断
    // if (distance >= 500)
    //   continue;
    if (distance < min_mahalanobis_distance)
    {
      min_mahalanobis_distance = distance;
      best_id = id;
    }

    // NV_INFO("ID:{} Mahalanobis Distance: {} Z_YAW: {}",
    //         id,
    //         distance,
    //         math::WrapToPi(z(MEASURE_YAW_IDX)));
  }

  if (best_id == -1)
  {
    // NV_INFO("NOOOO {}", min_mahalanobis_distance);
    return false;
  }

  matched_id = best_id;

  // NV_INFO("ID:{}", matched_id);

  return true;
}

} // namespace sion
} // namespace ne_vision
