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
#include "ne_vision/utils/ne_rerun_debug.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <vector>

#define _I(exp) (decltype(exp)::Identity())

namespace ne_vision
{
namespace sion
{

/* === AIM PREDICTOR 用来后续规划器预测 === */

bool NeSionAimPredictor::Predict(double                         dt,
                                 const interfaces::NeImuData_t& imu_data,
                                 Eigen::Vector3d&               target_position,
                                 double& target_yaw) const
{
  // 基于当前的系统状态及过去的时差 dt 预测出轨迹的角度
  // 这里名义状态简单的一阶前向积分：
  NeSionState_t x_pred;
  x_pred.p.x() = state_.p.x() + state_.v.x() * dt;
  x_pred.p.y() = state_.p.y() + state_.v.y() * dt;
  // TODO: 后续视业务情况可进一步将车辆旋转 omega 计算在内以及选板逻辑

  // 输出目标的 x y z (预测位置)
  target_position.x() = x_pred.p.x();
  target_position.y() = x_pred.p.y();
  target_position.z() = state_.z1; // 取装甲板的其中之一高度作为追踪目标的高度

  // 预测目标的 yaw 并包装到 pi
  target_yaw = math::WrapToPi(std::atan2(x_pred.p.y(), x_pred.p.x()));

  return true;
}

std::shared_ptr<interfaces::NeAimPredictorBase> NeSionModel::GetAimPredictor(
    std::chrono::steady_clock::time_point       cap_stamp,
    std::chrono::steady_clock::time_point       imu_stamp,
    const std::vector<interfaces::NeImuData_t>& imu_history) const
{
  NeSionState_t fast_state = GetState();
  auto          current_stamp = cap_stamp;

  // 使用所有的有效IMU历史帧进行高频全量积分，剥离滞后的延迟
  for (const auto& imu : imu_history)
  {
    if (imu.receive_stamp <= current_stamp)
      continue;

    double dt = std::chrono::duration<double>(imu.receive_stamp - current_stamp)
                    .count();
    if (dt <= 0.0)
      continue;

    // 提取加速度并积分状态
    AccDate_t a;
    a.x() = 0.0; // imu.acc.x();
    a.y() = 0.0; // imu.acc.y();

    NeSionState_t x_pred;
    predictState(dt, fast_state, a, x_pred);
    fast_state = x_pred;

    current_stamp = imu.receive_stamp;
  }

  return std::make_shared<NeSionAimPredictor>(cap_stamp, imu_stamp, fast_state);
}

NeSionModel::NeSionModel(const interfaces::NeArmors3D_t& init_armors)
{
  // 1. 参数赋值
  params_.LoadParam();

  // 2. 首次初始化
  initializeModel(init_armors, models_);
}

void NeSionModel::Predict(const interfaces::NeImuData_t& imu_data,
                          const double                   dt)
{
  // 1. 计算加速度
  AccDate_t a;
  // a.x() = imu_data.acc.x();
  // a.y() = imu_data.acc.y();
  a.x() = 0.0;
  a.y() = 0.0;

  // 2. 根据当前是否已经选定模型来决定是对单个模型进行预测还是对所有模型进行预测
  if (current_model_idx_ < 0)
    for (auto& model : models_)
      predictAndUpdateOnce(imu_data, dt, a, model);
  else
    predictAndUpdateOnce(imu_data, dt, a, models_.at(current_model_idx_));
}

void NeSionModel::Update(const interfaces::NeArmors3D_t& armors)
{
  // 根据当前是否已经选定模型来决定是对单个模型进行更新还是对所有模型进行更新
  if (current_model_idx_ < 0)
    for (auto& model : models_)
      updateOnce(armors, model, false); // 多模型评估模式，锁死半径
  else
    updateOnce(armors, models_.at(current_model_idx_), true);

  // 看看是否达到了多模型转单模型的条件
  checkMultipleModel(models_);

  // 如果发散了就重置
  if (current_model_idx_ >= 0 && models_.at(current_model_idx_).is_diverged)
  {
    // NV_WARN("Model {} is diverged! Reinitializing...", current_model_idx_);
    initializeModel(armors, models_);
  }
}

void NeSionModel::DebugInfo()
{
  std::string prefix = GetName() + "/";
  int         log_idx = current_model_idx_ >= 0 ? current_model_idx_ : 0;
  const auto& model = models_.at(log_idx);
  const auto& x = model.esikf_data.x;

  if (params_.debug.yaw)
    NV_REC_LOG(prefix + "yaw", rerun::Scalars(x.yaw));

  if (params_.debug.omega)
    NV_REC_LOG(prefix + "omega", rerun::Scalars(x.omega));

  if (params_.debug.R)
  {
    NV_REC_LOG(prefix + "R1", rerun::Scalars(x.R1));
    NV_REC_LOG(prefix + "R2", rerun::Scalars(x.R2));
  }

  if (params_.debug.Q_var)
  {
    NV_REC_LOG(prefix + "var_a", rerun::Scalars(model.current_var_a));
    NV_REC_LOG(prefix + "var_beta", rerun::Scalars(model.current_var_beta));
  }

  if (params_.debug.nis)
    NV_REC_LOG(prefix + "nis", rerun::Scalars(model.last_nis));

  if (params_.debug.dis)
    NV_REC_LOG(prefix + "dis", rerun::Scalars(model.last_dis));

  if (params_.debug.model_idx)
    NV_REC_LOG(prefix + "model_idx", rerun::Scalars(current_model_idx_));
}

/* === 统合函数区 === */

void NeSionModel::initializeModel(const interfaces::NeArmors3D_t& init_armors,
                                  Models_t&                       models)
{
  if (init_armors.armors.empty())
  {
    NV_WARN("No armors observed during model initialization!");
    return;
  }

  // 填写状态相同部分
  models.at(0).esikf_data.x.p.x() = init_armors.armors[0].t.x();
  models.at(0).esikf_data.x.p.y() = init_armors.armors[0].t.y();
  models.at(0).esikf_data.x.yaw = math::WrapToPi(init_armors.armors[0].yaw);
  models.at(0).esikf_data.x.v.x() = 0;
  models.at(0).esikf_data.x.v.y() = 0;
  models.at(0).esikf_data.x.R1 = params_.init_R;
  models.at(0).esikf_data.x.R2 = params_.init_R;
  models.at(0).esikf_data.x.z1 = init_armors.armors[0].t.z();
  models.at(0).esikf_data.x.z2 = init_armors.armors[0].t.z();
  models.at(0).current_var_a = std::sqrt(params_.var_a_min * params_.var_a_max);
  models.at(0).var_a_base = models.at(0).current_var_a;
  models.at(0).current_var_beta =
      std::sqrt(params_.var_beta_min * params_.var_beta_max);
  models.at(0).divergence_count = 0;
  models.at(0).last_nis = 0;
  models.at(0).last_dis = 0;

  // 初始化相同的协方差
  models.at(0).esikf_data.eP = ErrorStateMat_t::Identity() * 1e-3;

  // 初始化评估信息
  models.at(0).cumulative_cost = 0;
  models.at(0).update_count = 0;

  models.at(1) = models.at(0);
  models.at(2) = models.at(0);

  models.at(0).esikf_data.x.omega = 0;
  models.at(1).esikf_data.x.omega = params_.init_omega;
  models.at(2).esikf_data.x.omega = -params_.init_omega;

  current_model_idx_ = -1; // 重置为未选定状态
}

// 针对特定的status进行一次预测更新
void NeSionModel::predictAndUpdateOnce(const interfaces::NeImuData_t& imu_data,
                                       const double                   dt,
                                       const AccDate_t&               a,
                                       ModelStatus_t&                 model)
{
  auto& x = model.esikf_data.x;
  auto& x_pred = model.esikf_data.x_pred;
  auto& eP = model.esikf_data.eP;

  // 1. 进行状态预测
  predictState(dt, x, a, x_pred);

  // 2. 更新误差状态预测协方差
  ErrorStateMat_t Q, F;
  computeQ(dt, model.current_var_a, model.current_var_beta, Q);
  computeF(dt, F);
  eP = F * eP * F.transpose() + Q;
}

// 针对特定的模型，给定一组装甲板数据，更新一次状态
void NeSionModel::updateOnce(const interfaces::NeArmors3D_t& armors,
                             ModelStatus_t&                  model,
                             bool                            model_is_only_one)
{
  auto& x = model.esikf_data.x;
  auto& x_pred = model.esikf_data.x_pred;
  auto& eP = model.esikf_data.eP;

  auto update_one = [&](const interfaces::NeArmors3D_t::Armor3D_t& armor) {
    // 1. 处理观测值和观测协方差
    Measurement_t z;
    z(MEASURE_X_IDX) = armor.t.x();
    z(MEASURE_Y_IDX) = armor.t.y();
    z(MEASURE_Z_IDX) = armor.t.z();
    z(MEASURE_YAW_IDX) = math::WrapToPi(armor.yaw);
    auto& R = armor.cov;

    // 2. 处理ID，判断属于车的哪个装甲板
    NePeriodicNumber_t matched_id;
    double             cost; // 对数似然代价
    double             nis;  // NIS值
    if (!matchID(armor, z, x_pred, eP, matched_id, cost, nis))
    {
      // 如果没有成功匹配就放弃本轮更新
      return;
    }

    // 3. 发散检测和自适应Q
    adaptiveQAndDivergenceCheck(model, nis);

    // 记录debug信息
    model.last_nis = nis;
    model.last_dis = std::sqrt(z(MEASURE_X_IDX) * z(MEASURE_X_IDX) +
                               z(MEASURE_Y_IDX) * z(MEASURE_Y_IDX) +
                               z(MEASURE_Z_IDX) * z(MEASURE_Z_IDX));

    // 4. 计算累计代价，用于进行多模型评估，低通滤波累计
    if (model.update_count == 0)
      model.cumulative_cost = cost;
    else
      model.cumulative_cost =
          params_.init_alpha * model.cumulative_cost +
          (1 - params_.init_alpha) * cost; // 低通滤波累计代价
    model.update_count++;

    // 5. 迭代更新
    HJac_t H;
    K_t    K;
    auto   x_pred_start = x_pred; // 最初的先验
    for (size_t iter = 0; iter < params_.max_iter; ++iter)
    {
      // 1 计算测量预测值
      Measurement_t z_pred;
      h(matched_id, x_pred, z_pred);

      // 2 计算测量残差
      Measurement_t r = z - z_pred;
      r(MEASURE_YAW_IDX) = math::WrapToPi(r(MEASURE_YAW_IDX));

      // 3 计算测量雅克比
      computeH(matched_id, x_pred, H);

      // 4 计算卡尔曼增益
      // 相比于文档，这里使用的是其等价形式，可以推出来。这里计算速度更快
      auto S = H * eP * H.transpose() + R;  // 预测测量协方差
      K = eP * H.transpose() * S.inverse(); // 卡尔曼增益

      // 5. 角速度死区处理，如果角速度很小，就不更新半径
      //    前如果是多假设观测，则需要锁死半径
#if 1
      if (std::abs(x_pred.omega) < params_.omega_dead_band ||
          !model_is_only_one)
      {
        K(R1_IDX, MEASURE_X_IDX) = 0;
        K(R1_IDX, MEASURE_Y_IDX) = 0;
        K(R1_IDX, MEASURE_Z_IDX) = 0;
        K(R1_IDX, MEASURE_YAW_IDX) = 0;

        K(R2_IDX, MEASURE_X_IDX) = 0;
        K(R2_IDX, MEASURE_Y_IDX) = 0;
        K(R2_IDX, MEASURE_Z_IDX) = 0;
        K(R2_IDX, MEASURE_YAW_IDX) = 0;
      }
#endif

      // 6 更新名义状态
      auto ex = K * r -
                (_I(K * H) - K * H) * (x_pred - x_pred_start); // 误差状态更新量
      x_pred += ex;                                            // 更新名义状态
      if (ex.norm() < params_.epsilon)
      {
        // 如果更新量足够小就认为收敛了，提前退出迭代
        break;
      }
    }

    // 4. 更新状态和协方差
    x = x_pred;
    eP = (_I(K * H) - K * H) * eP;

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

// 判断当前应该跟踪多个模型还是模型，应该跟踪哪一个
void NeSionModel::checkMultipleModel(const Models_t& models)
{
  if (current_model_idx_ < 0)
  {
    // 1. 检查有效帧数是否达标 -> 至少有一个模型的更新帧数达标了
    if (std::none_of(
            models.begin(), models.end(), [&](const ModelStatus_t& model) {
              return model.update_count >= params_.init_max_count_value;
            }))
    {
      // 如果没有任何一个模型的更新帧数达标，就继续保持多模型状态
      return;
    }

    // TODO: 这里有一个安全隐患：就是如果观测很差很差，导致完全没有任何有效观测
    //       有可能寄掉

    // 2. 提取并排序
    std::vector<std::pair<double, int>> cost_ranking = {
        {models.at(0).cumulative_cost, 0},
        {models.at(1).cumulative_cost, 1},
        {models.at(2).cumulative_cost, 2},
    };
    std::sort(cost_ranking.begin(), cost_ranking.end());

    int best_idx = cost_ranking.at(0).second;

    current_model_idx_ = best_idx;

    NV_DEBUG("Model {}, M1{} M2{} M3{}",
             best_idx,
             cost_ranking.at(0).first,
             cost_ranking.at(1).first,
             cost_ranking.at(2).first);
  }
}

// 发散返回True
void NeSionModel::adaptiveQAndDivergenceCheck(ModelStatus_t& model, double nis)
{
  if (nis > params_.divergence_threshold)
    model.divergence_count++;
  else
    model.divergence_count = 0;

  // if (model.divergence_count > params_.max_divergence_count)
  // {
  //   model.is_diverged = true;
  //   return;
  // }

  // 自适应Q调节
  double scale_factor = nis / 4.0;

  // 计算目标协方差
  double target_var_a = model.var_a_base * scale_factor;
  double target_var_beta = model.var_beta_base * scale_factor;

  // 限制在合理范围内
  target_var_a = std::clamp(target_var_a, params_.var_a_min, params_.var_a_max);
  target_var_beta =
      std::clamp(target_var_beta, params_.var_beta_min, params_.var_beta_max);

  // 平滑调整当前协方差
  model.current_var_a +=
      params_.q_scale_rate * (target_var_a - model.current_var_a);
  model.current_var_beta +=
      params_.q_scale_rate * (target_var_beta - model.current_var_beta);

  model.is_diverged = false;
}

/* === 工具函数区 === */

Eigen::Vector3d
NeSionModel::PredictAndChoose(const interfaces::NeImuData_t& imu_data,
                              double                         b_dt,
                              std::vector<Eigen::Vector3d>&  all_pred_armors)
{
  // // 预测和选板

  // // 1. 预测固定时间
  // AccDate_t a;

  // double total_dt = params_.additional_dt + b_dt;

  // predictState(total_dt, _x, a, _x_pred);

  // // 2. 选板
  // //    我们选择最正对云台的一块装甲板

  // double min_yaw_diff = std::numeric_limits<double>::max();

  // Eigen::Vector3d aim_point = {_x_pred.p.x(), _x_pred.p.y(), 0};

  // all_pred_armors.clear();
  // for (int id = 0; id < 4; ++id)
  // {
  //   Measurement_t z_pred;
  //   h(id, _x_pred, z_pred);

  //   double yaw_a = z_pred(MEASURE_YAW_IDX);
  //   double yaw_b = math::QuaternionToYaw(imu_data.quat);

  //   double yaw_diff = math::WrapToPi(yaw_a - yaw_b);

  //   if (yaw_diff < min_yaw_diff)
  //   {
  //     min_yaw_diff = yaw_diff;
  //     aim_point.x() = z_pred(MEASURE_X_IDX);
  //     aim_point.y() = z_pred(MEASURE_Y_IDX);
  //     aim_point.z() = z_pred(MEASURE_Z_IDX);
  //   }
  //   all_pred_armors.emplace_back(
  //       z_pred(MEASURE_X_IDX), z_pred(MEASURE_Y_IDX), z_pred(MEASURE_Z_IDX));
  // }

  // // NV_DEBUG("{} {} {}", aim_point.x(), aim_point.y(), aim_point.z());

  return {};
}

// 名义状态预测函数
void NeSionModel::predictState(const double        dt,
                               const NeSionState_t x,
                               const AccDate_t&    a,
                               NeSionState_t&      x_pred) const
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

  if (x_pred.R1 <= 0)
    x_pred.R1 = 0.3;
  if (x_pred.R2 <= 0)
    x_pred.R2 = 0.3;
}

// 计算过程噪声协方差矩阵Q
void NeSionModel::computeQ(double           dt,
                           double           var_a,
                           double           var_beta,
                           ErrorStateMat_t& Q)
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
  double q_a = var_a;
  Q.block<2, 2>(P_X_IDX, P_X_IDX) =
      0.25 * q_a * dt4 * Eigen::Matrix2d::Identity(); // 位置方差
  Q.block<2, 2>(DP_X_IDX, DP_X_IDX) =
      q_a * dt2 * Eigen::Matrix2d::Identity(); // 速度方差
  Q.block<2, 2>(P_X_IDX, DP_X_IDX) =
      0.5 * q_a * dt3 * Eigen::Matrix2d::Identity(); // Pos-Vel
  Q.block<2, 2>(DP_X_IDX, P_X_IDX) =
      0.5 * q_a * dt3 * Eigen::Matrix2d::Identity(); // Vel-Pos

  // 2. 旋转部分
  double q_beta = var_beta;
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
  double yaw_offset = -id * M_PI / 2.0;           // 根据ID计算yaw偏移
  int    r_idx = (id % 2 == 0) ? R1_IDX : R2_IDX; // 根据ID选择R1或R2的索引
  int    z_idx = (id % 2 == 0) ? Z1_IDX : Z2_IDX; // 根据ID选择z1或z2的索引
  double R = (id % 2 == 0) ? x.R1 : x.R2;         // 根据ID选择R1或R2的值

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
                          const NeSionState_t&                       x_pred,
                          const ErrorStateMat_t&                     eP,
                          NePeriodicNumber_t&                        matched_id,
                          double&                                    cost,
                          double&                                    nis)
{
  // 来自julyfun的思路，让我们把整车模型展开来看，看看这个装甲板花落谁家

  // 打包观测协方差为矩阵
  auto& R = armor.cov;

  // 计算马氏距离
  auto compute_mahalanobis_distance = [&](const NePeriodicNumber_t id,
                                          double& tmp_cost) -> double {
    // 1. 获取从预测值得到的观测值
    Measurement_t z_pred;
    h(id, x_pred, z_pred);

    // 2. 计算残差
    Eigen::Vector4d residual = z - z_pred;
    residual(MEASURE_YAW_IDX) = math::WrapToPi(residual(MEASURE_YAW_IDX));

    // 3. 获取新息协方差S
    HJac_t H;
    computeH(id, x_pred, H);
    Eigen::Matrix4d S = H * eP * H.transpose() + R;

    // 4. 计算马氏距离
    double mahalanobis_distance =
        residual.transpose() * S.llt().solve(residual);

    // 5. 输出S的行列式（用于外部逻辑用来决定该模型是否可靠）
    tmp_cost = S.determinant();
    tmp_cost = tmp_cost > 0 ? std::log(tmp_cost) : 0.0;
    tmp_cost += mahalanobis_distance;

    return mahalanobis_distance;
  };

  double min_mahalanobis_distance = std::numeric_limits<double>::max();
  double best_cost = std::numeric_limits<double>::max();
  int    best_id = -1;
  for (int id = 0; id < 4; ++id)
  {
    // 限制范围，如果yaw差值过大，就不考虑这个ID了，直接跳过
    Measurement_t z_pred;
    h(id, x_pred, z_pred);
    double yaw_diff =
        math::WrapToPi(z_pred(MEASURE_YAW_IDX) - z(MEASURE_YAW_IDX));
    if (std::abs(yaw_diff) > M_PI / 2.0)
      continue;

    double distance = compute_mahalanobis_distance(id, cost);
    // TODO: 这里加入阈值判断
    // if (distance >= 500)
    //   continue;
    if (distance < min_mahalanobis_distance)
    {
      min_mahalanobis_distance = distance;
      best_cost = cost;
      best_id = id;
    }
  }

  if (best_id == -1)
  {
    return false;
  }

  cost = best_cost;
  matched_id = best_id;
  nis = min_mahalanobis_distance;

  return true;
}

} // namespace sion
} // namespace ne_vision
