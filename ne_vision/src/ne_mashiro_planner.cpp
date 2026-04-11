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

#include "ne_vision/planner/ne_mashiro_planner.hpp"

#include <cmath>
#include <vector>

#include "ne_vision/ballistic_compensation/model_param.hpp"
#include "tinympc/tiny_api.hpp"
#include "tinympc/types.hpp"

#include "ne_vision/utils/ne_debug.hpp"
#include "ne_vision/utils/ne_math.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"

#include "ne_vision/interfaces/ne_gimbal_control_ref.hpp"
#include "ne_vision/interfaces/ne_aim_state.hpp"

#include "ne_vision/ne_channals.hpp"
#include "ne_vision/ballistic_compensation/ballistic_slove.hpp"

namespace ne_vision
{

NeMashiroPlanner::NeMashiroPlanner(std::string name) : name_(std::move(name))
{
  // 读取参数
  auto   param_node = NV_PARAM["auto_aim"]["mashiro_planner"];
  double q_y_pos = param_node["q_y_pos"].as<double>(100.0);
  double q_y_vel = param_node["q_y_vel"].as<double>(1.0);
  double q_p_pos = param_node["q_p_pos"].as<double>(100.0);
  double q_p_vel = param_node["q_p_vel"].as<double>(1.0);
  double r_y_acc = param_node["r_y_acc"].as<double>(10.0);
  double r_p_acc = param_node["r_p_acc"].as<double>(10.0);

  // 箱式约束参数
  double max_y_vel = param_node["max_y_vel"].as<double>(10.0);
  double max_p_vel = param_node["max_p_vel"].as<double>(10.0);
  double max_y_acc = param_node["max_y_acc"].as<double>(20.0);
  double max_p_acc = param_node["max_p_acc"].as<double>(20.0);

  // 设置几个矩阵
  mpc_mats_.Adyn.resize(kStateDim, kStateDim);
  mpc_mats_.Adyn.setIdentity();
  mpc_mats_.Adyn(0, 1) = params_.step;
  mpc_mats_.Adyn(2, 3) = params_.step;

  mpc_mats_.Bdyn.resize(kStateDim, kControlDim);
  mpc_mats_.Bdyn.setZero();
  mpc_mats_.Bdyn(0, 0) = 0.5 * params_.step * params_.step;
  mpc_mats_.Bdyn(1, 0) = params_.step;
  mpc_mats_.Bdyn(2, 1) = 0.5 * params_.step * params_.step;
  mpc_mats_.Bdyn(3, 1) = params_.step;

  mpc_mats_.fdyn.resize(kStateDim, 1);
  mpc_mats_.fdyn.setZero();

  mpc_mats_.Q.resize(kStateDim, kStateDim);
  mpc_mats_.Q.setZero();
  mpc_mats_.Q(0, 0) = q_y_pos;
  mpc_mats_.Q(1, 1) = q_y_vel;
  mpc_mats_.Q(2, 2) = q_p_pos;
  mpc_mats_.Q(3, 3) = q_p_vel;

  mpc_mats_.R.resize(kControlDim, kControlDim);
  mpc_mats_.R.setZero();
  mpc_mats_.R(0, 0) = r_y_acc;
  mpc_mats_.R(1, 1) = r_p_acc;

  mpc_mats_.max_x.resize(kStateDim, params_.horizon);
  mpc_mats_.min_x.resize(kStateDim, params_.horizon);
  mpc_mats_.max_u.resize(kControlDim, params_.horizon - 1);
  mpc_mats_.min_u.resize(kControlDim, params_.horizon - 1);

  Vec_x_t single_max_x, single_min_x;
  Vec_u_t single_max_u, single_min_u;

  single_max_x << std::numeric_limits<tinytype>::max(), max_y_vel,
      std::numeric_limits<tinytype>::max(), max_p_vel;
  single_min_x << std::numeric_limits<tinytype>::lowest(), -max_y_vel,
      std::numeric_limits<tinytype>::lowest(), -max_p_vel;
  single_max_u << max_y_acc, max_p_acc;
  single_min_u << -max_y_acc, -max_p_acc;

  mpc_mats_.max_x.colwise() = single_max_x;
  mpc_mats_.min_x.colwise() = single_min_x;
  mpc_mats_.max_u.colwise() = single_max_u;
  mpc_mats_.min_u.colwise() = single_min_u;

  // 初始化求解器 小心：返回0为成功
  if (tiny_setup(&solver_ptr_,
                 mpc_mats_.Adyn,
                 mpc_mats_.Bdyn,
                 mpc_mats_.fdyn,
                 mpc_mats_.Q,
                 mpc_mats_.R,
                 rho_value_,
                 kStateDim,
                 kControlDim,
                 params_.horizon,
                 1) != 0)
  {
    NV_ASSERT(0 && "Failed to Set Up TinyMPC Solver");
  }
}

void NeMashiroPlanner::Plan()
{
  interfaces::NeAimState_t aim_state_i;

  if (!NV_CHANNELS.aim_state_sPtr()->Receive(aim_state_i))
  {
    NV_WARN("Waiting for aim state...");
    return;
  }

  // 读取机器人状态（主要获取弹速）
  if (!NV_CHANNELS.robot_state_sPtr()->Receive(robot_state_i_))
  {
    NV_WARN("Waiting for robot state...");
  }

  // 先行转换
  double current_yaw = math::QuaternionToYaw(aim_state_i.newest_imu.quat);
  double current_pitch = math::QuaternionToPitch(aim_state_i.newest_imu.quat);

  // 创建消息，并填写最基本的内容
  interfaces::NeGimbalControlRef_t gimbal_control_ref_o;
  gimbal_control_ref_o.cap_stamp = aim_state_i.cap_stamp;
  gimbal_control_ref_o.valid = false;
  gimbal_control_ref_o.armor_id = "NULL";

  // 回环传送，懂得都懂
  gimbal_control_ref_o.yaw_ref = current_yaw;
  gimbal_control_ref_o.pitch_ref = current_pitch;

  // 如果没有目标，就不规划了
  if (aim_state_i.has_target == false)
  {
    // 即使没了目标也必须发消息
    NV_CHANNELS.gimbal_control_ref_sPtr()->Transmit(gimbal_control_ref_o);
    return;
  };

  if (!aim_state_i.aim_predictor)
  {
    // 虽然是异常情况，也不要停止发消息
    NV_CHANNELS.gimbal_control_ref_sPtr()->Transmit(gimbal_control_ref_o);
    NV_WARN("Aim predictor is null, cannot plan");
    return;
  }

  // 1. 基础预测时间，就是 现在时间-最新IMU + 该任务执行时间 + 额外预测时间
  double imu_to_new_time =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    aim_state_i.aim_predictor->GetImuStamp())
          .count();
  double base_dt = imu_to_new_time + 0;

  // 2. 填时间
  gimbal_control_ref_o.predicted_stamp =
      aim_state_i.cap_stamp +
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(base_dt));

  // 3. 由于控制延迟，预测云台初始状态

  // 预测 base_dt 时间后的云台状态，该状态作为MPC规划器的初始状态
  double pred_gimbal_yaw =
      current_yaw + aim_state_i.newest_imu.gyro.z() * base_dt;
  double pred_gimbal_pitch =
      current_pitch + aim_state_i.newest_imu.gyro.y() * base_dt;

  /* === 正式走MPC流程 ==== */
  // 1. 填入初始云台pitch yaw 以及对应速度
  Vec_x_t x;
  x << pred_gimbal_yaw, aim_state_i.newest_imu.gyro.z(), pred_gimbal_pitch,
      aim_state_i.newest_imu.gyro.y();

  // // ----- 测试代码: 将初始状态强制设为上次规划的轨迹期望值 -----
  // static bool    has_last_planned_x = false;
  // static Vec_x_t last_planned_x;
  // if (has_last_planned_x)
  // {
  //   x = last_planned_x;
  //   pred_gimbal_yaw = x(0); // 同步给下面的 last_target_yaw 使用
  //   pred_gimbal_pitch = x(2);
  // }
  // ----------------------------------------------------

  // 2. 动态计算并填入此时的约束
  // TODO: 根据小陀螺状态动态计算约束
  tiny_set_bound_constraints(solver_ptr_,
                             mpc_mats_.min_x,
                             mpc_mats_.max_x,
                             mpc_mats_.min_u,
                             mpc_mats_.max_u);

  // 3. 迭代弹道补偿和预测，更新参考轨迹
  TinyWorkspace* work_ptr = solver_ptr_->work;

  // 用于微分角速度
  double last_target_yaw = pred_gimbal_yaw;
  double last_target_pitch = pred_gimbal_pitch;

  for (int i = 0; i < params_.horizon; ++i)
  {
    double extra_dt = base_dt + i * params_.step;

    Eigen::Array4d target_xzyyaw;
    target_xzyyaw.setZero();
    Eigen::Array4d target_angle_and_angular_v;
    target_angle_and_angular_v.setZero();
    if (!predictTargetPose(aim_state_i.aim_predictor,
                           extra_dt,
                           aim_state_i.newest_imu,
                           target_xzyyaw,
                           target_angle_and_angular_v))
    {
      NV_WARN("Failed to predict target pose for MPC compensation");
      // 无论是什么错误都别忘了发数据
      NV_CHANNELS.gimbal_control_ref_sPtr()->Transmit(gimbal_control_ref_o);
      return;
    }

    // 将第一次的预测结果保存下来，用来可视化选板
    if (i == 0)
      gimbal_control_ref_o.debug.target_armor_xyzy = target_xzyyaw;

    // 存储序列用于优化
    work_ptr->Xref(0, i) = target_angle_and_angular_v(0);
    work_ptr->Xref(1, i) = target_angle_and_angular_v(2);
    work_ptr->Xref(2, i) = target_angle_and_angular_v(1);
    work_ptr->Xref(3, i) = target_angle_and_angular_v(3);

    last_target_yaw = target_angle_and_angular_v(0);
    last_target_pitch = target_angle_and_angular_v(1);
  }

  // 发消息
  gimbal_control_ref_o.valid = true;
  NV_CHANNELS.gimbal_control_ref_sPtr()->Transmit(gimbal_control_ref_o);

  // // 4. 算
  // tiny_set_x0(solver_ptr_, x);
  // tiny_solve(solver_ptr_);

  // // 5. 导出规划结果轨迹
  // auto yaw_traj = work_ptr->x.row(0);
  // auto pitch_traj = work_ptr->x.row(2);
  // auto yaw_vel_traj = work_ptr->x.row(1);
  // auto pitch_vel_traj = work_ptr->x.row(3);

  // // 利用 Rerun 记录并展示规划结果与期望值之间的关系
  // std::vector<double> log_planned_yaw(params_.horizon);
  // std::vector<double> log_ref_yaw(params_.horizon);
  // for (int i = 0; i < params_.horizon; ++i)
  // {
  //   log_planned_yaw[i] = yaw_traj(i);
  //   log_ref_yaw[i] = work_ptr->Xref(0, i);
  // }

  // NeRerunDebug::GetInstance().EnableRealtimeDebug();
  // NV_REC_LOG(name_ + "/planned_yaw_traj",
  //            rerun::SeriesLines()
  //                .with_names("planned_yaw")
  //                .with_colors({{0, 255, 0}}));
  // NV_REC_LOG(name_ + "/planned_yaw_traj", rerun::Scalars(log_planned_yaw));
  // NV_REC_LOG(
  //     name_ + "/ref_yaw_traj",
  //     rerun::SeriesLines().with_names("ref_yaw").with_colors({{255, 0, 0}}));
  // NV_REC_LOG(name_ + "/ref_yaw_traj", rerun::Scalars(log_ref_yaw));

  // // ----- 测试代码: 记录规划后的下一个节点用于下次测试 -----
  // if (params_.horizon > 1)
  // {
  //   last_planned_x << yaw_traj(1), yaw_vel_traj(1), pitch_traj(1),
  //       pitch_vel_traj(1);
  //   has_last_planned_x = true;
  // }
  // // ------------------------------------------------------

  // // NV_REC_LOG(name_ + "/planned_yaw_traj", rerun::Scalars(yaw_traj.data(),
  // // params_.horizon));
  // double          yawyaw = 0;
  // double          pitchpitch = 0;
  // Eigen::Vector3d target_pos;
  // aim_state_i.aim_predictor->Predict(
  //     base_dt, aim_state_i.newest_imu, target_pos, yawyaw);
  // NV_REC_LOG(name_ + "/predicted_target_yaw", rerun::Scalars(yawyaw));
}

bool NeMashiroPlanner::predictTargetPose(
    const std::shared_ptr<interfaces::NeAimPredictorBase>& predictor,
    double                                                 extra_dt,
    const interfaces::NeImuData_t&                         imu_data,
    Eigen::Array4d&                                        target_xzyyaw_out,
    Eigen::Array4d& target_angle_and_angular_v_out)
{
  if (!predictor)
    return false;

  auto calculatePose = [&](double           delta_t,
                           Eigen::Vector3d& target_pos,
                           double&          yaw,
                           double&          pitch) -> bool {
    YUKINO::BallisticModel bm;
    double                 fly_time = 0.0;

    // 设置弹速
    // NJQ你全局指针几个意思啊
    if (Com_ptr_)
      Com_ptr_->muzzle_v = 20;

    for (int i = 0; i < 5; ++i)
    {
      Eigen::Vector3d target_vel;
      if (!predictor->Predict(
              delta_t + fly_time, imu_data, target_pos, yaw, target_vel))
        return false;

      double distance = std::hypot(target_pos.x(), target_pos.y());
      double height = target_pos.z();

      double cur_pitch = bm.Cal_TargetposPitch(distance, height, 0.0, 0.0);
      double new_fly_time = bm.get_ft(distance, cur_pitch);

      pitch = cur_pitch;

      if (std::abs(new_fly_time - fly_time) < 1e-3)
        break;

      fly_time = new_fly_time;
    }
    return true;
  };

  Eigen::Vector3d target_pos_1;
  double          yaw1 = 0.0;
  double          pitch1 = 0.0;
  if (!calculatePose(extra_dt, target_pos_1, yaw1, pitch1))
    return false;

  // 输出顺序: [x, z, y, yaw]
  target_xzyyaw_out << target_pos_1.x(), target_pos_1.z(), target_pos_1.y(),
      yaw1;

  double          epsilon = 0.001; // 1ms duration for derivative
  Eigen::Vector3d target_pos_2;
  double          yaw2 = 0.0;
  double          pitch2 = 0.0;
  if (!calculatePose(extra_dt + epsilon, target_pos_2, yaw2, pitch2))
    return false;

  const double yaw_v = math::WrapToPi(yaw2 - yaw1) / epsilon;
  const double pitch_v = math::WrapToPi(pitch2 - pitch1) / epsilon;

  // 输出顺序: [yaw, pitch, yaw_v, pitch_v]
  target_angle_and_angular_v_out << yaw1, pitch1, yaw_v, pitch_v;

  return true;
}

} // namespace ne_vision