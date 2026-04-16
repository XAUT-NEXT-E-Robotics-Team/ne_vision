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

#include <chrono>
#include <cmath>
#include <vector>

#include "ne_vision/ballistic_compensation/model_param.hpp"
#include "ne_vision/interfaces/ne_imu_data.hpp"
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
  // 读取弹丸类型并新建弹道补偿
  auto b_type =
      NV_PARAM["hardware"]["muzzle"]["ball_type"].as<std::string>("small");
  if (b_type != "small" && b_type != "large")
  {
    NV_ERROR("Invalid ball type in config: %s. Must be 'small' or 'large'.",
             b_type.c_str());
    throw std::runtime_error("Invalid ball type in config");
  }
  else
  {
    NV_INFO("Ball type: %s", b_type.c_str());
    Com_ptr_->ball_type = b_type;
  }
  bm_.judgeK1();

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

  // gimbal到muzzle的x轴偏移
  params_.muzzle_x_offset =
      NV_PARAM["hardware"]["muzzle"]["gimbal_to_muzzle"]["t"]["x"].as<double>(
          0.0);

  // 额外预测时间
  params_.additional_predict_time =
      param_node["additional_predict_time"].as<double>(0.0);

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

  // 读取IMU并转换
  interfaces::NeImuData_t current_imu;
  if (!NV_CHANNELS.imu_data_sPtr()->Receive(current_imu))
  {
    NV_WARN("Waiting for IMU data...");
    return;
  }

  // 创建消息，并填写最基本的内容
  interfaces::NeGimbalControlRef_t gimbal_control_ref_o;
  gimbal_control_ref_o.cap_stamp = aim_state_i.cap_stamp;
  gimbal_control_ref_o.valid = false;
  gimbal_control_ref_o.armor_id = "NULL";

  // 确保安全，实则无用
  gimbal_control_ref_o.yaw_ref = 0;
  gimbal_control_ref_o.pitch_ref = 0;

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

  // 初始化即进行进行预测，将时间从cap_stamp推到最新IMU时间（不是当前时间）
  aim_state_i.aim_predictor->Init();

  // 1. 基础预测时间，就是 现在时间-最新IMU + 该任务执行时间 + 额外预测时间
  // TODO: 该任务执行时间是否需要考虑
  double imu_to_new_time =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    aim_state_i.aim_predictor->GetImuStamp())
          .count();
  double base_dt = imu_to_new_time + params_.additional_predict_time;

  // 2. 填时间
  // gimbal_control_ref_o.gimbal_predicted_stamp =
  //     aim_state_i.aim_predictor->GetImuStamp() +
  //     std::chrono::duration_cast<std::chrono::milliseconds>(
  //         std::chrono::duration<double>(params_.additional_predict_time));
  // 控制时间为有IMU预测时间（IMU最新当前时间）+ 额外时间  +
  // 一个步长（MPC轨迹的最近值）
  gimbal_control_ref_o.control_stamp =
      aim_state_i.aim_predictor->GetImuStamp() +
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(base_dt + params_.step));

  // 3. 由于控制延迟，预测云台初始状态

  // 预测 additional_predict_time
  // 时间后的云台状态，该状态作为MPC规划器的初始状态。
  interfaces::NeImuData_t predicted_imu = current_imu;
  predicted_imu.quat =
      predicted_imu.quat *
      math::So3Exp(current_imu.gyro * params_.additional_predict_time);
  double pred_gimbal_yaw = math::QuaternionToYaw(predicted_imu.quat);
  double pred_gimbal_pitch = math::QuaternionToPitch(predicted_imu.quat);
  // 预测结果存为调试
  gimbal_control_ref_o.debug.predicted_imu_data = predicted_imu;

  /* === 正式走MPC流程 ==== */
  // 1. 填入初始云台pitch yaw 以及对应速度
  Vec_x_t x;
  x << pred_gimbal_yaw, current_imu.gyro.z(), pred_gimbal_pitch,
      current_imu.gyro.y();

  // 2. 动态计算并填入此时的约束
  // TODO: 根据小陀螺状态动态计算约束
  tiny_set_bound_constraints(solver_ptr_,
                             mpc_mats_.min_x,
                             mpc_mats_.max_x,
                             mpc_mats_.min_u,
                             mpc_mats_.max_u);

  // 3. 迭代弹道补偿和预测，更新参考轨迹
  TinyWorkspace* work_ptr = solver_ptr_->work;

  for (int i = 0; i < params_.horizon; ++i)
  {
    double extra_dt = base_dt + i * params_.step;

    Eigen::Array4d target_xzyyaw;
    target_xzyyaw.setZero();
    Eigen::Array4d target_angle_and_angular_v;
    target_angle_and_angular_v.setZero();
    predictTargetPose(aim_state_i.aim_predictor,
                      extra_dt,
                      current_imu,
                      target_xzyyaw,
                      target_angle_and_angular_v);

    // 将第一次的预测结果保存下来，用来可视化选板
    if (i == 0)
    {
      gimbal_control_ref_o.debug.target_armor_xyzy = target_xzyyaw;
      gimbal_control_ref_o.debug.aim_yaw = target_angle_and_angular_v(0);
      gimbal_control_ref_o.debug.aim_pitch = target_angle_and_angular_v(1);
      gimbal_control_ref_o.debug.aim_yaw_v = target_angle_and_angular_v(2);
      gimbal_control_ref_o.debug.aim_pitch_v = target_angle_and_angular_v(3);
    }

    // 存储序列用于优化
    // 注意顺序
    work_ptr->Xref(0, i) = target_angle_and_angular_v(0);
    work_ptr->Xref(1, i) = target_angle_and_angular_v(2);
    work_ptr->Xref(2, i) = target_angle_and_angular_v(1);
    work_ptr->Xref(3, i) = target_angle_and_angular_v(3);
  }

  // 4. 算
  tiny_set_x0(solver_ptr_, x);
  tiny_solve(solver_ptr_);

  // 5. 填入结果
  // gimbal_control_ref_o.yaw_ref = work_ptr->x(0, 1);
  // gimbal_control_ref_o.yaw_v_ref = work_ptr->x(1, 1);
  // gimbal_control_ref_o.pitch_ref = work_ptr->x(2, 1);
  // gimbal_control_ref_o.pitch_v_ref = work_ptr->x(3, 1);

  gimbal_control_ref_o.yaw_ref = gimbal_control_ref_o.debug.aim_yaw;
  gimbal_control_ref_o.yaw_v_ref = gimbal_control_ref_o.debug.aim_yaw_v;
  gimbal_control_ref_o.pitch_ref = gimbal_control_ref_o.debug.aim_pitch;
  gimbal_control_ref_o.pitch_v_ref = gimbal_control_ref_o.debug.aim_pitch_v;

  gimbal_control_ref_o.debug.yaw_local_traj.clear();
  gimbal_control_ref_o.debug.pitch_local_traj.clear();
  for (int i = 0; i < params_.horizon; ++i)
  {
    gimbal_control_ref_o.debug.yaw_local_traj.push_back(work_ptr->x(0, i));
    gimbal_control_ref_o.debug.pitch_local_traj.push_back(work_ptr->x(2, i));
  }

  // 发消息
  gimbal_control_ref_o.valid = true;
  NV_CHANNELS.gimbal_control_ref_sPtr()->Transmit(gimbal_control_ref_o);
}

void NeMashiroPlanner::predictTargetPose(
    const std::shared_ptr<interfaces::NeAimPredictorBase>& predictor,
    double                                                 extra_dt,
    const interfaces::NeImuData_t&                         imu_data,
    Eigen::Array4d&                                        target_xzyyaw_out,
    Eigen::Array4d& target_angle_and_angular_v_out)
{
  if (!predictor)
    return;

  auto calculatePose = [&](double           delta_t,
                           Eigen::Vector3d& target_pos,
                           Eigen::Vector3d& target_vel,
                           double&          yaw,
                           double&          pitch) {
    double fly_time = 0.0;

    // 设置弹速
    // NJQ你全局指针几个意思啊
    NV_ASSERT(Com_ptr_ && "Com_ptr_ is null, cannot set bullet speed");
    Com_ptr_->muzzle_v = robot_state_i_.bullet_speed;

    for (int i = 0; i < 5; ++i)
    {
      predictor->Predict(
          delta_t + fly_time, imu_data, target_pos, yaw, target_vel);

      double distance = std::hypot(target_pos.x(), target_pos.y());
      double height = target_pos.z();

      double cur_pitch = bm_.Cal_TargetposPitch(
          distance, height, params_.muzzle_x_offset, 0.0);
      double new_fly_time = bm_.get_ft(distance, cur_pitch);

      pitch = cur_pitch;

      if (std::abs(new_fly_time - fly_time) < 1e-3)
        break;

      fly_time = new_fly_time;
    }
  };

  Eigen::Vector3d target_pos;
  Eigen::Vector3d target_vel;
  double          yaw = 0.0;
  double          pitch = 0.0;
  calculatePose(extra_dt, target_pos, target_vel, yaw, pitch);

  // 输出顺序: [x, z, y, yaw]
  target_xzyyaw_out << target_pos.x(), target_pos.z(), target_pos.y(), yaw;

  double distance2 =
      target_pos.x() * target_pos.x() + target_pos.y() * target_pos.y();
  double distance = std::sqrt(distance2);

  double yaw_v = 0.0;
  double pitch_v = 0.0;
  if (distance2 > 1e-5)
  {
    yaw_v =
        (target_pos.x() * target_vel.y() - target_pos.y() * target_vel.x()) /
        distance2;
    double distance3d2 = distance2 + target_pos.z() * target_pos.z();
    double distance_v =
        (target_pos.x() * target_vel.x() + target_pos.y() * target_vel.y()) /
        distance;
    pitch_v =
        (distance * target_vel.z() - target_pos.z() * distance_v) / distance3d2;
  }

  // 输出顺序: [yaw, pitch, yaw_v, pitch_v]
  target_angle_and_angular_v_out << yaw, pitch, yaw_v, pitch_v;
}

} // namespace ne_vision