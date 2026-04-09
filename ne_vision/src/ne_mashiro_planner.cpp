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

#include <cmath>

#include "ne_vision/planner/ne_mashiro_planner.hpp"
#include "ne_vision/interfaces/ne_aim_traj.hpp"
#include "ne_vision/utils/ne_debug.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/ne_channals.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/utils/ne_math.hpp"
#include "tinympc/tiny_api.hpp"
#include "ne_vision/ballistic_compensation/ballistic_slove.hpp"

namespace ne_vision
{

NeMashiroPlanner::NeMashiroPlanner()
{
  // 读取参数
  auto   param_node = NV_PARAM["auto_aim"]["mashiro_planner"];
  double q_y_pos = param_node["q_y_pos"].as<double>(100.0);
  double q_y_vel = param_node["q_y_vel"].as<double>(1.0);
  double q_p_pos = param_node["q_p_pos"].as<double>(100.0);
  double q_p_vel = param_node["q_p_vel"].as<double>(1.0);
  double r_y_acc = param_node["r_y_acc"].as<double>(10.0);
  double r_p_acc = param_node["r_p_acc"].as<double>(10.0);

  // 设置几个矩阵
  mpc_mats_.Adyn.setIdentity();
  mpc_mats_.Adyn(0, 1) = params_.step;
  mpc_mats_.Adyn(2, 3) = params_.step;

  mpc_mats_.Bdyn.setZero();
  mpc_mats_.Bdyn(0, 0) = 0.5 * params_.step * params_.step;
  mpc_mats_.Bdyn(1, 0) = params_.step;
  mpc_mats_.Bdyn(2, 1) = 0.5 * params_.step * params_.step;
  mpc_mats_.Bdyn(3, 1) = params_.step;

  mpc_mats_.fdyn.setZero();

  mpc_mats_.Q.setZero();
  mpc_mats_.Q(0, 0) = q_y_pos;
  mpc_mats_.Q(1, 1) = q_y_vel;
  mpc_mats_.Q(2, 2) = q_p_pos;
  mpc_mats_.Q(3, 3) = q_p_vel;

  mpc_mats_.R.setZero();
  mpc_mats_.R(0, 0) = r_y_acc;
  mpc_mats_.R(1, 1) = r_p_acc;

  // 初始化求解器
  if (!tiny_setup(&solver_ptr_,
                  mpc_mats_.Adyn,
                  mpc_mats_.Bdyn,
                  mpc_mats_.fdyn,
                  mpc_mats_.Q,
                  mpc_mats_.R,
                  rho_value_,
                  kStateDim,
                  kControlDim,
                  kHorizon,
                  1))
  {
    NV_ASSERT(0 && "Failed to Set Up TinyMPC Solver");
  }
}

void NeMashiroPlanner::Plan()
{
  interfaces::NeAimTraj_t aim_traj_i;

  if (!NV_CHANNELS.aim_traj_sPtr()->Receive(aim_traj_i))
  {
    NV_WARN("Aim trajectory channel is not available, cannot plan");
    return;
  }

  // 如果没有目标，就不规划了
  if (aim_traj_i.has_target == false) {};

  // 1. 基础预测时间，就是 现在时间-最新IMU + 该任务执行时间 + 额外预测时间
  double imu_to_new_time =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    aim_traj_i.aim_predictor->GetImuStamp())
          .count();
  double base_dt = imu_to_new_time + 0;

  // 2. 由于控制延迟，预测云台状态
  double current_yaw = math::QuaternionToYaw(aim_traj_i.newest_imu.quat);
  double current_pitch = math::QuaternionToPitch(aim_traj_i.newest_imu.quat);

  // 预测 base_dt
  double pred_gimbal_yaw =
      current_yaw + aim_traj_i.newest_imu.gyro.z() * base_dt;
  double pred_gimbal_pitch =
      current_pitch + aim_traj_i.newest_imu.gyro.y() * base_dt;

  // 开始MPC流程
}

bool NeMashiroPlanner::predictTargetPose(
    const std::shared_ptr<interfaces::NeAimPredictorBase>& predictor,
    double                                                 extra_dt,
    const interfaces::NeImuData_t&                         imu_data,
    double&                                                target_yaw_out,
    double&                                                target_pitch_out)
{
  if (!predictor)
    return false;

  YUKINO::BallisticModel bm;
  double                 fly_time = 0.0;
  Eigen::Vector3d        target_pos;
  double                 target_yaw = 0.0;
  double                 pitch_out = 0.0;

  // 由于弹道补偿的飞行时间是预测目标位置的函数，因此需要迭代求解。
  for (int i = 0; i < MAX_ITERATION_COUNT; i++)
  {
    if (!predictor->Predict(
            extra_dt + fly_time, imu_data, target_pos, target_yaw))
      return false;

    // 水平距离
    double distance = std::hypot(target_pos.x(), target_pos.y());
    double height = target_pos.z();

    double cur_pitch = bm.Cal_TargetposPitch(distance, height, 0.0, 0.0);
    double new_fly_time = bm.get_ft(distance, cur_pitch);

    pitch_out = cur_pitch;

    if (std::abs(new_fly_time - fly_time) < 1e-3)
    {
      break;
    }
    fly_time = new_fly_time;
  }

  target_yaw_out = target_yaw;
  target_pitch_out = pitch_out;

  return true;
}

} // namespace ne_vision