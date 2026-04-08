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
#include "ne_vision/utils/ne_debug.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include "tinympc/tiny_api.hpp"

namespace ne_vision
{

NeMashiroPlanner::NeMashiroPlanner(double dt) : dt_(dt)
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
  mpc_mats_.Adyn(0, 1) = dt_;
  mpc_mats_.Adyn(2, 3) = dt_;

  mpc_mats_.Bdyn.setZero();
  mpc_mats_.Bdyn(0, 0) = 0.5 * dt_ * dt_;
  mpc_mats_.Bdyn(1, 0) = dt_;
  mpc_mats_.Bdyn(2, 1) = 0.5 * dt_ * dt_;
  mpc_mats_.Bdyn(3, 1) = dt_;

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

} // namespace ne_vision