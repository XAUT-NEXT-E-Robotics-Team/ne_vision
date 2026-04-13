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
// MPC 轨迹重规划，懂得都懂

#include <memory>

#include "tinympc/types.hpp"
#include "ne_vision/interfaces/types/ne_aim_predictor.hpp"
#include "ne_vision/interfaces/ne_imu_data.hpp"
#include "ne_vision/interfaces/ne_robot_state.hpp"
#include "ne_vision/ballistic_compensation/ballistic_slove.hpp"

namespace ne_vision
{

class NeMashiroPlanner final
{

private:
  constexpr static int kStateDim = 4;
  constexpr static int kControlDim = 2;
  using Mat_A_t = Eigen::Matrix<tinytype, kStateDim, kStateDim>;
  using Mat_B_t = Eigen::Matrix<tinytype, kStateDim, kControlDim>;
  using Vec_x_t = Eigen::Matrix<tinytype, kStateDim, 1>;
  using Vec_u_t = Eigen::Matrix<tinytype, kControlDim, 1>;

public:
  NeMashiroPlanner(std::string name);
  ~NeMashiroPlanner() = default;

  // 规划，任务调用这个
  void Plan();

  std::string GetName() const { return name_; }

private:
  // 迭代弹道补偿和预测
  // 用于根据预测时间dt，获取结果yaw pitch
  void predictTargetPose(
      const std::shared_ptr<interfaces::NeAimPredictorBase>& predictor,
      double                                                 extra_dt,
      const interfaces::NeImuData_t&                         imu_data,
      Eigen::Array4d&                                        target_xzyyaw_out,
      Eigen::Array4d& target_angle_and_angular_v_out);

  TinySolver* solver_ptr_;
  double      rho_value_ = 1.0;

  struct
  {
    tinyMatrix Adyn;
    tinyMatrix Bdyn;
    tinyMatrix fdyn;
    tinyMatrix Q;
    tinyMatrix R;

    // 这里的参数设定后是不会变的
    // 需要自适应的话，请增加变量
    tinyMatrix max_x;
    tinyMatrix min_x;
    tinyMatrix max_u;
    tinyMatrix min_u;
  } mpc_mats_;

  struct
  {
    // 预测补偿和步数：
    // 你需要根据实际情况去调，需要保证MPC能比较好的优化跳变
    //
    // 基本上，对于响应比较好的云台，预测步数可以小一点，
    // 因为他们规划轨迹更贴近于实际轨迹（不太需要能够早早看到跳变点）
    //
    // 反之，如果云台响应比较慢，预测步数可以大一点，能够提前看到跳变点，提前规划出补偿轨迹

    double step = 0.01;  // 时间步长，单位秒
    int    horizon = 10; // MPC 预测步数

    // 弹道补偿迭代次数，基本上时间都打差不差，建议不要设置太大
    int predict_compensation_iterations = 5;

    // 额外预测时间
    double additional_predict_time = 0.0;
  } params_;

  interfaces::NeRobotState_t robot_state_i_;

  YUKINO::BallisticModel bm_;

  std::string name_;
};

} // namespace ne_vision