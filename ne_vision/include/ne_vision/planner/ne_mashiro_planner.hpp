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
#include "ne_vision/ballistic_compensation/ballistic_slove.hpp"

namespace ne_vision
{

class NeMashiroPlanner final
{

private:
  constexpr static int kStateDim = 4;
  constexpr static int kControlDim = 2;
  constexpr static int kHorizon = 10;
  using Mat_A_t = Eigen::Matrix<tinytype, kStateDim, kStateDim>;
  using Mat_B_t = Eigen::Matrix<tinytype, kStateDim, kControlDim>;
  using Vec_x_t = Eigen::Matrix<tinytype, kStateDim, 1>;

public:
  NeMashiroPlanner();
  ~NeMashiroPlanner() = default;

  // 根据预测器和额外预测时间计算带弹道补偿的云台目标 yaw 和 pitch
  bool predictTargetPose(
      const std::shared_ptr<interfaces::NeAimPredictorBase>& predictor,
      double                                                 extra_dt,
      const interfaces::NeImuData_t&                         imu_data,
      double&                                                target_yaw_out,
      double&                                                target_pitch_out);

  // 规划，任务调用这个
  void Plan();

private:
  // 弹道补偿
  void ballisticCompensation(const Vec_x_t& x0,
                             const Vec_x_t& target,
                             Vec_x_t&       x0_comp);

  TinySolver* solver_ptr_;
  double      rho_value_ = 1.0;

  struct
  {
    tinyMatrix Adyn;
    tinyMatrix Bdyn;
    tinyMatrix fdyn;
    tinyMatrix Q;
    tinyMatrix R;
  } mpc_mats_;

  struct
  {
    double step = 0.001; // 时间步长，单位秒

  } params_;
};

} // namespace ne_vision