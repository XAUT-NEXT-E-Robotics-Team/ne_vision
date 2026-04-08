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
  // 采样周期是必须的，所以该任务务必设置为定周期执行
  NeMashiroPlanner(double dt);
  ~NeMashiroPlanner() = default;

  // 规划，任务调用这个
  bool Plan();

private:
  // 弹道补偿
  void ballisticCompensation(const Vec_x_t& x0,
                             const Vec_x_t& target,
                             Vec_x_t&       x0_comp);

  double      dt_ = 0.01; // 时间步长，单位秒
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
};

} // namespace ne_vision