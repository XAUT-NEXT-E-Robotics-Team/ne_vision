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
// 2D Kalman Filter for tracking
//
// State: [x, vx, y, vy]
// Measurement: [x, y]
// Process model modle: sigma_x .* [ 1/4 * T^4, 1/2 * T^3 ]
//                                 [ 1/2 * T^3,       T^2 ]
//                                 Qk=Γ⋅σa2⋅Γ^T
// Measurement model: sigma_x .* [ 1, 0 ]
//                               [ 0, 1 ]

#pragma once

#include "Eigen/Dense"

#include "ne_vision/utils/ne_log.hpp"

namespace ne_vision
{
namespace kf
{

class NeTracker2DKf
{
private:
  using Z_t = Eigen::Vector2d;
  using X_t = Eigen::Vector4d;
  using A_t = Eigen::Matrix4d;
  using H_t = Eigen::Matrix<double, 2, 4>;
  using Q_t = Eigen::Matrix4d;
  using R_t = Eigen::Matrix2d;
  using P_t = Eigen::Matrix4d;
  using K_t = Eigen::Matrix<double, 4, 2>;

public:
  explicit NeTracker2DKf(double sigma_q_x,
                         double sigma_q_y,
                         double sigma_r_x,
                         double sigma_r_y)
      : sigma_q_x_(sigma_q_x), sigma_q_y_(sigma_q_y), sigma_r_x_(sigma_r_x),
        sigma_r_y_(sigma_r_y)
  {
    // clang-format off
    H_ << 1, 0, 0, 0,
          0, 0, 1, 0;
    // clang-format on
    Reset();

    NV_DEBUG("NeTracker2DKf created with sigma_q_x: {}, sigma_q_y: {}, "
             "sigma_r_x: {}, sigma_r_y: {}",
             sigma_q_x_,
             sigma_q_y_,
             sigma_r_x_,
             sigma_r_y_);
  }
  ~NeTracker2DKf() = default;

  inline void Init(const Z_t& z) { x_post_ << z(0), 0, z(1), 0; }

  inline void Reset()
  {
    p_pred_ = P_t::Identity();
    p_post_ = P_t::Identity();
    x_pred_ = X_t::Zero();
    x_post_ = X_t::Zero();
  }

  X_t Predict(double dt)
  {
    Q_t Q;
    A_t A;
    // clang-format off
    A << 1, dt,  0,  0,
         0,  1,  0,  0,
         0,  0,  1, dt,
         0,  0,  0,  1;
    Q << pow(dt, 4) / 4.0 * sigma_q_x_, pow(dt, 3) / 2.0 * sigma_q_x_, 0, 0,
         pow(dt, 3) / 2.0 * sigma_q_x_, pow(dt, 2) / 1.0 * sigma_q_x_, 0, 0,
         0, 0, pow(dt, 4) / 4.0 * sigma_q_y_, pow(dt, 3) / 2.0 * sigma_q_y_,
         0, 0, pow(dt, 3) / 2.0 * sigma_q_y_, pow(dt, 2) / 1.0 * sigma_q_y_;
    // clang-format on

    x_pred_ = A * x_post_;
    p_pred_ = A * p_post_ * A.transpose() + Q;

    return x_pred_;
  }

  X_t Update(const Z_t& z)
  {
    R_t R;
    // clang-format off
    R << sigma_r_x_, 0,
         0, sigma_r_y_;
    // clang-format on

    K_t K = p_pred_ * H_.transpose() *
            (H_ * p_pred_ * H_.transpose() + R).inverse();
    x_post_ = x_pred_ + K * (z - H_ * x_pred_);
    p_post_ = (P_t::Identity() - K * H_) * p_pred_;

    return x_post_;
  }

  inline Eigen::Vector2d GetPrePos()
  {
    return Eigen::Vector2d(x_pred_(0), x_pred_(2));
  }
  X_t& StatePost() { return x_post_; }

private:
  double sigma_q_x_;
  double sigma_q_y_;
  double sigma_r_x_;
  double sigma_r_y_;

  double dt_;

  P_t p_pred_;
  P_t p_post_;
  X_t x_pred_;
  X_t x_post_;

  H_t H_;
};
} // namespace kf
} // namespace ne_vision
