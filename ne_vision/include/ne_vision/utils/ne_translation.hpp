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
// 齐次变换的快速运算
// 懂得都懂

#pragma once

#include "Eigen/Dense"

namespace ne_vision
{

class NeTranslation
{

public:
  // 默认构造函数：平移为0，旋转为单位四元数
  NeTranslation()
      : t_(Eigen::Vector3d::Zero()), q_(Eigen::Quaterniond::Identity())
  {
  }
  // 带参构造函数
  NeTranslation(const Eigen::Vector3d& t, const Eigen::Quaterniond& q)
      : t_(t), q_(q)
  {
  }

  inline Eigen::Vector3d&          t() { return t_; }
  inline const Eigen::Vector3d&    t() const { return t_; }
  inline Eigen::Quaterniond&       q() { return q_; }
  inline const Eigen::Quaterniond& q() const { return q_; }

  // 乘法
  NeTranslation operator*(const NeTranslation& rhs) const
  {
    NeTranslation res;
    // 旋转：q1 * q2
    res.q_ = this->q_ * rhs.q_;
    // 平移：q1 * t2 + t1
    res.t_ = this->q_ * rhs.t_ + this->t_;
    return res;
  }

  // *=
  NeTranslation& operator*=(const NeTranslation& rhs)
  {
    // 注意顺序
    this->t_ = this->q_ * rhs.t_ + this->t_;
    this->q_ = this->q_ * rhs.q_;
    return *this;
  }

  // 对坐标乘法
  Eigen::Vector3d operator*(const Eigen::Vector3d& rhs) const
  {
    return this->q_ * rhs + this->t_;
  }

  // 求inv
  NeTranslation Inverse() const
  {
    // 不要直接逆
    NeTranslation res;
    res.q_ = this->q_.conjugate();
    res.t_ = -(res.q_ * this->t_);
    return res;
  }

private:
  Eigen::Vector3d    t_;
  Eigen::Quaterniond q_;
};

} // namespace ne_vision