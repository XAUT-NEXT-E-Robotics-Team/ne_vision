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
// Math

#pragma once

#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"

namespace ne_vision
{

namespace math
{

// rvec opencv 转 四元数 Eigen
inline void RvecToQuaternion(const cv::Mat& rvec, Eigen::Quaterniond& q)
{
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d R_eigen;
  cv::cv2eigen(R, R_eigen);
  q = Eigen::Quaterniond(R_eigen);
}

inline auto CvPoint3dToEigen(const cv::Point3d& pt)
{
  return Eigen::Vector3d(pt.x, pt.y, pt.z);
}

inline auto EigenVec2dToCv(const Eigen::Vector2d& vec)
{
  return cv::Point2d(vec.x(), vec.y());
}

inline auto CvPoint2dToEigen(const cv::Point2d& pt)
{
  return Eigen::Vector2d(pt.x, pt.y);
}

inline auto CvMat33ToEigen(const cv::Mat& mat)
{
  Eigen::Matrix3d eigen_mat;
  cv::cv2eigen(mat, eigen_mat);
  return eigen_mat;
}

inline double QuaternionToYaw(const Eigen::Quaterniond& q)
{
  // 计算yaw角
  // 旋转矩阵的(2,1)
  double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  // 旋转矩阵的(1,1)
  double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

inline double DegToRad(double deg) { return deg * M_PI / 180.0; }

inline double RadToDeg(double rad) { return rad * 180.0 / M_PI; }

inline double WrapToPi(double rad)
{
  return std::fmod(rad + M_PI, 2 * M_PI) - M_PI;
}

} // namespace math

// Start 0 and end num-1
// Like 0, 1, 2.
// This class has not advanture function.
// It works as long as it can be used.
template <int num_T>
class NePeriodicNumber
{

public:
  NePeriodicNumber() : num_(0)
  {
    static_assert(num_T > 0, "num_T should be positive");
  }
  NePeriodicNumber(int num) : num_(normalize(num))
  {
    static_assert(num_T > 0, "num_T should be positive");
  }
  ~NePeriodicNumber() = default;

  NePeriodicNumber& operator=(int num)
  {
    num_ = normalize(num);
    return *this;
  }

  operator int() const { return num_; }

  NePeriodicNumber operator+(int num) const
  {
    return NePeriodicNumber(normalize(num_ + num));
  }

  NePeriodicNumber operator-(int num) const
  {
    return NePeriodicNumber(normalize(num_ - num + num_T));
  }

  NePeriodicNumber& operator+=(int num)
  {
    num_ = normalize(num_ + num);
    return *this;
  }

  NePeriodicNumber& operator-=(int num)
  {
    num_ = normalize(num_ - num + num_T);
    return *this;
  }

  NePeriodicNumber& operator++()
  {
    num_ = normalize(num_ + 1);
    return *this;
  }

  NePeriodicNumber operator++(int)
  {
    NePeriodicNumber temp = *this;
    ++(*this);
    return temp;
  }

  NePeriodicNumber& operator--()
  {
    num_ = normalize(num_ - 1 + num_T);
    return *this;
  }

  NePeriodicNumber operator--(int)
  {
    NePeriodicNumber temp = *this;
    --(*this);
    return temp;
  }

private:
  int normalize(int val) const
  {
    int res = val % num_T;
    return (res < 0) ? (res + num_T) : res;
  }
  int num_;
};

} // namespace ne_vision
