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
// We have all known, that cv::Mat uses shallow copy. It is not good for
// multi-threading. If you try to copy a cv::Mat from a channel and you try to
// change it, it will change the original cv::Mat. It could be VERY DANGEROUS.
//
// So we wrap cv::Mat with a deep copy version.
//

#pragma once

#include "opencv2/core/mat.hpp"

namespace ne_vision
{

// cv::Mat couldn't be inherited, because it doesn't support virtual destructor.
class NeDMat final
{
public:
  NeDMat() = default;

  NeDMat(const cv::Mat& src)
  {
    if (!src.empty())
      mat_ = src.clone(); // We should insure to deep copy the mat
  }

  /* === Copy Semantics === */

  // Copy constructor
  NeDMat(const NeDMat& other)
  {
    if (!other.mat_.empty())
      mat_ = other.mat_.clone();
  }

  // Copy overload
  NeDMat& operator=(const NeDMat& other)
  {
    if (this != &other)
    {
      if (!other.mat_.empty())
        mat_ = other.mat_.clone();
      else
        mat_.release();
    }
    return *this;
  }

  /* === Move Semantics === */

  // Move constructor
  NeDMat(NeDMat&& other) noexcept : mat_(std::move(other.mat_)) {}

  // Move overload
  NeDMat& operator=(NeDMat&& other) noexcept
  {
    if (this != &other)
      mat_ = std::move(other.mat_);
    return *this;
  }

  /* === Utility Functions === */

  operator cv::Mat() const { return mat_.empty() ? cv::Mat() : mat_.clone(); }

  bool     Empty() const { return mat_.empty(); }
  cv::Size Size() const { return mat_.size(); }
  int      Type() const { return mat_.type(); }

private:
  cv::Mat mat_;
};

} // namespace ne_vision
