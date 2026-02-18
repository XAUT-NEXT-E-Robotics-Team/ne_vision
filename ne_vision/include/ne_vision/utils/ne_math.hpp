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

namespace ne_vision
{

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
