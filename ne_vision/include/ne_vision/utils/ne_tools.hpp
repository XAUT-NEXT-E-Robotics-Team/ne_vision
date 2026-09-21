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
// 一些小工具
// 啥都有，很杂

#pragma once

#include <tuple>
#include <utility>

namespace ne_vision
{

// 外部应用不要直接调用detail的东西j
namespace detail
{

// 遍历tuple
// 传入Func应该是[](auto){};
template <typename Tuple, typename Func>
inline void NeTraverseTuple(Tuple&& tuple, Func&& f)
{
  std::apply(
      [&](auto&&... elems) { (f(std::forward<decltype(elems)>(elems)), ...); },
      std::forward<Tuple>(tuple));
}

} // namespace detail

// 帮你从robot_id中判断类型
inline std::string GetArmorTypeFromId(std::string robot_id)
{
  if (robot_id == "outpost")
    return "outpost";
  else if (robot_id == "base" || robot_id == "1")
    return "large";
  else
    return "small";
}

// 帮你从robot_id中得到pitch角度
inline double GetPitchFromId(std::string robot_id)
{
  if (robot_id == "outpost")
    return 0.0;
  else if (robot_id == "base" || robot_id == "1")
    return 0.0;
  else
    return 0.0;
}

} // namespace ne_vision
