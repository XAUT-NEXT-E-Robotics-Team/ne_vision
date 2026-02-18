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
// By AI

#pragma once

#include <memory>
#include <string>
#include <concepts>

#include "spdlog/fmt/fmt.h"
#include "spdlog/spdlog.h"

#include "ne_vision/utils/ne_def.hpp"

namespace ne_vision
{

class Logger
{
public:
  static spdlog::logger& getInstance()
  {
    static Logger instance;
    return *instance.logger_;
  }

  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

private:
  Logger();
  ~Logger() { spdlog::shutdown(); }

  std::shared_ptr<spdlog::logger> logger_;
};

// This function is defined as weak to allow users to provide their own
// implementation if they wish.
void NvWeakLogTrace(const std::string& msg);

void NvWeakLogDebug(const std::string& msg);

void NvWeakLogInfo(const std::string& msg);

void NvWeakLogWarn(const std::string& msg);

void NvWeakLogError(const std::string& msg);

// Fallback GetName for contexts without one
inline std::string GetName() { return ""; }

// This is detail macro DON NOT use it directly. It will automatically add
// context prefix if the class has GetName
#define NV_GET_CONTEXT_PREFIX()                                                \
  [&]() -> std::string {                                                       \
    using namespace ne_vision; /* Allow finding ne_vision::GetName fallback */ \
    if constexpr (requires {                                                   \
                    { GetName() } -> std::same_as<std::string>;                \
                  })                                                           \
    {                                                                          \
      std::string n = GetName();                                               \
      if (n.empty()) return "";                                                \
      return "[" + n + "] ";                                                   \
    }                                                                          \
    else                                                                       \
    {                                                                          \
      return "";                                                               \
    }                                                                          \
  }()

// You should to use these macros for logging.
// The logging macros will automatically add context prefix if the class has
// GetName()
#define NV_TRACE(...)                                                          \
  do                                                                           \
  {                                                                            \
    auto prefix = NV_GET_CONTEXT_PREFIX();                                     \
    ne_vision::NvWeakLogTrace(prefix + fmt::format(__VA_ARGS__));              \
  } while (0)

#define NV_DEBUG(...)                                                          \
  do                                                                           \
  {                                                                            \
    auto prefix = NV_GET_CONTEXT_PREFIX();                                     \
    ne_vision::NvWeakLogDebug(prefix + fmt::format(__VA_ARGS__));              \
  } while (0)

#define NV_INFO(...)                                                           \
  do                                                                           \
  {                                                                            \
    auto prefix = NV_GET_CONTEXT_PREFIX();                                     \
    ne_vision::NvWeakLogInfo(prefix + fmt::format(__VA_ARGS__));               \
  } while (0)

#define NV_WARN(...)                                                           \
  do                                                                           \
  {                                                                            \
    auto prefix = NV_GET_CONTEXT_PREFIX();                                     \
    ne_vision::NvWeakLogWarn(prefix + fmt::format(__VA_ARGS__));               \
  } while (0)

#define NV_ERROR(...)                                                          \
  do                                                                           \
  {                                                                            \
    auto prefix = NV_GET_CONTEXT_PREFIX();                                     \
    ne_vision::NvWeakLogError(prefix + fmt::format(__VA_ARGS__));              \
  } while (0)

} // namespace ne_vision
