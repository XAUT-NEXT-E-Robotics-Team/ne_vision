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

#include "ne_vision/utils/ne_log.hpp"
#include "spdlog/sinks/stdout_color_sinks.h"
#include <iostream>

namespace ne_vision
{

Logger::Logger()
{
  try
  {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    console_sink->set_level(spdlog::level::trace);
    console_sink->set_pattern("[%H:%M:%S.%e] [%n] [%^%l%$] %v");

    logger_ = std::make_shared<spdlog::logger>("NE_VISION", console_sink);

    logger_->set_level(spdlog::level::trace);

    logger_->flush_on(spdlog::level::err);
  }
  catch (const spdlog::spdlog_ex& ex)
  {
    printf("Logger initialization failed: %s\n", ex.what());
  }
}

NV_WEAK_CPP
void NvWeakLogTrace(const std::string& msg) { Logger::getInstance().trace(msg); }

NV_WEAK_CPP
void NvWeakLogDebug(const std::string& msg) { Logger::getInstance().debug(msg); }

NV_WEAK_CPP
void NvWeakLogInfo(const std::string& msg) { Logger::getInstance().info(msg); }

NV_WEAK_CPP
void NvWeakLogWarn(const std::string& msg) { Logger::getInstance().warn(msg); }

NV_WEAK_CPP
void NvWeakLogError(const std::string& msg) { Logger::getInstance().error(msg); }

} // namespace ne_vision
