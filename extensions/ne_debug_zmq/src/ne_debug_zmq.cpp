// ###########################################################
// ##                                                       ##
// ##                        .                 .:-:         ##
// ##                       :-:              :-::           ##
// ##                      -----          .:---.            ##
// ##                    .-------.     .:-----:             ##
// ##                   :---------. .:-------.              ##
// ##                  :--------------------.               ##
// ##                ---------------------                  ##
// ##               .-------:. :---------:                  ##
// ##              :-----:.     .-------.                   ##
// ##             .:---:         .-----.                    ##
// ##            .:-:.             :-:                      ##
// ##          .-:.                 .                       ##
// ##         .:                                            ##
// ##                                                       ##
// ##    ███╗   ██╗███████╗██╗  ██╗████████╗    ███████╗    ##
// ##    ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝    ██╔════╝    ##
// ##    ██╔██╗ ██║█████╗   ╚███╔╝    ██║       █████╗      ##
// ##    ██║╚██╗██║██╔══╝   ██╔██╗    ██║       ██╔══╝      ##
// ##    ██║ ╚████║███████╗██╔╝ ██╗   ██║       ███████╗    ##
// ##    ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝       ╚══════╝    ##
// ##                                                       ##
// ###########################################################
// ##                                                       ##
// ## Copyright (c) 2026 XAUT NEXT-E. All Rights Reserved.  ##
// ## Author: ziyedeyuu@163.com (Zhaoyu Chen)               ##
// ## License: GPL License                                  ##
// ##                                                       ##
// ###########################################################
//
#include "ne_debug_zmq/ne_debug_zmq.hpp"

namespace ne_vision {
namespace extensions {

NeDebugZmq::NeDebugZmq() {
    NE_INFO("[NeDebugZmq] Constructed.");
}

NeDebugZmq::~NeDebugZmq() {
    close();
    NE_INFO("[NeDebugZmq] Destructed.");
}

bool NeDebugZmq::init() {
    NE_INFO("[NeDebugZmq] Initializing ZMQ publisher...");
    // TODO: Implement ZMQ initialization logic
    return true;
}

void NeDebugZmq::close() {
    NE_INFO("[NeDebugZmq] Closing ZMQ publisher...");
    // TODO: Implement ZMQ cleanup logic
}

} // namespace extensions
} // namespace ne_vision
