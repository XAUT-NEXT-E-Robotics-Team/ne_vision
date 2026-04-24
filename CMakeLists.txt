###########################################################
##                                                       ##
##                        .                 .:-:         ##
##                       :-:              :-::           ##
##                      -----          .:---.            ##
##                    .-------.     .:-----:             ##
##                   :---------. .:-------.              ##
##                  :--------------------.               ##
##                ---------------------                  ##
##               .-------:. :---------:                  ##
##              :-----:.     .-------.                   ##
##             .:---:         .-----.                    ##
##            .:-:.             :-:                      ##
##          .-:.                 .                       ##
##         .:                                            ##
##                                                       ##
##    ███╗   ██╗███████╗██╗  ██╗████████╗    ███████╗    ##
##    ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝    ██╔════╝    ##
##    ██╔██╗ ██║█████╗   ╚███╔╝    ██║       █████╗      ##
##    ██║╚██╗██║██╔══╝   ██╔██╗    ██║       ██╔══╝      ##
##    ██║ ╚████║███████╗██╔╝ ██╗   ██║       ███████╗    ##
##    ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝       ╚══════╝    ##
##                                                       ##
###########################################################
##                                                       ##
## Copyright (c) 2026 XAUT NEXT-E. All Rights Reserved.  ##
## Author: ziyedeyuu@163.com (Zhaoyu Chen)               ##
## License: GPL License                                  ##
##                                                       ##
###########################################################

# Description:
# NE_VISION_BULDER 
# Just a building manager.

cmake_minimum_required(VERSION 3.22.1)
project(ne_vision_builder)

#######################
# Some basic settings #
#######################
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

####################
# Add some options #
####################

# 是否开启以下这些内容的编译
option(EXTENSION_ENABLE_NE_DEBUG_ZMQ "Enable ZeroMQ debug extension" OFF)

option(DRIVER_ENABLE_NE_SERIAL_DRIVER "Enable serial driver" ON)

option(ADAPTER_ENABLE_NE_VISION_GD "Enable Godot Extension adapter" OFF)
option(ADAPTER_ENABLE_NE_VISION_REALITY "Enable Reality hardware adapter" OFF)


##############################
# Build ne_vision main model #
##############################
message(STATUS "[ne_vision_builder] Adding ne_vision core library")
add_subdirectory(ne_vision)

####################
# Build extensions #
####################

if(EXTENSION_ENABLE_NE_DEBUG_ZMQ)
    message(STATUS "[ne_vision_builder] Extension [ne_debug_zmq] enabled")
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/extensions/ne_debug_zmq/CMakeLists.txt")
        add_subdirectory(extensions/ne_debug_zmq)
    else()
        message(WARNING "[ne_vision_builder] extensions/ne_debug_zmq directory not found, skipping...")
    endif()
endif()

#################
# Build drivers #
#################

if(DRIVER_ENABLE_NE_SERIAL_DRIVER)
    message(STATUS "[ne_vision_builder] Driver [ne_serial_driver] enabled")
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/drivers/ne_serial_driver/CMakeLists.txt")
        add_subdirectory(drivers/ne_serial_driver)
    else()
        message(WARNING "[ne_vision_builder] drivers/ne_serial_driver directory not found, skipping...")
    endif()
endif()

##################
# Build adapters #
##################
if(ADAPTER_ENABLE_NE_VISION_GD)
    message(STATUS "[ne_vision_builder] Adapter [ne_vision_gd] enabled")
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/adapters/ne_vision_gd/CMakeLists.txt")
        add_subdirectory(adapters/ne_vision_gd)
    else()
        message(WARNING "[ne_vision_builder] adapters/ne_vision_gd directory not found, skipping...")
    endif()
endif()

if(ADAPTER_ENABLE_NE_VISION_REALITY)
    message(STATUS "[ne_vision_builder] Adapter [ne_vision_reality] enabled")
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/adapters/ne_vision_reality/CMakeLists.txt")
        add_subdirectory(adapters/ne_vision_reality)
    else()
        message(WARNING "[ne_vision_builder] adapters/ne_vision_reality directory not found, skipping...")
    endif()
endif()
