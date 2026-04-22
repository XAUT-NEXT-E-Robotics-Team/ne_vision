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
option(ADAPTER_ENABLE_NE_VISION_GD "Enable Godot Extension adapter" ON)

# 定义一个全局选项，控制是否在安装时使用软链接复制资源
option(NE_INSTALL_SYMLINK "Use symlinks for resources during install instead of copying" OFF)

##############################
# Build ne_vision main model #
##############################
message(STATUS "[ne_vision_builder] Adding ne_vision core library")
add_subdirectory(ne_vision)

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
