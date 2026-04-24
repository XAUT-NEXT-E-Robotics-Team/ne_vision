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
# NE_DEBUG_ZMQ 
# Use ZeroMQ to send debug messages to other debug tools.

cmake_minimum_required(VERSION 3.22.1)
project(ne_debug_zmq)

#####################
# Find dependencies #
#####################

find_package(cppzmq REQUIRED)

###################################
# Add all source and header files #
###################################

file(GLOB_RECURSE NE_DEBUG_ZMQ_SOURCES
    "src/*.cpp" "src/*.c"
)

file(GLOB_RECURSE NE_DEBUG_ZMQ_HEADERS
    "include/*.hpp" "include/*.h"
)

foreach(file ${NE_DEBUG_ZMQ_SOURCES} ${NE_DEBUG_ZMQ_HEADERS})
    message(STATUS "[ne_debug_zmq] Adding source/header file: ${file}")
endforeach()

#####################
# Build the library #
#####################

add_library(ne_debug_zmq STATIC ${NE_DEBUG_ZMQ_SOURCES} ${NE_DEBUG_ZMQ_HEADERS})
target_link_libraries(ne_debug_zmq PUBLIC cppzmq ne_vision)
target_compile_features(ne_debug_zmq PUBLIC cxx_std_20)
target_include_directories(ne_debug_zmq PUBLIC include)