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
# NE_SERIAL_DRIVER 
# Hardware driver for serial communication.

cmake_minimum_required(VERSION 3.22.1)
project(ne_serial_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###################################
# Add all source and header files #
###################################

file(GLOB_RECURSE NE_SERIAL_DRIVER_SOURCES
    "src/*.cpp" "src/*.c"
)

file(GLOB_RECURSE NE_SERIAL_DRIVER_HEADERS
    "include/*.hpp" "include/*.h"
)

foreach(file ${NE_SERIAL_DRIVER_SOURCES} ${NE_SERIAL_DRIVER_HEADERS})
    message(STATUS "[ne_serial_driver] Adding source/header file: ${file}")
endforeach()

#####################
# Build the library #
#####################

add_library(ne_serial_driver STATIC ${NE_SERIAL_DRIVER_SOURCES} ${NE_SERIAL_DRIVER_HEADERS})
target_link_libraries(ne_serial_driver PUBLIC ne_vision)
target_compile_features(ne_serial_driver PUBLIC cxx_std_20)
target_include_directories(ne_serial_driver PUBLIC include)
