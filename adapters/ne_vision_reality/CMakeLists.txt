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
# NE_VISION_REALITY
# Real hardware adapter: connects ne_vision with ne_serial_driver.

cmake_minimum_required(VERSION 3.22.1)
project(ne_vision_reality)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###################################
# Add all source and header files #
###################################

file(GLOB_RECURSE NE_VISION_REALITY_SOURCES
    "src/*.cpp" "src/*.c"
)

file(GLOB_RECURSE NE_VISION_REALITY_HEADERS
    "include/*.hpp" "include/*.h"
)

foreach(file ${NE_VISION_REALITY_SOURCES} ${NE_VISION_REALITY_HEADERS})
    message(STATUS "[ne_vision_reality] Adding source/header file: ${file}")
endforeach()

#####################
# Build executable  #
#####################

add_executable(ne_vision_reality ${NE_VISION_REALITY_SOURCES} ${NE_VISION_REALITY_HEADERS})
target_link_libraries(ne_vision_reality PRIVATE ne_vision ne_serial_driver)
target_compile_features(ne_vision_reality PRIVATE cxx_std_20)
target_include_directories(ne_vision_reality PRIVATE include)

###########
# Install #
###########
install(TARGETS ne_vision_reality
    RUNTIME DESTINATION bin
)
