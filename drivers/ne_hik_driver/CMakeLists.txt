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
# NE_HIK_DRIVER
# Hardware driver for Hikvision (MVS SDK) cameras.

cmake_minimum_required(VERSION 3.22.1)
project(ne_hik_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###################
# Find MVS SDK    #
###################

set(MVS_SDK_ROOT "/opt/MVS" CACHE PATH "Path to Hikvision MVS SDK root")

find_path(MVS_INCLUDE_DIR
    NAMES MvCameraControl.h
    PATHS "${MVS_SDK_ROOT}/include"
    REQUIRED
)

find_library(MVS_LIBRARY
    NAMES MvCameraControl
    PATHS "${MVS_SDK_ROOT}/lib/64"
          "${MVS_SDK_ROOT}/lib/aarch64"
          "${MVS_SDK_ROOT}/lib"
    REQUIRED
)

message(STATUS "[ne_hik_driver] MVS include: ${MVS_INCLUDE_DIR}")
message(STATUS "[ne_hik_driver] MVS library: ${MVS_LIBRARY}")

find_package(OpenCV REQUIRED)

###################################
# Add all source and header files #
###################################

file(GLOB_RECURSE NE_HIK_DRIVER_SOURCES
    "src/*.cpp" "src/*.c"
)

file(GLOB_RECURSE NE_HIK_DRIVER_HEADERS
    "include/*.hpp" "include/*.h"
)

foreach(file ${NE_HIK_DRIVER_SOURCES} ${NE_HIK_DRIVER_HEADERS})
    message(STATUS "[ne_hik_driver] Adding source/header file: ${file}")
endforeach()

#####################
# Build the library #
#####################

add_library(ne_hik_driver STATIC ${NE_HIK_DRIVER_SOURCES} ${NE_HIK_DRIVER_HEADERS})
target_link_libraries(ne_hik_driver PUBLIC ne_vision ${MVS_LIBRARY} ${OpenCV_LIBS})
target_compile_features(ne_hik_driver PUBLIC cxx_std_20)
target_include_directories(ne_hik_driver
    PUBLIC  include
    PRIVATE "${MVS_INCLUDE_DIR}"
)

#####################################
# Build and install test executable #
#####################################

message(STATUS "[ne_hik_driver] Building test executable")
add_executable(ne_hik_driver_test test/ne_hik_driver_test.cpp)
target_link_libraries(ne_hik_driver_test PRIVATE ne_hik_driver)
target_compile_features(ne_hik_driver_test PRIVATE cxx_std_20)

# Embed MVS SDK lib path into binary so runtime linker finds it without ldconfig
get_filename_component(MVS_LIB_DIR "${MVS_LIBRARY}" DIRECTORY)
set_target_properties(ne_hik_driver_test PROPERTIES
    INSTALL_RPATH "${MVS_LIB_DIR}"
    BUILD_RPATH   "${MVS_LIB_DIR}"
)

install(TARGETS ne_hik_driver_test RUNTIME DESTINATION bin)