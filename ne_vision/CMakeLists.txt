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
# Main CMakeLists.txt for ne_vision module.

cmake_minimum_required(VERSION 3.27)

project(ne_vision)

#############################
# Config CMake options here #
#############################

# If you want to enable building GDExtension for godot sim, set to ON.
option(ENABLE_BUILD_GDEXTENSION "Enable building GDExtension for godot sim" ON)
option(ENABLE_BUILD_ROS2_NODE "Enable building ROS2 node and component" ON)
option(ENABLE_BUILD_TESTS "Enable building tests" ON)

option(USE_SYMLINK_INSTALL "Use symlink installation" OFF)

option(USE_RERUN_DEBUG "Use rerun for debugging" ON)

#####################
# Find dependencies #
#####################

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Ceres REQUIRED)
message(STATUS "Ceres libraries: ${CERES_LIBRARIES}")

#################################
# Build basic ne_vision library #
#################################

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE NE_VISION_SOURCES src/*.cpp src/*.c)
file(GLOB_RECURSE NE_VISION_HEADERS include/*.h include/*.hpp)
foreach(source_file ${NE_VISION_SOURCES})
    message(STATUS "[ne_vision] Source file: ${source_file}")
endforeach()
foreach(header_file ${NE_VISION_HEADERS})
    message(STATUS "[ne_vision] Header file: ${header_file}")
endforeach()

add_library(
    ${PROJECT_NAME}
    SHARED
    ${NE_VISION_SOURCES}
    ${NE_VISION_HEADERS}
)

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

add_subdirectory(3rds/yaml-cpp)
add_subdirectory(3rds/Sophus)

if(USE_RERUN_DEBUG)
    message(STATUS "[ne_vision] Rerun debug is enabled.
        Rerun SDK will be integrated for enhanced debugging capabilities.")
    message(STATUS "[ne_vision] Please wait for download.")
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_RERUN)
    include(FetchContent)
    FetchContent_Declare(
        rerun_sdk
        URL
        https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)

    # Ensure rerun_sdk is built with -fPIC
    # so it can be linked into our shared library
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    FetchContent_MakeAvailable(rerun_sdk)
    target_link_libraries(${PROJECT_NAME} PUBLIC rerun_sdk)
endif()

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
    ${OpenCV_LIBS}
    openvino::runtime
    spdlog::spdlog
    Eigen3::Eigen
    yaml-cpp
    sophus
    PRIVATE
    Ceres::ceres
)

############
# Optimize #
############

file(GLOB_RECURSE NE_VISION_UTILS_HEADERS include/ne_vision/utils/*.h include/ne_vision/utils/*.hpp)

target_precompile_headers(
    ${PROJECT_NAME}
    PUBLIC
    ${NE_VISION_UTILS_HEADERS}
)

###########
# Install #
###########

include(${PROJECT_SOURCE_DIR}/cmake/install_resource.cmake)

install_resource(
    SOURCE ${CMAKE_SOURCE_DIR}/model
    DESTINATION share
)

install_resource(
    SOURCE ${CMAKE_SOURCE_DIR}/config
    DESTINATION share
)

install(PROGRAMS ${CMAKE_SOURCE_DIR}/scripts/setup.sh DESTINATION .)

#############################
# Build interfaces and test #
#############################

if(ENABLE_BUILD_GDEXTENSION)
    message(STATUS "[ne_vision] GDExtension build is enabled.")
    message(STATUS "[ne_vision] Start building GDExtension for godot sim...")
    include(cmake/build_gdextension.cmake)
endif()

if(ENABLE_BUILD_TESTS)
    message(STATUS "[ne_vision] Tests build is enabled.")
    message(STATUS "[ne_vision] Start building tests...")
    include(cmake/build_tests.cmake)
endif()

include(cmake/build_real.cmake)

#################################
# Build ros2 node and component #
#################################
