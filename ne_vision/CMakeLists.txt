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

cmake_minimum_required(VERSION 3.22.1)

project(ne_vision)

option(NV_USE_TENSORRT "Use TensorRT backend for detector (requires CUDA + TensorRT)" OFF)

#####################
# Find dependencies #
#####################

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

if(NV_USE_TENSORRT)
    find_package(CUDAToolkit REQUIRED)
    find_library(NVINFER_LIB nvinfer
        HINTS ${CUDAToolkit_LIBRARY_DIR} /usr/lib/x86_64-linux-gnu)
    find_library(NVINFER_PLUGIN_LIB nvinfer_plugin
        HINTS ${CUDAToolkit_LIBRARY_DIR} /usr/lib/x86_64-linux-gnu)
    if(NOT NVINFER_LIB)
        message(FATAL_ERROR "[ne_vision] TensorRT nvinfer not found. Install TensorRT or set NV_USE_TENSORRT=OFF")
    endif()
    enable_language(CUDA)
    message(STATUS "[ne_vision] TensorRT backend enabled.")
else()
    find_package(OpenVINO REQUIRED)
    message(STATUS "[ne_vision] OpenVINO backend enabled.")
endif()

#################################
# Build basic ne_vision library #
#################################

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE NE_VISION_SOURCES src/*.cpp src/*.c)
file(GLOB_RECURSE NE_VISION_HEADERS include/*.h include/*.hpp)

if(NV_USE_TENSORRT)
    list(FILTER NE_VISION_SOURCES EXCLUDE REGEX ".*/openvino_infer\\.cpp$")
else()
    list(FILTER NE_VISION_SOURCES EXCLUDE REGEX ".*/ne_cu_infer\\.cpp$")
    file(GLOB_RECURSE NV_CU_HEADERS include/ne_vision/detector/ne_cu/*.h include/ne_vision/detector/ne_cu/*.hpp)
    list(REMOVE_ITEM NE_VISION_HEADERS ${NV_CU_HEADERS})
endif()
foreach(source_file ${NE_VISION_SOURCES})
    message(STATUS "[ne_vision] Source file: ${source_file}")
endforeach()
foreach(header_file ${NE_VISION_HEADERS})
    message(STATUS "[ne_vision] Header file: ${header_file}")
endforeach()

add_library(
    ${PROJECT_NAME}
    STATIC
    ${NE_VISION_SOURCES}
    ${NE_VISION_HEADERS}
)

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

add_subdirectory(3rds/yaml-cpp)
add_subdirectory(3rds/TinyMPC)

# 拦截并过滤掉tinympcstatic库中的Eigen包含路径 BY AI
get_target_property(TINYMPC_INCLUDES tinympcstatic INTERFACE_INCLUDE_DIRECTORIES)
if(TINYMPC_INCLUDES)
    set(TINYMPC_CLEANED_INCLUDES "")
    foreach(INC_DIR IN LISTS TINYMPC_INCLUDES)
        # 如果路径里不包含 "Eigen"，就保留下来
        if(NOT "${INC_DIR}" MATCHES "Eigen")
            list(APPEND TINYMPC_CLEANED_INCLUDES "${INC_DIR}")
        endif()
    endforeach()
    # 把过滤后的纯净路径重新设置回去
    set_target_properties(tinympcstatic PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${TINYMPC_CLEANED_INCLUDES}")
endif()

# 动态生成一个仅包含基础库的 Eigen.h 供外部包含 tinympc 宏时使用，以规避冲突
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/generated_includes/Eigen.h" "#pragma once\n#include <Eigen/Core>\n#include <Eigen/LU>\n")
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/generated_includes")

# 强依赖RERUN
message(STATUS "[ne_vision] Rerun debug is enabled.
    Rerun SDK will be integrated for enhanced debugging capabilities.")
message(STATUS "[ne_vision] Please wait for download.")
include(FetchContent)
FetchContent_Declare(
    rerun_sdk
    URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)

# Ensure rerun_sdk is built with -fPIC
# so it can be linked into our shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
FetchContent_MakeAvailable(rerun_sdk)
target_link_libraries(${PROJECT_NAME} PUBLIC rerun_sdk)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
    ${OpenCV_LIBS}
    spdlog::spdlog
    Eigen3::Eigen
    yaml-cpp
    tinympcstatic
)

if(NV_USE_TENSORRT)
    target_compile_definitions(${PROJECT_NAME} PUBLIC NV_USE_TENSORRT)
    target_link_libraries(${PROJECT_NAME} PUBLIC
        CUDA::cudart
        ${NVINFER_LIB}
        ${NVINFER_PLUGIN_LIB}
    )
    set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
else()
    target_link_libraries(${PROJECT_NAME} PUBLIC openvino::runtime)
endif()

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

include(${CMAKE_SOURCE_DIR}/cmake/install_resource.cmake)

# Install library
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT runtime)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Model
install_resource(
    SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/model/
    DESTINATION share/${PROJECT_NAME}
)

# install(PROGRAMS ${CMAKE_SOURCE_DIR}/scripts/setup.sh DESTINATION .)

#############################
# Build interfaces and test #
#############################

# if(ENABLE_BUILD_GDEXTENSION)
#     message(STATUS "[ne_vision] GDExtension build is enabled.")
#     message(STATUS "[ne_vision] Start building GDExtension for godot sim...")
#     include(cmake/build_gdextension.cmake)
# endif()

