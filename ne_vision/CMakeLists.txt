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

#############################
# Config CMake options here #
#############################

# If you want to enable building GDExtension for godot sim, set to ON.
option(ENABLE_BUILD_GDEXTENSION "Enable building GDExtension for godot sim" ON)
option(ENABLE_BUILD_ROS2_NODE "Enable building ROS2 node and component" ON)
option(ENABLE_BUILD_TESTS "Enable building tests" ON)
option(USE_TENSORRT_FOR_DETECTOR "Use TensorRT backend for detector" ON)
option(USE_SYMLINK_INSTALL "Use symlink installation" OFF)

option(USE_RERUN_DEBUG "Use rerun for debugging" ON)

#####################
# Find dependencies #
#####################

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
    message(STATUS "[ne_vision] CUDA toolkit found. Enabling CUDA sources compilation.")
    # Enable CUDA language so CMake knows how to build .cu sources
    enable_language(CUDA)
else()
    message(STATUS "[ne_vision] CUDA toolkit not found via CUDAToolkit package. If you need CUDA support, install CUDA or set CUDAToolkit_ROOT.")
endif()

if(USE_TENSORRT_FOR_DETECTOR)
    find_package(CUDAToolkit REQUIRED)
    find_package(CUDA REQUIRED)
    find_library(NVINFER_LIB nvinfer HINTS ${CUDAToolkit_LIBRARY_DIR} /usr/lib/x86_64-linux-gnu)
    find_library(NVINFER_PLUGIN_LIB nvinfer_plugin HINTS ${CUDAToolkit_LIBRARY_DIR} /usr/lib/x86_64-linux-gnu)
    find_library(NVONNXPARSER_LIB nvonnxparser HINTS ${CUDAToolkit_LIBRARY_DIR} /usr/lib/x86_64-linux-gnu)
    enable_language(CUDA)
    if(NOT NVINFER_LIB)
        message(FATAL_ERROR "TensorRT nvinfer library not found. Install TensorRT or set USE_TENSORRT_FOR_DETECTOR=OFF")
    endif()
    message(STATUS "[ne_vision] TensorRT backend enabled for detector.")
endif()
#################################
# Build basic ne_vision library #
#################################

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE NE_VISION_SOURCES src/*.cpp src/*.c src/*.cu)
# Also include any .cu files that may be placed under include/ (some CUDA helpers live there)
file(GLOB_RECURSE NE_VISION_CUDA_HEADERS include/**/*.cu)
if(NE_VISION_CUDA_HEADERS)
    list(APPEND NE_VISION_SOURCES ${NE_VISION_CUDA_HEADERS})
endif()
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
FetchContent_MakeAvailable(rerun_sdk)
target_link_libraries(${PROJECT_NAME} PUBLIC rerun_sdk)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
    ${OpenCV_LIBS}
    openvino::runtime
    spdlog::spdlog
    Eigen3::Eigen
    yaml-cpp
    tinympcstatic
)

if(CUDAToolkit_FOUND)
    # Link CUDA runtime to ensure CUDA symbols are resolved
    target_link_libraries(${PROJECT_NAME} PUBLIC CUDA::cudart)
    # Allow separable compilation if CUDA code uses device linking across translation units
    set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()

if(USE_TENSORRT_FOR_DETECTOR)
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_TENSORRT)
    target_link_libraries(${PROJECT_NAME} PUBLIC
        CUDA::cudart
        ${NVINFER_LIB}
        ${NVINFER_PLUGIN_LIB}
        ${NVONNXPARSER_LIB}
    )
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

# include(cmake/build_real.cmake)

#################################
# Build ros2 node and component #
#################################
