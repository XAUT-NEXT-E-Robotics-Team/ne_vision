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
# Gdextension adapter for simulation.

cmake_minimum_required(VERSION 3.22.1)
project(ne_vision_gd)

#######################
# Some basic settings #
#######################

set(
    GD_EXTENSION_FILE_PATH
    "${CMAKE_CURRENT_BINARY_DIR}/ne_vision_gd.gdextension"
)

set(
    GD_SIM_GD_EXTENSION_DIR
    "${CMAKE_SOURCE_DIR}/ne_vision_sim/ne-vision-sim/gdextensions/ne_vision_gd"
)

######################################################
# Check and set the target platform and architecture #
######################################################

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(OS_RPATH "$ORIGIN")
  set(OS_LINK_FLAGS "-Wl,--disable-new-dtags")
  set(LIB_SUFFIX ".so")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  # macOS 使用 @loader_path 查找同目录下的动态库
  set(OS_RPATH "@loader_path")
  set(OS_LINK_FLAGS "")
  set(LIB_SUFFIX ".dylib")
  # 允许 macOS 上的 RPATH 自动解析
  set(CMAKE_MACOSX_RPATH 1)
else()
  message(FATAL_ERROR "[${PROJECT_NAME}] This project is only supported on Linux and macOS.")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(ARCHITECTURE "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ARCHITECTURE "arm64")
else()
  message(
    FATAL_ERROR
    "[${PROJECT_NAME}] Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}"
  )
endif()

###########################
# Add all file you needed #
###########################

file(GLOB_RECURSE ${PROJECT_NAME}SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE ${PROJECT_NAME}HEADER_FILES "include/*.hpp" "include/*.h")
foreach(${PROJECT_NAME}SOURCE_FILE ${${PROJECT_NAME}SOURCE_FILES})
    message(STATUS "[${PROJECT_NAME}] Source file: ${${PROJECT_NAME}SOURCE_FILE}")
endforeach(${PROJECT_NAME}SOURCE_FILE ${${PROJECT_NAME}SOURCE_FILES})
foreach(${PROJECT_NAME}HEADER_FILE ${${PROJECT_NAME}HEADER_FILES})
    message(STATUS "[${PROJECT_NAME}] Header file: ${${PROJECT_NAME}HEADER_FILE}")
endforeach(${PROJECT_NAME}HEADER_FILE ${${PROJECT_NAME}HEADER_FILES})

#####################################
# Build a GDExtension for ne_vision #
#####################################

add_library(
    ne_vision_gd
    SHARED
    ${${PROJECT_NAME}SOURCE_FILES}
    ${${PROJECT_NAME}HEADER_FILES}
)

target_compile_features(ne_vision_gd PUBLIC cxx_std_20)
target_include_directories(ne_vision_gd PUBLIC include)

# Link ne_vision.so
target_link_libraries(ne_vision_gd PUBLIC ne_vision)

# godot-cpp
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rds/godot-cpp)
target_link_libraries(ne_vision_gd PUBLIC ne_vision godot-cpp)

# Set library properties and RPATH for dynamic loading
set_target_properties(
    ne_vision_gd
    PROPERTIES
    PREFIX "lib"
    OUTPUT_NAME "ne_vision_gd.${ARCHITECTURE}"
    INSTALL_RPATH "${OS_RPATH}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    LINK_FLAGS "${OS_LINK_FLAGS}"
    BUILD_WITH_INSTALL_RPATH TRUE
    # MacOS RPATH
    MACOSX_RPATH TRUE
)

#######################################
# Auto generate the .gdextension file #
#######################################

add_custom_command(
  OUTPUT ${GD_EXTENSION_FILE_PATH}
  COMMAND python3
          ${CMAKE_CURRENT_SOURCE_DIR}/gen_gdextension.py
          ${GD_EXTENSION_FILE_PATH}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/gen_gdextension.py
  COMMENT "[${PROJECT_NAME}] Generating GDExtension file: ${GD_EXTENSION_FILE_PATH}"
  VERBATIM
)

add_custom_target(
    generate_gdextension ALL
    DEPENDS ${GD_EXTENSION_FILE_PATH}
)

########################################################################
# Copy Godot extension to simulation project directory when it install #
########################################################################

include(${CMAKE_SOURCE_DIR}/cmake/install_resource.cmake)

# Install resource 
install_resource(
    SOURCE ${GD_EXTENSION_FILE_PATH}
    DESTINATION ./
    BASE_DIR ${GD_SIM_GD_EXTENSION_DIR}
)

install_resource(
    SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/config
    DESTINATION ./
    BASE_DIR ${GD_SIM_GD_EXTENSION_DIR}
)

# Use default model in ne_vision
install_resource(
    SOURCE ${CMAKE_SOURCE_DIR}/ne_vision/model
    DESTINATION ./
    BASE_DIR ${GD_SIM_GD_EXTENSION_DIR}
)

# Shared library
install(TARGETS ne_vision_gd
        DESTINATION ${GD_SIM_GD_EXTENSION_DIR}
        LIBRARY DESTINATION ${GD_SIM_GD_EXTENSION_DIR}/lib
)
