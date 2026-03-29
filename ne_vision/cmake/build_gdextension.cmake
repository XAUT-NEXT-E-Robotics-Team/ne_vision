###########################################################
##                                                       ##
##                        .                 .:-:         ##
##                        :-:               :-::         ##
##                      -----           .:---.           ##
##                    .-------.     .:-----:             ##
##                   :---------. .:-------.              ##
##                  :--------------------.               ##
##                 ---------------------                 ##
##                .-------:. :---------:                 ##
##               :-----:.     .-------.                  ##
##              .:---:         .-----.                   ##
##            .:-:.              :-:                     ##
##          .-:.                  .                      ##
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
# Build GDExtension for ne_vision module with Linux/macOS support.

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(LOG_PREFIX "[ne_vision][GDExtension]")

set(
  GD_EXTENSION_FILE_PATH
  "${CMAKE_CURRENT_BINARY_DIR}/ne_vision_gd.gdextension"
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
  message(FATAL_ERROR "${LOG_PREFIX} This project is only supported on Linux and macOS.")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(ARCHITECTURE "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ARCHITECTURE "arm64")
else()
  message(
    FATAL_ERROR
    "${LOG_PREFIX} Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}"
  )
endif()

#####################################
# Build a GDExtension for ne_vision #
#####################################

file(GLOB_RECURSE NE_VISION_GD_EXTENSION_SOURCES
  interfaces/gdextension/src/*.cpp
  interfaces/gdextension/src/*.c
)

file(GLOB_RECURSE NE_VISION_GD_EXTENSION_HEADERS
  interfaces/gdextension/include/*.h
  interfaces/gdextension/include/*.hpp
)

message(STATUS "${LOG_PREFIX} GDExtension Source files:")
foreach(source_file ${NE_VISION_GD_EXTENSION_SOURCES} ${NE_VISION_GD_EXTENSION_HEADERS})
  message(STATUS "${LOG_PREFIX}   ${source_file}")
endforeach()

# 定义库文件
add_library(
  ne_vision_gd
  SHARED
  ${NE_VISION_GD_EXTENSION_SOURCES}
  ${NE_VISION_GD_EXTENSION_HEADERS}
)

target_compile_features(ne_vision_gd PUBLIC cxx_std_20)
target_include_directories(
  ne_vision_gd
  PUBLIC
  interfaces/gdextension/include
)

# 编译 godot-cpp 依赖
if(NOT TARGET godot-cpp)
    add_subdirectory(${PROJECT_SOURCE_DIR}/3rds/godot-cpp)
endif()

target_link_libraries(
  ne_vision_gd
  PRIVATE
  godot-cpp
  ne_vision
)

# 设置目标属性，适配 macOS 命名规范
set_target_properties(
  ne_vision_gd
  PROPERTIES
  PREFIX "lib"
  OUTPUT_NAME "ne_vision_gd.${ARCHITECTURE}"
  INSTALL_RPATH "${OS_RPATH}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
  LINK_FLAGS "${OS_LINK_FLAGS}"
  BUILD_WITH_INSTALL_RPATH TRUE
  # macOS 专属设置：
  MACOSX_RPATH TRUE
)

# 自动生成 .gdextension 配置文件
add_custom_command(
  OUTPUT ${GD_EXTENSION_FILE_PATH}
  COMMAND python3
          ${PROJECT_SOURCE_DIR}/interfaces/gdextension/gen_gdextension.py
          ${GD_EXTENSION_FILE_PATH}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/interfaces/gdextension/gen_gdextension.py
  COMMENT "${LOG_PREFIX} Generating GDExtension file: ${GD_EXTENSION_FILE_PATH}"
  VERBATIM
)

add_custom_target(
  generate_gdextension ALL
  DEPENDS ${GD_EXTENSION_FILE_PATH}
)

#################
# Install Logic #
#################

set(CMAKE_INSTALL_RPATH "${OS_RPATH}/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 安装动态库至 lib 目录
install(
  TARGETS ne_vision_gd ne_vision
  LIBRARY DESTINATION ne_vision_gd/lib
  RUNTIME DESTINATION ne_vision_gd/lib  # 兼容 Windows 逻辑
)

# 安装配置文件至根目录
install(
  FILES ${GD_EXTENSION_FILE_PATH}
  DESTINATION ne_vision_gd
)

# 安装资源与配置
install_resource(
    SOURCE ${CMAKE_SOURCE_DIR}/model
    DESTINATION ne_vision_gd/
)

install_resource(
    SOURCE ${CMAKE_SOURCE_DIR}/config
    DESTINATION ne_vision_gd/
)
