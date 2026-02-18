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
# Build GDExtension for ne_vision module.

##########################
# Set some CMake options #
##########################

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(LOG_PREFIX "[ne_vision][GDExtension]")

set(
  GD_EXTENSION_FILE_PATH
  "${CMAKE_CURRENT_BINARY_DIR}/ne_vision_gd.gdextension"
)

######################################################
# Check and set the target platform and architecture #
######################################################

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "${LOG_PREFIX} This project is only supported on Linux.")
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

message(STATUS "[ne_vision][GDExtension] GDExtension Source files:")

foreach(source_file ${GD_EXTENSION_SOURCES})
  message(STATUS "[ne_vision][GDExtension]   ${source_file}")
endforeach()
foreach(header_file ${GD_EXTENSION_HEADERS})
  message(STATUS "[ne_vision][GDExtension]   ${header_file}")
endforeach()

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

# godot-cpp is only needed for GDExtension build
add_subdirectory(${PROJECT_SOURCE_DIR}/3rds/godot-cpp)

target_link_libraries(
  ne_vision_gd
  PRIVATE
  godot-cpp
  ne_vision
)

set_target_properties(
  ne_vision_gd
  PROPERTIES
  PREFIX "lib"
  OUTPUT_NAME "ne_vision_gd.${ARCHITECTURE}"
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE # Use INSTALL_RPATH for build tree
)

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

set(CMAKE_INSTALL_RPATH "$ORIGIN/../bin")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

install(
  TARGETS ne_vision_gd
  LIBRARY DESTINATION ne_vision_gd/lib
)


install(
  TARGETS ne_vision
  LIBRARY DESTINATION ne_vision_gd/lib
)

install(
  FILES ${GD_EXTENSION_FILE_PATH}
  DESTINATION ne_vision_gd
)

install_resource(
    SOURCE ${CMAKE_SOURCE_DIR}/model
    DESTINATION ne_vision_gd/
)

install_resource(
    SOURCE ${CMAKE_SOURCE_DIR}/config
    DESTINATION ne_vision_gd/
)

#####################################################
# Find all runtime dependencies of the ne_vision_gd #
#####################################################
# install(CODE [[
#     file(GET_RUNTIME_DEPENDENCIES
#         EXECUTABLES $<TARGET_FILE:ne_vision_gd>
#         DIRECTORIES "${CMAKE_BINARY_DIR}"
#         RESOLVED_DEPENDENCIES_VAR _resolved_deps
#         UNRESOLVED_DEPENDENCIES_VAR _unresolved_deps
#     )

#     foreach(_file ${_resolved_deps})
#         message(STATUS "Packing ALL dependency: ${_file}")
#         file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/ne_vision_gd/lib"
#              TYPE SHARED_LIBRARY
#              FOLLOW_SYMLINK_CHAIN
#              FILES "${_file}")
#     endforeach()

#     foreach(_file ${_unresolved_deps})
#         message(WARNING "CRITICAL: Dependency not found: ${_file}")
#     endforeach()
# ]])
