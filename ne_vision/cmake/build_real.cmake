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
#

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE NE_VISION_REAL_SOURCES
  interfaces/real/src/*.cpp
  interfaces/real/src/*.c
)

file(GLOB_RECURSE NE_VISION_REAL_HEADERS
  interfaces/real/include/*.h
  interfaces/real/include/*.hpp
)

set(HIK_INCLUDE_DIR "/opt/MVS/include")
set(HIK_LIBRARY_DIR "/opt/MVS/lib/64")

include_directories(${HIK_INCLUDE_DIR})

add_executable(
    ne_vision_real
    ${NE_VISION_REAL_SOURCES}
    ${NE_VISION_REAL_HEADERS}
)

target_include_directories(ne_vision
  PUBLIC
  HIK_INCLUDE_DIR
)

target_link_libraries(ne_vision_real
  PRIVATE
  ${HIK_LIBRARY_DIR}/libMvCameraControl.so
)

target_include_directories(ne_vision_real
  PRIVATE
    interfaces/real/include
)

target_link_libraries(ne_vision_real
  PRIVATE
  ne_vision
)

set_target_properties(
    ne_vision_real
    PROPERTIES
    INSTALL_RPATH
    "$ORIGIN/../lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    LINK_FLAGS "-Wl,--disable-new-dtags")

# install lib
install(
    TARGETS ne_vision
    LIBRARY DESTINATION ne_vision_real/lib
)

# install bin
install(
    TARGETS
    ne_vision_real
    RUNTIME
    DESTINATION ne_vision_real/bin
)
