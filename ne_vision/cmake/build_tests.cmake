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
# Build test
# The tests can divide into two categories:
#
# [ATTENTION]: The ctest should be only used for the unit test
#              (first categories), and you should not use ctest
#              for the manual test.
#
# 1. unit test: Test case will run automatically and the result is shown
#    automatically. (use "ut" for prefix of test file case name)
# 2. manual test: You need to run the test case manually and you should to
#    check the result by yourself. (use "mt" for prefix of test file case name)

enable_testing()
include(GoogleTest)
include(${CMAKE_CURRENT_LIST_DIR}/install_resource.cmake)

###################################
# Find all dependencies for tests #
###################################
find_package(GTest REQUIRED)

# 你个mathgl，为啥就不能把路径写正常一点
set(MathGL2_DIR "/usr/local/lib/cmake/mathgl")
find_package(MathGL2 REQUIRED)

#######################################
# Build all tests in first categories #
#######################################

message(STATUS "[ne_vision][test] Building unit tests...")

macro(build_unit_test test_name)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(
        ${test_name}
        PRIVATE
        ne_vision
        GTest::GTest GTest::Main)
    set_target_properties(
        ${test_name}
        PROPERTIES
        INSTALL_RPATH
        "$ORIGIN/../lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
        LINK_FLAGS "-Wl,--disable-new-dtags")
    gtest_discover_tests(${test_name})
endmacro()

macro(build_manual_test test_name other_deps)
    message(STATUS "[ne_vision][test] Building test: ${test_name}.cpp")
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(
        ${test_name}
        PRIVATE
        ne_vision
        ${other_deps})
    set_target_properties(
        ${test_name}
        PROPERTIES
        INSTALL_RPATH
        "$ORIGIN/../lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
        LINK_FLAGS "-Wl,--disable-new-dtags")
endmacro()

build_unit_test(ut_channel_and_task)
build_unit_test(ut_log_and_param)

##########################################
# Build all test cases in the categories #
##########################################

message(STATUS "[ne_vision][test] Building manual tests...")

build_manual_test(mt_auto_aim_video_test "")

# 为什么这里必须要显式的写ceres
build_manual_test(mt_tracker_2d "Ceres::ceres;mgl")

####################
# Install all test #
####################

# install lib
install(
    TARGETS ne_vision
    LIBRARY DESTINATION test/lib
)

# install bin
install(
    TARGETS
    ut_channel_and_task
    ut_log_and_param
    mt_auto_aim_video_test
    mt_tracker_2d
    RUNTIME
    DESTINATION test/bin
)

# install resource
install_resource(
    SOURCE ${CMAKE_SOURCE_DIR}/test/video
    DESTINATION share/test
)

install_resource(
    SOURCE ${CMAKE_SOURCE_DIR}/test/config
    DESTINATION share/test
)
