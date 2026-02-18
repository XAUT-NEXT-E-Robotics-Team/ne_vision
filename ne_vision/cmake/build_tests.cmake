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
        "$ORIGIN/../lib")
    gtest_discover_tests(${test_name})
endmacro()

build_unit_test(ut_channel_and_task)
build_unit_test(ut_log_and_param)
build_unit_test(ut_tracker_2d_kf)

##########################################
# Build all test cases in the categories #
##########################################

message(STATUS "[ne_vision][test] Building manual tests...")

message(STATUS "[ne_vision][test] Building test: mt_auto_aim_video_test.cpp")
add_executable(mt_auto_aim_video_test test/mt_auto_aim_video_test.cpp)
target_link_libraries(mt_auto_aim_video_test PRIVATE ne_vision)
set_target_properties(mt_auto_aim_video_test PROPERTIES INSTALL_RPATH
    "$ORIGIN/../lib")

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
    ut_tracker_2d_kf
    mt_auto_aim_video_test
    RUNTIME
    DESTINATION test/bin
)

# install resource
install_resource(
    SOURCE ${CMAKE_SOURCE_DIR}/test/video
    DESTINATION share/test
)
