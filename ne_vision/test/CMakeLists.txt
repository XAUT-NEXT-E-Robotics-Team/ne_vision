cmake_minimum_required(VERSION 3.22)
project(ne_task_tests LANGUAGES CXX)

# Standalone: cmake -S ne_vision/test -B build/task-tests
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)
find_package(spdlog REQUIRED)
enable_testing()
add_executable(ne_task_test ne_task_test.cpp ../src/ne_task.cpp ../src/ne_log.cpp)
target_compile_features(ne_task_test PRIVATE cxx_std_20)
target_include_directories(ne_task_test PRIVATE ../include)
target_link_libraries(ne_task_test PRIVATE GTest::gtest_main spdlog::spdlog Threads::Threads)
add_test(NAME ne_task_stop COMMAND ne_task_test)
set_tests_properties(ne_task_stop PROPERTIES TIMEOUT 30)
