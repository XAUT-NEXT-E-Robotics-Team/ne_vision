///////////////////////////////////////////////////////////
//                                                       //
//                        .                 .:-:         //
//                       :-:              :-::           //
//                      -----          .:---.            //
//                    .-------.     .:-----:             //
//                   :---------. .:-------.              //
//                  :--------------------.               //
//                ---------------------                  //
//               .-------:. :---------:                  //
//              :-----:.     .-------.                   //
//             .:---:         .-----.                    //
//            .:-:.             :-:                      //
//          .-:.                 .                       //
//         .:                                            //
//                                                       //
//    ███╗   ██╗███████╗██╗  ██╗████████╗    ███████╗    //
//    ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝    ██╔════╝    //
//    ██╔██╗ ██║█████╗   ╚███╔╝    ██║       █████╗      //
//    ██║╚██╗██║██╔══╝   ██╔██╗    ██║       ██╔══╝      //
//    ██║ ╚████║███████╗██╔╝ ██╗   ██║       ███████╗    //
//    ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝       ╚══════╝    //
//                                                       //
///////////////////////////////////////////////////////////
//                                                       //
// Copyright (c) 2026 XAUT NEXT-E. All Rights Reserved.  //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// Description:
// Unit tests for NeParam and NeLog (including weak symbol override)

#include "gtest/gtest.h"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include <thread>
#include <vector>
#include <string>
#include <atomic>
#include <mutex>

// -----------------------------------------------------------------------------
// NeLog Tests with Weak Symbol Override
// -----------------------------------------------------------------------------

// Capture logs to verify weak override works
static std::vector<std::string> g_log_capture;
static std::mutex g_log_capture_mtx;

// Override the weak functions defined in ne_log.cpp
// This simulates the user providing their own logging implementation (e.g. printf)
// linking against the library.
namespace ne_vision {

void NvWeakLogTrace(const std::string& msg) {
    std::lock_guard<std::mutex> lock(g_log_capture_mtx);
    // Use printf style format or just push to buffer for verification
    g_log_capture.push_back(std::string("[TRACE] ") + msg);
}

void NvWeakLogDebug(const std::string& msg) {
    std::lock_guard<std::mutex> lock(g_log_capture_mtx);
    g_log_capture.push_back(std::string("[DEBUG] ") + msg);
}

void NvWeakLogInfo(const std::string& msg) {
    std::lock_guard<std::mutex> lock(g_log_capture_mtx);
    g_log_capture.push_back(std::string("[INFO] ") + msg);
}

void NvWeakLogWarn(const std::string& msg) {
    std::lock_guard<std::mutex> lock(g_log_capture_mtx);
    g_log_capture.push_back(std::string("[WARN] ") + msg);
}

void NvWeakLogError(const std::string& msg) {
    std::lock_guard<std::mutex> lock(g_log_capture_mtx);
    g_log_capture.push_back(std::string("[ERROR] ") + msg);
}

} // namespace ne_vision

// Helper class to test context prefix
class TestContextName {
public:
    std::string GetName() const { return "TestCtx"; }
    void DoLog() {
        NV_INFO("Message from context");
    }
};

TEST(NeLogTest, WeakOverrideAndFormatting) {
    {
        std::lock_guard<std::mutex> lock(g_log_capture_mtx);
        g_log_capture.clear();
    }

    NV_INFO("Hello {}", "World");
    NV_WARN("This is warning code {}", 404);
    
    TestContextName ctx;
    ctx.DoLog();

    const std::vector<std::string>& logs = g_log_capture;
    
    ASSERT_GE(logs.size(), 3);
    EXPECT_EQ(logs[0], "[INFO] Hello World");
    EXPECT_EQ(logs[1], "[WARN] This is warning code 404");
    // Verify NV_GET_CONTEXT_PREFIX() works (automatically adds [GetName()] )
    EXPECT_EQ(logs[2], "[INFO] [TestCtx] Message from context"); 
}

// -----------------------------------------------------------------------------
// NeParam Tests
// -----------------------------------------------------------------------------

using namespace ne_vision;

TEST(NeParamTest, BasicReadWrite) {
    NeParam& param = NeParam::Instance();
    param.Reset();
    
    // Test assignment with new operator[] wrapping
    param["int_val"] = 42;
    param["str_val"] = std::string("hello");
    param["double_val"] = 3.14;
    
    // Test reading via wrapper
    int i = param["int_val"].as<int>();
    std::string s = param["str_val"].as<std::string>();
    double d = param["double_val"].as<double>();
    
    EXPECT_EQ(i, 42);
    EXPECT_EQ(s, "hello");
    EXPECT_DOUBLE_EQ(d, 3.14);
}

TEST(NeParamTest, NestedAccess) {
    NeParam& param = NeParam::Instance();
    param.Reset();
    
    // Write deep structure mimicking yaml-cpp
    param["network"]["ip"] = std::string("192.168.1.1");
    param["network"]["port"] = 8080;
    
    // Read back
    std::string ip = param["network"]["ip"].as<std::string>();
    int port = param["network"]["port"].as<int>();
    
    EXPECT_EQ(ip, "192.168.1.1");
    EXPECT_EQ(port, 8080);
}

TEST(NeParamTest, ThreadSafety) {
    NeParam& param = NeParam::Instance();
    param.Reset();
    const int num_threads = 10;
    const int iterations = 100;
    
    param["counter"] = 0;
    
    std::vector<std::thread> threads;
    for(int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&param, i]() {
            for(int j = 0; j < iterations; ++j) {
                // Safely update using locked wrapping
                std::string key = "thread_" + std::to_string(i);
                param[key] = j;
                int val = param[key].as<int>();
                EXPECT_EQ(val, j);
            }
        });
    }
    
    for(auto& t : threads) t.join();
}

TEST(NeParamTest, Dump) {
    NeParam& param = NeParam::Instance();
    param.Reset();
    param["foo"] = std::string("bar");
    
    std::string dump = param.Dump();
    EXPECT_TRUE(dump.find("bar") != std::string::npos); 
}

TEST(NeParamTest, SetDefault) {
  ne_vision::NeParam& param = NeParam::Instance();
  param.Reset();
  
  // Custom helper for 'SetDefault' behavior using IsDefined check
  auto setDefault = [&](const std::string& key, auto val) {
      if (!param["config"][key].IsDefined()) {
          param["config"][key] = val;
      }
  };
  
  // Case 1: Key does not exist -> Set default
  if(!param["config"]["threshold"].IsDefined()) param["config"]["threshold"] = 0.5;
  EXPECT_DOUBLE_EQ(param["config"]["threshold"].as<double>(), 0.5);
  
  // Case 2: Key exists -> Do NOT overwrite
  param["config"]["threshold"] = 0.8;
  if(!param["config"]["threshold"].IsDefined()) param["config"]["threshold"] = 0.5;
  EXPECT_DOUBLE_EQ(param["config"]["threshold"].as<double>(), 0.8);
  
  // Case 3: Key exists, type mismatch -> Overwrite (relying on user logic)
  param["config"]["mode"] = std::string("auto");
  // Check type manually? Or just overwrite relying on application logic.
  // The 'wrapper' doesn't implement SetDefault logic internally anymore to keep it thin.
  // User code should handle logic.
  try {
      param["config"]["mode"].as<int>();
  } catch(...) {
      param["config"]["mode"] = 1;
  }
  EXPECT_EQ(param["config"]["mode"].as<int>(), 1);
}

TEST(NeParamTest, ArrayIteration) {
    NeParam& param = NeParam::Instance();
    param.Reset();
    
    param["list"].push_back(10);
    param["list"].push_back(20);
    param["list"].push_back(30);
    
    std::vector<int> values;
    // Iterate using range-based for loop (lock held by temporary wrapper)
    for(auto it : param["list"]) {
        values.push_back(it.as<int>());
    }
    
    EXPECT_EQ(values.size(), 3);
    EXPECT_EQ(values[0], 10);
    EXPECT_EQ(values[1], 20);
    EXPECT_EQ(values[2], 30);
}


int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
