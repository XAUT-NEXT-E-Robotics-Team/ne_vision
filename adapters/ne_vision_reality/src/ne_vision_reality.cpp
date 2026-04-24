///////////////////////////////////////////////////////////
//                                                       //
//                        .                .:-:          //
//                        :-:              :-::          //
//                      -----          .:---.            //
//                    .-------.     .:-----:             //
//                   :---------. .:-------.              //
//                  :--------------------.               //
//                 ---------------------                 //
//                .-------:. :---------:                 //
//               :-----:.     .-------.                  //
//              .:---:         .-----.                   //
//            .:-:.              :-:                     //
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
// 串口回环性能测试：接收cmd=0x01，立即回环发送cmd=0x02

#include "ne_serial_driver/ne_serial_driver.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include <atomic>
#include <chrono>
#include <thread>

#pragma pack(push, 1)
struct TestProtocol_t
{
  float   f;
  int32_t i;
};
#pragma pack(pop)

int main()
{
  ne_vision::drivers::NeSerialDriver ser("/dev/ttys002");

  std::atomic<uint64_t> recv_count{0};

  auto handler = ser.NewProtocolHandler<TestProtocol_t>(0x01);
  handler->AddCallBack([&ser, &recv_count](const TestProtocol_t& p) {
    ser.TransmitProtocol(0x02, p);
    recv_count.fetch_add(1, std::memory_order_relaxed);
  });

  ser.Open();

  while (!ser.IsOpen())
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  NV_INFO("Loopback running...");

  uint64_t prev = 0;
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    uint64_t cur = recv_count.load(std::memory_order_relaxed);
    NV_INFO("RX rate: {}/s  total={}", cur - prev, cur);
    prev = cur;
  }
}