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

#include <atomic>
#include <memory>

#include "ne_hik_driver/ne_hik_driver.hpp"
#include "opencv2/opencv.hpp"

int main()
{
  std::atomic<std::shared_ptr<cv::Mat>> latest_frame;
  latest_frame.store(std::make_shared<cv::Mat>());

  ne_vision::drivers::NeHikDriverParams_t params;
  params.pixel_format = ne_vision::drivers::NeHikPixelFormat_e::BGR8;

  ne_vision::drivers::NeHikDriver driver(
      "test_cam",
      [&latest_frame](cv::Mat& frame, std::chrono::steady_clock::time_point) {
        latest_frame.store(std::make_shared<cv::Mat>(frame.clone()));
      },
      params);

  driver.Open();

  // imshow/waitKey 必须在主线程（X11/Qt GUI 线程）调用
  while (true)
  {
    auto frame = latest_frame.load();
    if (frame && !frame->empty())
      cv::imshow("frame", *frame);
    if (cv::waitKey(1) == 27)
      break;
  }

  driver.Stop();
  return 0;
}