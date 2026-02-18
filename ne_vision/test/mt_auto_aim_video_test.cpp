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
//

#include <opencv2/highgui.hpp>
#include <openvino/runtime/properties.hpp>
#include <cstdlib>
#include <string>

#include "ne_vision/ne_auto_aim.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"

#include "opencv2/opencv.hpp"

using namespace ne_vision;

int main()
{

  const char* install_path_name = "NE_VISION_INSTALL_PATH";
  const char* install_path = std::getenv(install_path_name);
  NV_ASSERT(install_path != nullptr &&
            "You need to set the NE_VISION_INSTALL_PATH environment variable.");
  // NV_ASSERT_PATH(install_path);

  std::string install_path_str = std::string(install_path);

  std::string video_path = install_path_str + "/share/test/video/test.mp4";
  std::string config_path = install_path_str + "/share/config/config.yaml";
  std::string out_path = install_path_str + "/share/test/video/out.mp4";

  NV_INFO("NE_VISION_INSTALL_PATH: {}", install_path);
  NV_INFO("Config Path: {}", config_path);
  NV_INFO("Video Path: {}", video_path);

  NeAutoAim auto_aim;
  auto_aim.Start(config_path);
  cv::VideoCapture cap(video_path);

  if (cap.isOpened())
  {
    cv::Mat frame;

    bool one_frame_mode = false;

    int             fourcc = cv::VideoWriter::fourcc('M', 'P', '4', 'V');
    cv::VideoWriter writer(out_path,
                           fourcc,
                           30,
                           cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH),
                                    cap.get(cv::CAP_PROP_FRAME_HEIGHT)));

    while (1)
    {
      cap >> frame;
      if (frame.empty())
        break;
      auto_aim.UpdateFrame(frame, 'B');
      auto_aim.AutoAim();
      cv::Mat re;
      auto_aim.DebugFrame(re);
      if (!re.empty())
      {
        writer.write(re);
        cv::imshow("abab", re);
      }
      else
      {
        writer.write(frame);
        cv::imshow("abab", frame);
      }

      auto key = cv::waitKey(one_frame_mode ? 0 : 20);
      if (key == 27)
        break;
      if (key == 'p')
      {
        one_frame_mode = !one_frame_mode;
      }
    }
  }
  else
  {
    NV_ERROR("Videon Not Open!");
  }

  auto_aim.Stop();
  return 0;
}
