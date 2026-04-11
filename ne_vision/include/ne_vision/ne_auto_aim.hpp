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
// Auto-aiming module for ne_vision.

#pragma once

#include <memory>

#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/utils/ne_task.hpp"

#include "ne_vision/detector/ne_detector.hpp"
#include "ne_vision/tracker/ne_tracker_2d.hpp"
#include "ne_vision/tracker/ne_tracker_3d.hpp"
#include "ne_vision/planner/ne_mashiro_planner.hpp"
#include "ne_vision/debug/ne_vision_visualization.hpp"

namespace ne_vision
{

class NeAutoAim final
{
public:
  explicit NeAutoAim();
  ~NeAutoAim();

  void UpdateFrame(const cv::Mat& frame, char our_color);
  void UpdateImu(const Eigen::Vector3d&    acc,
                 const Eigen::Vector3d&    gyro,
                 const Eigen::Quaterniond& quat,
                 double                    delay_test_s = 0.0);

  inline void UpdateTestImu(double delay_s)
  {
    Eigen::Vector3d    acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d    gyro = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

    UpdateImu(acc, gyro, quat, delay_s);
  }

  inline void UpdateMuzzelVelocity(double muzzel_velocity)
  {
    muzzel_velocity_ = muzzel_velocity;
  }

  inline bool IsRunning() const { return is_running_; }
  void        Start(std::string config_file_path);
  void        AutoAim();

  void DebugFrame(cv::Mat& frame);

  void GetResult(double& yaw, double& pitch);

  void Stop();

  const NeParam& Params();

private:
  void setupParameters();
  void setupChannels();
  void setupTasks();

  struct
  {
    std::unique_ptr<NeTask> detector_uPtr_;
    std::unique_ptr<NeTask> tracker_2d_uPtr_;
    std::unique_ptr<NeTask> tracker_3d_uPtr_;
    std::unique_ptr<NeTask> mashiro_planner_uPtr_;

    std::unique_ptr<NeTask> debug_visualization_uPtr_;
  } tasks_;

  struct
  {
    std::shared_ptr<NeDetector>       detector_sPtr_;
    std::shared_ptr<NeTracker2D>      tracker_2d_sPtr_;
    std::shared_ptr<NeTracker3D>      tracker_3d_sPtr_;
    std::shared_ptr<NeMashiroPlanner> mashiro_planner_sPtr_;

    std::shared_ptr<NeVisionVisualization> debug_visualization_sPtr_;
  } task_objs_;

  bool is_running_ = false;

  double muzzel_velocity_ = 20.0;
};

} // namespace ne_vision
