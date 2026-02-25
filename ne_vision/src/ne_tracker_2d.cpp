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

#include "ne_vision/tracker/ne_tracker_2d.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <algorithm>
#include <ceres/types.h>
#include <chrono>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"

#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/utils/ne_math.hpp"
#include "ne_vision/utils/ne_rerun_debug.hpp"
#include "rerun/archetypes/scalars.hpp"
#include "rerun/archetypes/points3d.hpp"

#define BEBUG_LOG

namespace ne_vision
{

NeTracker2D::NeTracker2D(const std::string&       name,
                         const NeArmors2DCsPtr_t& armors_2d_cs_ptr,
                         const NeImuDataCsPtr_t&  imu_data_cs_ptr,
                         const NeArmors3DCsPtr_t& armors_3d_cs_ptr)
    : name_(name), armors_2d_c_sPtr_(armors_2d_cs_ptr),
      imu_data_c_sPtr_(imu_data_cs_ptr), armors_3d_c_sPtr_(armors_3d_cs_ptr)
{

  // 读取一些参数
  try
  {
    current_aim_.lost_count_threshold =
        NV_PARAM["auto_aim"]["tracker_2d"]["lost_count_threshold"].as<int>();

    // Camera intrinsic parameters.
    const double fx =
        NV_PARAM["hardware"]["camera"]["camera_matrix"]["fx"].as<double>();
    const double fy =
        NV_PARAM["hardware"]["camera"]["camera_matrix"]["fy"].as<double>();
    const double cx =
        NV_PARAM["hardware"]["camera"]["camera_matrix"]["cx"].as<double>();
    const double cy =
        NV_PARAM["hardware"]["camera"]["camera_matrix"]["cy"].as<double>();
    pnp_param_.camera_matrix =
        (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    const double k1 =
        NV_PARAM["hardware"]["camera"]["dist_coeffs"]["k1"].as<double>();
    const double k2 =
        NV_PARAM["hardware"]["camera"]["dist_coeffs"]["k2"].as<double>();
    const double p1 =
        NV_PARAM["hardware"]["camera"]["dist_coeffs"]["p1"].as<double>();
    const double p2 =
        NV_PARAM["hardware"]["camera"]["dist_coeffs"]["p2"].as<double>();
    const double k3 =
        NV_PARAM["hardware"]["camera"]["dist_coeffs"]["k3"].as<double>();
    pnp_param_.dist_coeffs = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);

    // Armor 3D points in the armor frame.
    const double small_armor_width =
        NV_PARAM["rm"]["armor"]["small"]["width"].as<double>();
    const double small_armor_height =
        NV_PARAM["rm"]["armor"]["small"]["height"].as<double>();
    const double large_armor_width =
        NV_PARAM["rm"]["armor"]["large"]["width"].as<double>();
    const double large_armor_height =
        NV_PARAM["rm"]["armor"]["large"]["height"].as<double>();
    const double outpost_armor_width =
        NV_PARAM["rm"]["armor"]["outpost"]["width"].as<double>();
    const double outpost_armor_height =
        NV_PARAM["rm"]["armor"]["outpost"]["height"].as<double>();
    pnp_param_.object_points_small_armor = {
        {0, small_armor_width / 2.0, small_armor_height / 2.0},   // LT
        {0, small_armor_width / 2.0, -small_armor_height / 2.0},  // LB
        {0, -small_armor_width / 2.0, -small_armor_height / 2.0}, // RB
        {0, -small_armor_width / 2.0, small_armor_height / 2.0}}; // RT
    pnp_param_.object_points_large_armor = {
        {0, large_armor_width / 2.0, large_armor_height / 2.0},   // LT
        {0, large_armor_width / 2.0, -large_armor_height / 2.0},  // LB
        {0, -large_armor_width / 2.0, -large_armor_height / 2.0}, // RB
        {0, -large_armor_width / 2.0, large_armor_height / 2.0}}; // RT
    pnp_param_.object_points_outpost_armor = {
        {0, outpost_armor_width / 2.0, outpost_armor_height / 2.0},   // LT
        {0, outpost_armor_width / 2.0, -outpost_armor_height / 2.0},  // LB
        {0, -outpost_armor_width / 2.0, -outpost_armor_height / 2.0}, // RB
        {0, -outpost_armor_width / 2.0, outpost_armor_height / 2.0}}; // RT

    // 必须调用这个
    pnp_param_.ConvertToEigen();

    // Transform from gimbal frame to camera frame.
    const Eigen::Vector3d euler_angle(
        NV_PARAM["hardware"]["camera"]["gimbal_to_camera"]["r"]["r"]
            .as<double>(),
        NV_PARAM["hardware"]["camera"]["gimbal_to_camera"]["r"]["p"]
            .as<double>(),
        NV_PARAM["hardware"]["camera"]["gimbal_to_camera"]["r"]["y"]
            .as<double>());
    gimbal_to_camera_.q =
        Eigen::AngleAxisd(euler_angle.z(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler_angle.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler_angle.x(), Eigen::Vector3d::UnitX());
    gimbal_to_camera_.t
        << NV_PARAM["hardware"]["camera"]["gimbal_to_camera"]["t"]["x"]
               .as<double>(),
        NV_PARAM["hardware"]["camera"]["gimbal_to_camera"]["t"]["y"]
            .as<double>(),
        NV_PARAM["hardware"]["camera"]["gimbal_to_camera"]["t"]["z"]
            .as<double>();
  }
  catch (const std::exception& e)
  {
    NV_ERROR("Failed to load parameters for NeTracker2D: {}", e.what());
    std::exit(EXIT_FAILURE);
  }

  // 处理gimbal to camera
  // clang-format off
  Eigen::Matrix3d CC_to_C;
  CC_to_C <<  0,  0,  1,
             -1,  0,  0,
              0, -1,  0;
  // clang-format on
  gimbal_to_camera_.q = gimbal_to_camera_.q * Eigen::Quaterniond(CC_to_C);
  // 平移没变

  // 初始化Ceres
  yaw_optimize_.options.linear_solver_type = ceres::DENSE_QR;
  yaw_optimize_.options.minimizer_progress_to_stdout = false;
  yaw_optimize_.options.max_num_iterations = 5; // 降低迭代次数，通常2-3次收敛
  yaw_optimize_.options.num_threads = 1;
  yaw_optimize_.options.function_tolerance = 1e-3; // 放宽收敛条件以加速

  // 初始化复用的 Problem 和数据
  yaw_optimize_.problem = std::make_unique<ceres::Problem>();
  yaw_optimize_.img_points.resize(4);
  yaw_optimize_.obj_points.resize(4);

  // 构建 CostFunction，使用指针指向 yaw_optimize_ 中的数据
  ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<detail::ArmorReprojectionError, 8, 1>(
          new detail::ArmorReprojectionError(&yaw_optimize_.img_points,
                                             &yaw_optimize_.obj_points,
                                             &yaw_optimize_.q_c_i,
                                             &pnp_param_.camera_matrix_eigen,
                                             &yaw_optimize_.t_c_a,
                                             &yaw_optimize_.fixed_pitch_rad));

  // 添加残差块，参数块为 yaw_optimize_.yaw 的地址
  yaw_optimize_.problem->AddResidualBlock(
      cost_function, nullptr, &yaw_optimize_.yaw);

  // 初始化 Problem_full (用于 optimize 函数)
  yaw_optimize_.problem_full = std::make_unique<ceres::Problem>();

  // 构建 CostFunction for full optimization
  ceres::CostFunction* cost_function_full =
      new ceres::AutoDiffCostFunction<detail::OptimizeCost, 8, 1, 3>(
          new detail::OptimizeCost(&yaw_optimize_.img_points,
                                   &yaw_optimize_.obj_points,
                                   &yaw_optimize_.q_g_c,
                                   &yaw_optimize_.q_i_g,
                                   &pnp_param_.camera_matrix_eigen,
                                   &yaw_optimize_.t_g_c,
                                   &yaw_optimize_.fixed_pitch_rad));

  // 添加残差块
  yaw_optimize_.problem_full->AddResidualBlock(
      cost_function_full, nullptr, &yaw_optimize_.yaw, yaw_optimize_.t.data());

  // 初始化current aim 的计数器，令其大于最大值，确保第一次识别可以初始化
  current_aim_.lost_count = current_aim_.lost_count_threshold + 10;
}

void NeTracker2D::Tarck2D()
{
  // std::chrono::steady_clock::time_point now =
  // std::chrono::steady_clock::now();

  if (!armors_2d_c_sPtr_->Receive(armors_2d_))
  {
    // 不会发生吧，除非把任务触发方式设置错了。
    NV_WARN("Failed to receive 2D armors, skip this frame.");
    return;
  }

  // 清一下
  armors_3d_.armors.clear();

  trackAndChoose();

  if (!current_aim_.IsDetected())
    goto send; // 只要本次没识别到，就退出了

  // 能到这里，armors_2d不可能是空的
  solvePnP();

  current_aim_.cap_stamp = armors_2d_.cap_stamp;

  if (current_aim_.aim_armors.empty())
    goto send; // 可能全都解算失败了？

  if (!matchStamp())
    goto send;

  transformToImuFrame();
  // yawOptimize();
  optimize();

  // ONLY FOR DEBUG
  reprojectAndFillDebugInfo();

  // 循环填数据
  for (auto& each : current_aim_.aim_armors)
  {
    NeArmors3D_t::Armor3D_t armor_3d;
    armor_3d.debug = each.debug_info;
    armor_3d.q = each.imu_to_armor.q;
    armor_3d.t = each.imu_to_armor.t;
    armor_3d.yaw = each.imu_to_armor.yaw;
    armors_3d_.armors.push_back(armor_3d);
  }

  // 无论是谁不对，都不能阻止发数据，因为会影响可视化配对
send:
  armors_3d_.cap_stamp = armors_2d_.cap_stamp;
  armors_3d_c_sPtr_->Transmit(armors_3d_);

  // std::chrono::steady_clock::time_point end =
  // std::chrono::steady_clock::now(); double duration_ms =
  //     std::chrono::duration<double, std::milli>(end - now).count();
  // NV_DEBUG("Tracker 2D took {:.2f} ms", duration_ms);
}

void NeTracker2D::trackAndChoose()
{
  // 跟踪和选板。
  // 1.
  // 跟踪当前目标，无数据一定时间后认为丢失。用来防止出现一两次误识别就导致目标变化这种抽象的事情。
  // 2. 选板，优先级：当前目标 > 也许吧 >
  //    2D上最近的（巅峰陈雪送的吐槽给你解决了哦）

  if (armors_2d_.armors.empty())
  {
    // 啥都没有，丢失计数器加一。
    if (!current_aim_.IsLost())
      current_aim_.lost_count++;
    return;
  }

  // 下面是个小技巧，用来给armors归类
  std::unordered_map<std::string, std::vector<NeArmors2D_t::Armor_t>>
      armors_2d_by_id;
  for (const auto& armor_2d : armors_2d_.armors)
    armors_2d_by_id[armor_2d.armor_id].push_back(armor_2d);

  // 看一下是否有当前目标相同的装甲板，把他们拿出来更新

  auto it = armors_2d_by_id.find(current_aim_.aim_id);
  if (it != armors_2d_by_id.end())
  {
    // 找到了，更新当前目标
    current_aim_.lost_count = 0;
    current_aim_.aim_armors.clear();
    for (const auto& armor_2d : it->second)
      current_aim_.aim_armors.push_back({.armor = armor_2d});
  }
  else
  {
    if (!current_aim_.IsLost())
    {
      current_aim_.lost_count++;
      return;
    }

    // 已经丢失了，选一个新的目标。
    double          avg_distance_min = 0;
    Eigen::Vector2d center(armors_2d_.frame_width / 2.0,
                           armors_2d_.frame_height / 2.0);

    auto it2 = armors_2d_by_id.begin();

    for (auto t_it = armors_2d_by_id.begin(); t_it != armors_2d_by_id.end();
         t_it++)
    {
      // 不可能存在空的类别

      Eigen::Vector2d avg_center(0, 0);
      for (const auto& each : t_it->second)
      {
        avg_center += each.center;
      }
      avg_center /= (double)t_it->second.size();

      const double avg_distance = avg_center.norm();
      if (avg_distance < avg_distance_min)
      {
        avg_distance_min = avg_distance;
        it2 = t_it;
      }
    }

    // 注意上面哪个东西不可能是空的
    current_aim_.lost_count = 0;
    // 更新下当前装甲板ID
    current_aim_.aim_id = armors_2d_.armors.at(0).armor_id;
    current_aim_.aim_armors.clear();
    for (const auto& armor_2d : it2->second)
      current_aim_.aim_armors.push_back({.armor = armor_2d});
  }
}

void NeTracker2D::solvePnP()
{
  const auto& obj_ps = pnp_param_.GetObjectPoints(current_aim_.aim_id);

  cv::Mat                  tvec, rvec;
  std::vector<cv::Point2d> img_points;

  for (auto& each : current_aim_.aim_armors)
  {
    // 注意顺序
    img_points.clear();
    img_points.emplace_back(each.armor.LT.x(), each.armor.LT.y());
    img_points.emplace_back(each.armor.LB.x(), each.armor.LB.y());
    img_points.emplace_back(each.armor.RB.x(), each.armor.RB.y());
    img_points.emplace_back(each.armor.RT.x(), each.armor.RT.y());

    each.pnp_is_valid = cv::solvePnP(obj_ps,
                                     img_points,
                                     pnp_param_.camera_matrix,
                                     pnp_param_.dist_coeffs,
                                     rvec,
                                     tvec,
                                     false,
                                     cv::SOLVEPNP_IPPE);
    if (each.pnp_is_valid)
    {
      each.camera_to_armor.t = Eigen::Vector3d(
          tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
      math::RvecToQuaternion(rvec, each.camera_to_armor.q);
    }
  }

  // 删掉无效的
  current_aim_.aim_armors.erase(
      std::remove_if(
          current_aim_.aim_armors.begin(),
          current_aim_.aim_armors.end(),
          [](const TrackerAimArmor_t& armor) { return !armor.pnp_is_valid; }),
      current_aim_.aim_armors.end());
}

bool NeTracker2D::matchStamp()
{
  // 与IMU的时间戳配对

  std::pair<NeImuData_t, NeImuData_t> imu_data_pair;

  if (!imu_data_c_sPtr_->FindClosestPair(
          current_aim_.cap_stamp, imu_data_pair, [](const NeImuData_t& data) {
            return data.receive_stamp;
          }))
  {
    // 尝试获取一下仅有的哪一个
    NeImuData_t imu_data_single;
    if (!imu_data_c_sPtr_->Receive(imu_data_single))
    {
      // 一个都没有
      NV_WARN("Wait for IMU data!");

      return false;
    }
    imu_data_ = imu_data_single;

    NV_WARN(
        "Only one IMU data available, use it directly without interpolation.");

    return true;
  }

  // 判断视频时间和IMU的关系，如果夹在中间，就线性插值一下，否则就用最近的那个。
  if (current_aim_.cap_stamp < imu_data_pair.first.receive_stamp)
  {
    imu_data_ = imu_data_pair.first;
    NV_WARN("Current cap_stamp is EARILER than the earliest IMU data.");
  }
  else if (current_aim_.cap_stamp > imu_data_pair.second.receive_stamp)
  {
    imu_data_ = imu_data_pair.second;
    NV_WARN("Current cap_stamp is LATER than the latest IMU data.");
  }
  else
  {
    const double ratio =
        std::chrono::duration<double>(current_aim_.cap_stamp -
                                      imu_data_pair.first.receive_stamp)
            .count() /
        std::chrono::duration<double>(imu_data_pair.second.receive_stamp -
                                      imu_data_pair.first.receive_stamp)
            .count();

    imu_data_.acc = imu_data_pair.first.acc * (1 - ratio) +
                    imu_data_pair.second.acc * ratio;
    imu_data_.gyro = imu_data_pair.first.gyro * (1 - ratio) +
                     imu_data_pair.second.gyro * ratio;
    imu_data_.quat =
        imu_data_pair.first.quat.slerp(ratio, imu_data_pair.second.quat);
  }
  return true;
}

void NeTracker2D::transformToImuFrame()
{
  for (auto& each : current_aim_.aim_armors)
  {
    each.imu_to_armor.q =
        imu_data_.quat * gimbal_to_camera_.q * each.camera_to_armor.q;
    each.imu_to_armor.t =
        imu_data_.quat *
        (gimbal_to_camera_.q * each.camera_to_armor.t + gimbal_to_camera_.t);
  }
}

void NeTracker2D::yawOptimize()
{
  // 上交的那种yaw优化
  for (auto& each : current_aim_.aim_armors)
  {
    // 先计算修正畸变后的装甲板四个点坐标
    std::vector<cv::Point2d> img_points{math::EigenVec2dToCv(each.armor.LT),
                                        math::EigenVec2dToCv(each.armor.LB),
                                        math::EigenVec2dToCv(each.armor.RB),
                                        math::EigenVec2dToCv(each.armor.RT)};
    // 只有当需要高精度去畸变时才开启完整undistortPoints，或者考虑近似去畸变
    // 这里保留原逻辑但需注意耗时
    cv::undistortPoints(img_points,
                        img_points,
                        pnp_param_.camera_matrix,
                        pnp_param_.dist_coeffs,
                        cv::noArray(),
                        pnp_param_.camera_matrix);

    // 更新复用的数据结构
    for (size_t i = 0; i < 4; ++i)
    {
      yaw_optimize_.img_points[i] =
          Eigen::Vector2d(img_points[i].x, img_points[i].y);
    }

    yaw_optimize_.obj_points =
        pnp_param_.GetObjectPointsEigen(current_aim_.aim_id);
    yaw_optimize_.q_c_i =
        gimbal_to_camera_.q.conjugate() * imu_data_.quat.conjugate();
    yaw_optimize_.t_c_a = each.camera_to_armor.t;
    yaw_optimize_.fixed_pitch_rad = current_aim_.aim_id == "outpost"
                                        ? math::DegToRad(-15)
                                        : math::DegToRad(15);
    yaw_optimize_.yaw = 0.0; // 初始猜测

    // 复用 Problem 进行求解
    ceres::Solver::Summary summary;
    ceres::Solve(yaw_optimize_.options, yaw_optimize_.problem.get(), &summary);

    double optimized_yaw = math::WrapToPi(yaw_optimize_.yaw);

    Eigen::Quaterniond q_i_a =
        Eigen::AngleAxisd(optimized_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(yaw_optimize_.fixed_pitch_rad,
                          Eigen::Vector3d::UnitY());

    // 更新一下imu_to_armor
    each.imu_to_armor.q = q_i_a;
    each.imu_to_armor.t =
        imu_data_.quat *
        (gimbal_to_camera_.q * each.camera_to_armor.t + gimbal_to_camera_.t);
    each.imu_to_armor.yaw.exp(optimized_yaw);
  }
}

void NeTracker2D::optimize()
{
  // 自己造的一种优化方式，将tvec也优化进去，可以得到协方差。

  for (auto& each : current_aim_.aim_armors)
  {
    // 先计算修正畸变后的装甲板四个点坐标
    std::vector<cv::Point2d> img_points{math::EigenVec2dToCv(each.armor.LT),
                                        math::EigenVec2dToCv(each.armor.LB),
                                        math::EigenVec2dToCv(each.armor.RB),
                                        math::EigenVec2dToCv(each.armor.RT)};
    cv::undistortPoints(img_points,
                        img_points,
                        pnp_param_.camera_matrix,
                        pnp_param_.dist_coeffs,
                        cv::noArray(),
                        pnp_param_.camera_matrix);

    // 更新复用数据
    for (size_t i = 0; i < 4; ++i)
    {
      yaw_optimize_.img_points[i] =
          Eigen::Vector2d(img_points[i].x, img_points[i].y);
    }
    yaw_optimize_.obj_points =
        pnp_param_.GetObjectPointsEigen(current_aim_.aim_id);
    yaw_optimize_.fixed_pitch_rad = current_aim_.aim_id == "outpost"
                                        ? math::DegToRad(-15)
                                        : math::DegToRad(15);
    yaw_optimize_.q_g_c = gimbal_to_camera_.q;
    yaw_optimize_.q_i_g = imu_data_.quat;
    yaw_optimize_.t_g_c = gimbal_to_camera_.t;

    // pnp结果直接当yaw的初值，收敛更快
    yaw_optimize_.yaw = math::QuaternionToYaw(each.imu_to_armor.q);

    // nv_rec_g().log("tracker_2d_yaw_before_optimize",
    //                rerun::Scalars(yaw_optimize_.yaw));

    // 给下t的初始值
    yaw_optimize_.t = each.imu_to_armor.t;

    ceres::Solver::Summary summary;
    // 开算
    ceres::Solve(
        yaw_optimize_.options, yaw_optimize_.problem_full.get(), &summary);

    double optimized_yaw = math::WrapToPi(yaw_optimize_.yaw);

    Eigen::Quaterniond q_i_a =
        Eigen::AngleAxisd(optimized_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(yaw_optimize_.fixed_pitch_rad,
                          Eigen::Vector3d::UnitY());

    // 更新一下imu_to_armor
    each.imu_to_armor.q = q_i_a;
    each.imu_to_armor.t = yaw_optimize_.t;
    each.imu_to_armor.yaw = Sophus::SO2d::exp(optimized_yaw);
  }
}

void NeTracker2D::reprojectAndFillDebugInfo()
{
  for (auto& each : current_aim_.aim_armors)
  {
    Eigen::Quaterniond q_c_i =
        gimbal_to_camera_.q.conjugate() * imu_data_.quat.conjugate();

    // 计算相机到装甲板的旋转
    Eigen::Quaterniond q_c_a = q_c_i * each.imu_to_armor.q;

    // 计算相机到装甲板平移
    auto t_c_a = q_c_i * each.imu_to_armor.t -
                 gimbal_to_camera_.q.conjugate() * gimbal_to_camera_.t;

    Eigen::Vector3d P_LT =
        q_c_a * pnp_param_.GetObjectPointsEigen(current_aim_.aim_id)[0] + t_c_a;
    Eigen::Vector3d P_LB =
        q_c_a * pnp_param_.GetObjectPointsEigen(current_aim_.aim_id)[1] + t_c_a;
    Eigen::Vector3d P_RB =
        q_c_a * pnp_param_.GetObjectPointsEigen(current_aim_.aim_id)[2] + t_c_a;
    Eigen::Vector3d P_RT =
        q_c_a * pnp_param_.GetObjectPointsEigen(current_aim_.aim_id)[3] + t_c_a;

    P_LT = (pnp_param_.camera_matrix_eigen * P_LT) / P_LT.z();
    P_LB = (pnp_param_.camera_matrix_eigen * P_LB) / P_LB.z();
    P_RB = (pnp_param_.camera_matrix_eigen * P_RB) / P_RB.z();
    P_RT = (pnp_param_.camera_matrix_eigen * P_RT) / P_RT.z();

    cv::Point2d P_LT_2d(P_LT.x(), P_LT.y());
    cv::Point2d P_LB_2d(P_LB.x(), P_LB.y());
    cv::Point2d P_RB_2d(P_RB.x(), P_RB.y());
    cv::Point2d P_RT_2d(P_RT.x(), P_RT.y());

    each.debug_info.re_projected_pts.clear();
    each.debug_info.re_projected_pts.push_back(P_LT_2d);
    each.debug_info.re_projected_pts.push_back(P_LB_2d);
    each.debug_info.re_projected_pts.push_back(P_RB_2d);
    each.debug_info.re_projected_pts.push_back(P_RT_2d);
  }
}
} // namespace ne_vision
