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

#include <algorithm>
#include <chrono>
#include <vector>

// #include "ceres/types.h"

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/AngleAxis.h"
#include "Eigen/src/Geometry/Quaternion.h"

#include "ne_vision/utils/ne_debug.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"

#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_param.hpp"
#include "ne_vision/utils/ne_math.hpp"
#include "ne_vision/utils/ne_code_profiler.hpp"
#include "ne_vision/utils/ne_rerun_debug.hpp"
#include "ne_vision/ne_channals.hpp"

#define BEBUG_LOG

namespace ne_vision
{

NeTracker2D::NeTracker2D(const std::string& name)
    : name_(name), armors_2d_c_sPtr_(NV_CHANNELS.armor2d_sPtr()),
      imu_data_c_sPtr_(NV_CHANNELS.imu_data_sPtr()),
      armors_3d_c_sPtr_(NV_CHANNELS.armor3d_sPtr())
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

  detector_variance_ =
      NV_PARAM["auto_aim"]["tracker_2d"]["detector_variance"].as<double>();

  // 初始化current aim 的计数器，令其大于最大值，确保第一次识别可以初始化
  current_aim_.lost_count = current_aim_.lost_count_threshold + 10;
}

void NeTracker2D::Tarck2D()
{

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
  {
    // 虽然之前应该是能确保清零了，这一再做一下防呆
    // 清零代表没识别到，用来给后续做处理的
    current_aim_.aim_armors.clear();
    goto send; // 只要本次没识别到，就退出了
  }

  // 能到这里，armors_2d不可能是空的
  solvePnP();

  current_aim_.cap_stamp = armors_2d_.cap_stamp;

  if (current_aim_.aim_armors.empty())
    goto send; // 可能全都解算失败了？

  if (!matchStamp())
    goto send;

  transformToImuFrame();
  lmOptimize();

  // ONLY FOR DEBUG
  reprojectAndFillDebugInfo();

  // 循环填数据
  for (auto& each : current_aim_.aim_armors)
  {
    NeArmors3D_t::Armor3D_t armor_3d;
    armor_3d.debug = each.debug_info;
    armor_3d.q = each.imu_to_armor.q;
    armor_3d.t = each.imu_to_armor.t;
    // 坐标系变换
    armor_3d.yaw = math::WrapToPi(each.imu_to_armor.yaw + M_PI);
    armor_3d.cov = each.imu_to_armor.cov;
    armors_3d_.armors.push_back(armor_3d);
  }
  armors_3d_.aim_id = current_aim_.aim_id;

  // 无论是谁不对，都不能阻止发数据，因为会影响可视化配对，以及后续各类时间戳
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

  // 无论如何先清零
  current_aim_.aim_armors.clear();

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

void NeTracker2D::reprojectAndFillDebugInfo()
{
  for (auto& each : current_aim_.aim_armors)
  {
    // 相机到IMU的旋转
    each.debug_info.camera_to_imu.q =
        gimbal_to_camera_.q.conjugate() * imu_data_.quat.conjugate();

    // 相机到IMU的平移
    each.debug_info.camera_to_imu.t =
        gimbal_to_camera_.q.conjugate() * gimbal_to_camera_.t;

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
// 实现原理见docs：pnp_optimize_and_cov.md
// 这个函数是自己写的基础运算，发现贼慢，然后丢进去AI优化
// 所以很多东西我也不知道为啥这么写的，别问我啊。
// 我电脑测试结果大概一个装甲板1ms多，太慢就不对咯
void NeTracker2D::lmOptimize()
{
  NV_PROFILE_BLOCK("Tracker2D LM Optimize");

  constexpr int    kMaxN = 5;         // 外层最大循环次数
  constexpr int    kMaxInnerIter = 4; // 内层尝试调整阻尼的最大次数
  constexpr double kEpsilon = 1e-6;
  constexpr double kVarr = 0.008;
  constexpr double kVarp = 0.008;
  constexpr double kTargetRoll = 0.0;

  // 提取畸变参数
  std::vector<double> D;
  if (!pnp_param_.dist_coeffs.empty())
    pnp_param_.dist_coeffs.convertTo(D, CV_64F);
  const double k1 = D.size() > 0 ? D[0] : 0.0;
  const double k2 = D.size() > 1 ? D[1] : 0.0;
  const double p1 = D.size() > 2 ? D[2] : 0.0;
  const double p2 = D.size() > 3 ? D[3] : 0.0;
  const double k3 = D.size() > 4 ? D[4] : 0.0;

  // 理论固定pitch
  const double fixed_pitch_rad = (current_aim_.aim_id == "outpost")
                                     ? math::DegToRad(-15)
                                     : math::DegToRad(15);

  for (auto& each : current_aim_.aim_armors)
  {

    double fx = pnp_param_.camera_matrix_eigen(0, 0);
    double fy = pnp_param_.camera_matrix_eigen(1, 1);
    double cx = pnp_param_.camera_matrix_eigen(0, 2);
    double cy = pnp_param_.camera_matrix_eigen(1, 2);

    // 手动去畸变
    std::vector<Eigen::Vector2d> P_uv_hat_s;
    P_uv_hat_s.reserve(4);

    std::vector<Eigen::Vector2d> raw_pts{
        each.armor.LT, each.armor.LB, each.armor.RB, each.armor.RT};

    // AI 的，懂不了一点
    for (const auto& pt : raw_pts)
    {
      double x0 = (pt.x() - cx) / fx;
      double y0 = (pt.y() - cy) / fy;
      double x = x0, y = y0;

      for (int j = 0; j < 5; ++j)
      {
        double r2 = x * x + y * y;
        double icdist =
            1.0 / (1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
        double deltaX = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
        double deltaY = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
        x = (x0 - deltaX) * icdist;
        y = (y0 - deltaY) * icdist;
      }
      P_uv_hat_s.emplace_back(x * fx + cx, y * fy + cy);
    }

    // 构建W信息矩阵（来自外部参数），也可能来自模型推理结果+外参补偿，这里先写在循环内
    Eigen::DiagonalMatrix<double, 10> W;

    double var_p =
        std::max(detector_variance_, 0.00001); // 位置测量方差，来自外部参数

    // 位置残差权重
    for (int k = 0; k < 8; ++k)
      W.diagonal()[k] = 1.0 / var_p;

    W.diagonal()[8] = 1.0 / kVarr; // roll残差权重，来自外部参数
    W.diagonal()[9] = 1.0 / kVarp; // pitch残差权重，来自外部参数

    std::vector<Eigen::Vector3d>& P_a_s =
        pnp_param_.GetObjectPointsEigen(current_aim_.aim_id);

    Eigen::Matrix3d R_c_i =
        (imu_data_.quat * gimbal_to_camera_.q).conjugate().matrix();
    Eigen::Matrix3d R_c_g = gimbal_to_camera_.q.conjugate().matrix();
    Eigen::Vector3d const_t_offset = R_c_g * gimbal_to_camera_.t;

    // nv_rec_g().log("tracker_2d_initial_yaw",
    //                rerun::Scalars(math::QuaternionToYaw(each.imu_to_armor.q)));

    // nv_rec_g().log("initial_x", rerun::Scalars(each.imu_to_armor.t.x()));

    Eigen::Vector<double, 6> x;
    x.head<3>() = each.imu_to_armor.t;
    x(3) = kTargetRoll;
    x(4) = fixed_pitch_rad;
    x(5) = math::QuaternionToYaw(each.imu_to_armor.q);

    // 只算残差的
    auto eval_residual = [&](const Eigen::Vector<double, 6>& state_x,
                             Eigen::Matrix<double, 10, 1>&   res) -> bool {
      double cr = std::cos(state_x(3)), sr = std::sin(state_x(3));
      double cp = std::cos(state_x(4)), sp = std::sin(state_x(4));
      double cy_ = std::cos(state_x(5)), sy = std::sin(state_x(5));

      Eigen::Matrix3d Rx, Ry, Rz;
      Rx << 1, 0, 0, 0, cr, -sr, 0, sr, cr;
      Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
      Rz << cy_, -sy, 0, sy, cy_, 0, 0, 0, 1;

      Eigen::Matrix3d R_c_a = R_c_i * (Rz * Ry * Rx);
      Eigen::Vector3d t_base = R_c_i * state_x.head<3>() - const_t_offset;

      for (size_t j = 0; j < 4; ++j)
      {
        Eigen::Vector3d P_c = R_c_a * P_a_s[j] + t_base;
        if (std::abs(P_c.z()) < 1e-6)
          return false;

        double inv_z = 1.0 / P_c.z();
        res(2 * j) = P_uv_hat_s[j].x() - (fx * P_c.x() * inv_z + cx);
        res(2 * j + 1) = P_uv_hat_s[j].y() - (fy * P_c.y() * inv_z + cy);
      }
      res(8) = kTargetRoll - state_x(3);
      res(9) = fixed_pitch_rad - state_x(4);
      return true;
    };

    // 算残差和雅可比
    auto eval_residual_and_jac =
        [&](const Eigen::Vector<double, 6>& state_x,
            Eigen::Matrix<double, 10, 1>&   res,
            Eigen::Matrix<double, 10, 6>&   jac) -> bool {
      double cr = std::cos(state_x(3)), sr = std::sin(state_x(3));
      double cp = std::cos(state_x(4)), sp = std::sin(state_x(4));
      double cy_ = std::cos(state_x(5)), sy = std::sin(state_x(5));

      Eigen::Matrix3d Rx, Ry, Rz;
      Rx << 1, 0, 0, 0, cr, -sr, 0, sr, cr;
      Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
      Rz << cy_, -sy, 0, sy, cy_, 0, 0, 0, 1;

      Eigen::Matrix3d R_c_a = R_c_i * (Rz * Ry * Rx);
      Eigen::Vector3d t_base = R_c_i * state_x.head<3>() - const_t_offset;

      Eigen::Matrix3d dRxdr, dRydp, dRzdy;
      dRxdr << 0, 0, 0, 0, -sr, -cr, 0, cr, -sr;
      dRydp << -sp, 0, cp, 0, 0, 0, -cp, 0, -sp;
      dRzdy << -sy, -cy_, 0, cy_, -sy, 0, 0, 0, 0;

      Eigen::Matrix3d R_c_i_dRidr = R_c_i * (Rz * Ry * dRxdr);
      Eigen::Matrix3d R_c_i_dRidp = R_c_i * (Rz * dRydp * Rx);
      Eigen::Matrix3d R_c_i_dRidy = R_c_i * (dRzdy * Ry * Rx);

      for (size_t j = 0; j < 4; ++j)
      {
        Eigen::Vector3d P_c = R_c_a * P_a_s[j] + t_base;

        // 除0预防
        if (std::abs(P_c.z()) < 1e-6)
          return false;

        double inv_z = 1.0 / P_c.z();
        double inv_z2 = inv_z * inv_z;

        res(2 * j) = P_uv_hat_s[j].x() - (fx * P_c.x() * inv_z + cx);
        res(2 * j + 1) = P_uv_hat_s[j].y() - (fy * P_c.y() * inv_z + cy);

        Eigen::Matrix<double, 2, 3> Jc;
        Jc << fx * inv_z, 0, -fx * P_c.x() * inv_z2, 0, fy * inv_z,
            -fy * P_c.y() * inv_z2;

        Eigen::Matrix<double, 3, 6> Jp;
        Jp.block<3, 3>(0, 0) = R_c_i;
        Jp.block<3, 1>(0, 3) = R_c_i_dRidr * P_a_s[j];
        Jp.block<3, 1>(0, 4) = R_c_i_dRidp * P_a_s[j];
        Jp.block<3, 1>(0, 5) = R_c_i_dRidy * P_a_s[j];

        jac.block<2, 6>(2 * j, 0) = -(Jc * Jp);
      }

      res(8) = kTargetRoll - state_x(3);
      res(9) = fixed_pitch_rad - state_x(4);

      jac.block<2, 6>(8, 0).setZero();
      jac(8, 3) = -1.0;
      jac(9, 4) = -1.0;

      return true;
    };

    /* === 具体算法实现部分 === */
    double                       lambda = 1e-3;
    Eigen::Matrix<double, 10, 1> r;
    Eigen::Matrix<double, 10, 6> J;

    if (!eval_residual(x, r))
    {
      continue;
    }

    double current_cost = 0.5 * r.transpose() * W * r;

    for (int i = 0; i < kMaxN; ++i)
    {
      eval_residual_and_jac(x, r, J);

      Eigen::Matrix<double, 6, 6> H = J.transpose() * W * J;
      Eigen::Matrix<double, 6, 1> g = -J.transpose() * W * r;

      // AI 的 梯度早退机制 (Early Exit)
      // 如果梯度各分量的绝对值最大项已经极其微小，说明已经到达局部最优解，直接终止
      if (g.lpNorm<Eigen::Infinity>() < 1e-6)
        break;

      bool                     step_accepted = false;
      Eigen::Vector<double, 6> delta;

      // 内循环得到合适的阻尼值，LM算法比较重要的一点
      for (int ii = 0; ii < kMaxInnerIter; ++ii)
      {
        Eigen::Matrix<double, 6, 6> H_lm = H;

        // Marquardt 尺度自适应
        Eigen::Vector<double, 6> h_diag = H.diagonal();
        H_lm.diagonal().array() += lambda * (h_diag.array() + 1e-5);

        // 对于H这正定的，llt比较快
        delta = H_lm.llt().solve(g);

        if (delta.hasNaN() || delta.norm() > 1e6)
          break;
        if (delta.norm() < kEpsilon)
        {
          step_accepted = true;
          break;
        }

        Eigen::Vector<double, 6>     x_new = x + delta;
        Eigen::Matrix<double, 10, 1> r_new;

        if (!eval_residual(x_new, r_new))
        {
          lambda *= 10.0;
          continue;
        }

        double new_cost = 0.5 * r_new.transpose() * W * r_new;

        if (new_cost < current_cost)
        {
          x = x_new;
          current_cost = new_cost;
          lambda = std::max(1e-7, lambda / 10.0);
          step_accepted = true;
          break;
        }
        else
        {
          lambda *= 10.0;
        }
      }

      if (delta.hasNaN() || delta.norm() > 1e6)
        break;
      if (delta.norm() < kEpsilon || !step_accepted)
        break;
    }

    each.imu_to_armor.t = x.head<3>();
    each.imu_to_armor.yaw = math::WrapToPi(x(5));
    each.imu_to_armor.q = Eigen::AngleAxisd(x(5), Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(x(4), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(x(3), Eigen::Vector3d::UnitX());
    // nv_rec_g().log("tracker_2d_yaw_after_lm_optimize", rerun::Scalars(x(5)));

    // 计算当前最优的 J 和 r
    eval_residual_and_jac(x, r, J);

    // 最终的加权海森矩阵 H
    Eigen::Matrix<double, 6, 6> H_final = J.transpose() * W * J;

    // 3. 求逆得到协方差
    Eigen::Matrix<double, 6, 6> Cov_6x6 =
        H_final.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

    // 4. 提取 [x, y, z, yaw] 协方差 (0, 1, 2, 5)
    Eigen::Matrix<double, 4, 4> Cov_4x4;
    std::vector<int>            idx = {0, 1, 2, 5};
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        Cov_4x4(i, j) = Cov_6x6(idx[i], idx[j]);
      }
    }

    // TODO: 保护内容
    // if (Cov_4x4.hasNaN() || Cov_4x4(2, 2) > 1.0 || Cov_4x4(3, 3) > 1.0)
    // {
    //   NV_WARN("LM Covariance Exploded! Fallback to default large
    //   Covariance."); Cov_4x4.setIdentity(); Cov_4x4(0, 0) = 0.05; Cov_4x4(1,
    //   1) = 0.05; Cov_4x4(2, 2) = 0.2; Cov_4x4(3, 3) = 0.5; // Yaw
    //   角给极度不信任状态
    // }
    // else
    // {
    //   Eigen::Matrix<double, 4, 4> Cov_base =
    //       Eigen::Matrix<double, 4, 4>::Zero();
    //   Cov_base(0, 0) = 1e-4; // X轴底盘/云台机械底线震动噪声
    //   Cov_base(1, 1) = 1e-4; // Y轴
    //   Cov_base(2, 2) = 4e-4; // Z轴 (深度先天误差通常较大)
    //   Cov_base(3, 3) = 1e-3; // Yaw角底线噪声

    //   Cov_4x4 += Cov_base;
    // }

    each.imu_to_armor.cov = Cov_4x4;
    // double yaw_var = Cov_4x4(3, 3);
    // double yaw_error_max = x(5) + std::sqrt(yaw_var) * 3; // 3-sigma原则
    // double yaw_error_min = x(5) - std::sqrt(yaw_var) * 3;
    // nv_rec_g().log("tracker_2d_yaw_error_bound_max",
    //                rerun::Scalars(yaw_error_max));
    // nv_rec_g().log("tracker_2d_yaw_error_bound_min",
    //                rerun::Scalars(yaw_error_min));

    // double xxx = x(0);
    // double xxx_var = Cov_4x4(0, 0);
    // double xxx_error_max = xxx + std::sqrt(xxx_var) * 3;
    // double xxx_error_min = xxx - std::sqrt(xxx_var) * 3;
    // nv_rec_g().log("x_error_bound_max", rerun::Scalars(xxx_error_max));
    // nv_rec_g().log("x_error_bound_min", rerun::Scalars(xxx_error_min));
    // nv_rec_g().log("optimized_x", rerun::Scalars(xxx));
  }

  // auto ms =
  //     NV_PROFILE_INSTANCE("Tracker2D LM Optimize")->GetResult().GetCurrentS()
  //     * 1000.0;
  // NV_DEBUG("Tracker2D LM Optimize took {:.2f} ms on average", ms);
}
} // namespace ne_vision
