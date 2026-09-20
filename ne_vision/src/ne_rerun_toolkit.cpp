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
// 工具包单例实现

#include <cmath>
#include <algorithm>
#include <opencv2/calib3d.hpp>

#include "ne_vision/debug/ne_rerun_toolkit.hpp"
#include "rerun/recording_stream.hpp"

namespace ne_vision
{
namespace
{
bool ValidIntrinsics(const Eigen::Matrix3d& K)
{
  return K.cast<float>().allFinite() && K(0, 0) > 0.0 && K(1, 1) > 0.0 &&
         K(0, 1) == 0.0 && K(1, 0) == 0.0 && K(2, 0) == 0.0 && K(2, 1) == 0.0 &&
         K(2, 2) == 1.0;
}

rerun::datatypes::Vec3D ToRerun(const Eigen::Vector3d& value)
{
  return {static_cast<float>(value.x()),
          static_cast<float>(value.y()),
          static_cast<float>(value.z())};
}

rerun::Quaternion ToRerun(const Eigen::Quaterniond& rotation)
{
  const auto q = rotation.normalized();
  return rerun::Quaternion::from_xyzw(static_cast<float>(q.x()),
                                      static_cast<float>(q.y()),
                                      static_cast<float>(q.z()),
                                      static_cast<float>(q.w()));
}

std::string TFPath(const std::string& parent, const std::string& child)
{
  if (parent.empty() || child.empty() || child.find('/') != std::string::npos ||
      child == "." || child == "..")
  {
    NV_WARN(
        "Invalid TF path: parent must be a full path and child a node name.");
    return {};
  }
  const auto end = parent.find_last_not_of('/');
  return (end == std::string::npos ? "" : parent.substr(0, end + 1)) + "/" +
         child;
}

bool ValidGeometry(const std::string&        path,
                   const Eigen::Vector3d&    center,
                   const Eigen::Quaterniond& rotation,
                   const Eigen::Vector3d&    size)
{
  if (center.cast<float>().allFinite() && size.cast<float>().allFinite() &&
      (size.array() > 0.0).all() && rotation.coeffs().allFinite() &&
      std::isfinite(rotation.norm()) && rotation.norm() > 0.0)
    return true;

  NV_WARN("Invalid geometry at {}: check center, rotation and positive size.",
          path);
  return false;
}
} // namespace

NeRerunToolkit& NeRerunToolkit::Instance()
{
  static NeRerunToolkit instance;
  return instance;
}

bool NeRerunToolkit::Enable(const RecType_e&   rec_type,
                            const std::string& path,
                            const std::string& name)
{
  if (is_enabled_ && rec_uptr_ != nullptr)
    return true;

  timeline_name_ = name + "_clock";

  std::string real_path;
  if (path != "")
    real_path = path;
  else if (rec_type == RecType_e::ONLINE)
    real_path = "0.0.0.0:9876";
  else
    real_path = name + "_log.rrd";

  if (rec_type == RecType_e::ONLINE)
  {
    rec_uptr_ = std::make_unique<rerun::RecordingStream>(name);
    if (!rec_uptr_->connect_grpc(real_path).is_ok())
    {
      NV_WARN("Failed to connect to Rerun server at {}!", real_path);
      is_enabled_ = false;
      return false;
    }
    else
    {
      NV_INFO("Connected to Rerun server at {}!", real_path);
      is_enabled_ = true;
      return true;
    }
  }
  else
  {
    rec_uptr_ = std::make_unique<rerun::RecordingStream>(name);
    if (!rec_uptr_->save(real_path).is_ok())
    {
      NV_WARN("Failed to connect to Rerun file at {}!", real_path);
      is_enabled_ = false;
      return false;
    }
    else
    {
      NV_INFO("Log file will save at {}!", real_path);
      is_enabled_ = true;
      return true;
    }
  }
}

void NeRerunToolkit::LogCvMat(const std::string&                    path,
                              const cv::Mat&                        mat,
                              std::chrono::steady_clock::time_point stamp)
{
  if (!IsEnabled())
    return;

  if (mat.empty() || mat.dims != 2)
  {
    NV_WARN("Cannot log empty or non-2D image at {}!", path);
    return;
  }

  rerun::ColorModel color_model;
  switch (mat.type())
  {
  case CV_8UC1: color_model = rerun::ColorModel::L; break;
  case CV_8UC3: color_model = rerun::ColorModel::BGR; break;
  case CV_8UC4: color_model = rerun::ColorModel::BGRA; break;
  default:
    NV_WARN("Unsupported image type {} at {}; expected CV_8UC1/3/4!",
            mat.type(),
            path);
    return;
  }

  // Rerun 接收紧密排列的像素；ROI 可能带有额外行步长。
  const cv::Mat image = mat.isContinuous() ? mat : mat.clone();
  rec_uptr_->set_time_duration(timeline_name_, stamp.time_since_epoch());
  rec_uptr_->log(path,
                 rerun::Image(image.ptr<unsigned char>(),
                              {static_cast<uint32_t>(image.cols),
                               static_cast<uint32_t>(image.rows)},
                              color_model));
}

void NeRerunToolkit::LogLines2D(
    const std::string&                           path,
    const std::vector<std::vector<Eigen::Vector2d>>& groups,
    rerun::Color                                 color,
    float                                        line_width,
    std::chrono::steady_clock::time_point         stamp)
{
  if (!IsEnabled())
    return;

  std::vector<rerun::LineStrip2D> strips;
  strips.reserve(groups.size());
  for (const auto& group : groups)
  {
    if (group.size() < 2)
      continue;

    std::vector<rerun::Vec2D> points;
    points.reserve(group.size());
    for (const auto& point : group)
      points.emplace_back(static_cast<float>(point.x()),
                          static_cast<float>(point.y()));

    strips.emplace_back(std::move(points));
  }

  rec_uptr_->set_time_duration(timeline_name_, stamp.time_since_epoch());
  if (strips.empty())
  {
    rec_uptr_->log(path, rerun::Clear::RECURSIVE);
    return;
  }

  if (!std::isfinite(line_width) || line_width <= 0.0f)
  {
    NV_WARN("Invalid line width at {}: expected a finite positive value.", path);
    return;
  }

  rec_uptr_->log(path,
                 rerun::LineStrips2D(std::move(strips))
                     .with_colors({color})
                     .with_radii({line_width * 0.5f}));
}

void NeRerunToolkit::LogCvMatUndistorted(
    const std::string&                    path,
    const cv::Mat&                        mat,
    const Eigen::Matrix3d&                K,
    const std::vector<double>&            distortion,
    std::chrono::steady_clock::time_point stamp)
{
  if (!IsEnabled())
    return;

  const auto n = distortion.size();
  if (!ValidIntrinsics(K) ||
      !(n == 0 || n == 4 || n == 5 || n == 8 || n == 12 || n == 14) ||
      !std::all_of(distortion.begin(), distortion.end(), [](double v) {
        return std::isfinite(v);
      }))
  {
    NV_WARN("Invalid camera intrinsics or distortion at {}!", path);
    return;
  }
  if (mat.empty() || mat.dims != 2 ||
      (mat.type() != CV_8UC1 && mat.type() != CV_8UC3 && mat.type() != CV_8UC4))
  {
    NV_WARN("Expected a non-empty 2D CV_8UC1/3/4 image at {}!", path);
    return;
  }
  if (distortion.empty())
  {
    LogCvMat(path, mat, stamp);
    return;
  }

  // 显式逐元素转换，避免 Eigen 列主序与 OpenCV 行主序混淆。
  const cv::Matx33d camera_matrix(K(0, 0),
                                  K(0, 1),
                                  K(0, 2),
                                  K(1, 0),
                                  K(1, 1),
                                  K(1, 2),
                                  K(2, 0),
                                  K(2, 1),
                                  K(2, 2));
  cv::Mat           image;
  cv::undistort(mat, image, camera_matrix, distortion, camera_matrix);
  LogCvMat(path, image, stamp);
}

void NeRerunToolkit::LogStaticPinhole(const std::string&     path,
                                      const Eigen::Matrix3d& K,
                                      int                    width,
                                      int                    height)
{
  if (!IsEnabled())
    return;
  if (!ValidIntrinsics(K) || width <= 0 || height <= 0)
  {
    NV_WARN("Invalid camera intrinsics or image size at {}!", path);
    return;
  }

  const Eigen::Matrix3f matrix = K.cast<float>();
  rec_uptr_->log_static(
      path,
      rerun::Pinhole(rerun::components::PinholeProjection(
                         rerun::datatypes::Mat3x3(matrix.data())))
          .with_resolution(width, height)
          .with_camera_xyz(rerun::components::ViewCoordinates::RDF));
}

void NeRerunToolkit::LogTF(const std::string&                    parent,
                           const std::string&                    child,
                           const Eigen::Quaterniond&             rotation,
                           const Eigen::Vector3d&                translation,
                           std::chrono::steady_clock::time_point stamp)
{
  if (!IsEnabled())
    return;

  const auto path = TFPath(parent, child);
  if (path.empty() ||
      !ValidGeometry(path, translation, rotation, Eigen::Vector3d::Ones()))
    return;

  rec_uptr_->set_time_duration(timeline_name_, stamp.time_since_epoch());
  rec_uptr_->log(path,
                 rerun::Transform3D(ToRerun(translation), ToRerun(rotation)));
}

void NeRerunToolkit::LogStaticTF(const std::string&        parent,
                                 const std::string&        child,
                                 const Eigen::Quaterniond& rotation,
                                 const Eigen::Vector3d&    translation)
{
  if (!IsEnabled())
    return;

  const auto path = TFPath(parent, child);
  if (path.empty() ||
      !ValidGeometry(path, translation, rotation, Eigen::Vector3d::Ones()))
    return;

  rec_uptr_->log_static(
      path, rerun::Transform3D(ToRerun(translation), ToRerun(rotation)));
}

NeRerunToolkit::NeRerunToolkit() = default;

NeRerunToolkit::~NeRerunToolkit() { Disable(); };

} // namespace ne_vision
