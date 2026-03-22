#include "ne_vision/tracker/ne_tracker_3d.hpp"
#include "ne_vision/models/ne_sion_model.hpp"
#include "ne_vision/utils/ne_code_profiler.hpp"
#include "ne_vision/utils/ne_debug.hpp"
#include "ne_vision/utils/ne_log.hpp"

#define IS_SION_MODEL(aim_str)                                                 \
  (aim_str == "1" || aim_str == "2" || aim_str == "3" || aim_str == "4" ||     \
   aim_str == "7")

// 其他模型

namespace ne_vision
{

NeTracker3D::NeTracker3D(const std::string&       name,
                         const NeArmors3DCSPtr_t& armors_3d_c_sPtr,
                         const NeImuDataCSPtr_t&  imu_data_c_sPtr,
                         const NeAimTrajCSPtr_t&  aim_traj_c_sPtr)
    : name_(name), armors_3d_c_sPtr_(armors_3d_c_sPtr),
      imu_data_c_sPtr_(imu_data_c_sPtr), aim_traj_c_sPtr_(aim_traj_c_sPtr)
{
  NV_ASSERT(armors_3d_c_sPtr_ != nullptr && imu_data_c_sPtr_ != nullptr &&
            aim_traj_c_sPtr_ != nullptr &&
            "armors_3d_c_sPtr_, imu_data_c_sPtr_, and aim_traj_c_sPtr_ cannot "
            "be nullptr");

  last_cap_stamp_ = std::chrono::steady_clock::now();
}

void NeTracker3D::Track()
{

  // NV_PROFILE_BLOCK(GetName());

  NeAimTraj_t aim_traj_o;

  NeArmors3D_t armors_3d_i_;
  NeImuData_t  imu_data_i_;

  if (!armors_3d_c_sPtr_->Receive(armors_3d_i_))
  {
    NV_WARN("No 3D armor data received, skipping tracking");
    return;
  }

  // TODO: 考虑时间同步
  if (!imu_data_c_sPtr_->Receive(imu_data_i_, true))
  {
    NV_WARN("No IMU data received, skipping tracking");
    return;
  }

  // 0. 检查当前收到的装甲板数据是否是新的，如果不是新的，当作没有收到
  if (armors_3d_i_.cap_stamp <= last_cap_stamp_)
  {
    armors_3d_i_.armors.clear(); // 清空装甲板数据，当作没有收到
  }
  else
  {
    last_cap_stamp_ =
        armors_3d_i_.cap_stamp; // 更新最后接收的装甲板的拍摄时间戳
  }

  // 1. 检查当前目标是否跟丢
  // 注意，虽然cap_stamp和now使用的是不同的时间基准，但是由于算法延迟和传输延迟是ms级的
  // 对是否跟踪丢失判断影响不大，因此这里这么写
  const auto   now = std::chrono::steady_clock::now();
  const double lose_time =
      std::chrono::duration<double>(now - last_cap_stamp_).count();
  if (lose_time > param_.lose_time && armors_3d_i_.armors.empty())
  {
    current_tracking_aim_ = "NULL";
    model_ = std::monostate(); // 清空模型
    goto SEND;
  }

  // 2.如果还没有跟丢，在没识别到，或者识别到的还是当前目标，就继续跟踪
  if (armors_3d_i_.armors.empty() ||
      armors_3d_i_.aim_id == current_tracking_aim_)
  {
    goto DO_NORMAL_TRACKING;
  }

  // 3. 如果目标变化了，这里创建新的跟踪器
  if (current_tracking_aim_ != armors_3d_i_.aim_id)
  {
    if (IS_SION_MODEL(armors_3d_i_.aim_id))
    {
      // Sion
      model_ = sion::NeSionModel(armors_3d_i_);
    }
    else
    {
      NV_WARN("No tracer handle this type of aim: {}", armors_3d_i_.aim_id);
    }

    // 更新当前跟踪的目标
    current_tracking_aim_ = armors_3d_i_.aim_id;
  }

  // 4. 进行正常一次跟踪
DO_NORMAL_TRACKING:
  if (IS_SION_MODEL(current_tracking_aim_))
  {
    try
    {
      auto& sion = std::get<sion::NeSionModel>(model_);

      sion.Predict();

      // 如果没有识别到，上面给出的armors_3d_i_是空的，模型会自动处理
      sion.Update(armors_3d_i_);

      auto aim_point =
          sion.PredictAndChoose(imu_data_i_, aim_traj_o.all_armors);
      aim_traj_o.traj_points.push_back(aim_point);
    }
    catch (std::bad_variant_access&)
    {
      NV_ASSERT(0 && "Model type is not same as aim model type!");
    }
  }
SEND:
  aim_traj_o.cap_stamp = armors_3d_i_.cap_stamp;
  aim_traj_c_sPtr_->Transmit(aim_traj_o);
}

} // namespace ne_vision
