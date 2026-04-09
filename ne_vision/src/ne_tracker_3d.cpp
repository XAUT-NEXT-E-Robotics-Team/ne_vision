#include "ne_vision/tracker/ne_tracker_3d.hpp"
#include "ne_vision/ballistic_compensation/ballistic_slove.hpp"
#include "ne_vision/models/ne_sion_model.hpp"
#include "ne_vision/utils/ne_code_profiler.hpp"
#include "ne_vision/utils/ne_debug.hpp"
#include "ne_vision/utils/ne_log.hpp"
#include "ne_vision/utils/ne_rerun_debug.hpp"
#include "ne_vision/ne_channals.hpp"
#include "rerun/archetypes/scalars.hpp"
#include <chrono>
#include <string>

#define IS_SION_MODEL(aim_str)                                                 \
  (aim_str == "1" || aim_str == "2" || aim_str == "3" || aim_str == "4" ||     \
   aim_str == "7")

// 其他模型

namespace ne_vision
{

NeTracker3D::NeTracker3D(const std::string& name)
    : name_(name), armors_3d_c_sPtr_(NV_CHANNELS.armor3d_sPtr()),
      imu_data_c_sPtr_(NV_CHANNELS.imu_data_sPtr()),
      aim_traj_c_sPtr_(NV_CHANNELS.aim_traj_sPtr())
{
  NV_ASSERT(armors_3d_c_sPtr_ != nullptr && imu_data_c_sPtr_ != nullptr &&
            aim_traj_c_sPtr_ != nullptr &&
            "armors_3d_c_sPtr_, imu_data_c_sPtr_, and aim_traj_c_sPtr_ cannot "
            "be nullptr");

  last_cap_stamp_ = std::chrono::steady_clock::now();
}

void NeTracker3D::Track()
{
  NeAimTraj_t aim_traj_o;

  NeArmors3D_t armors_3d_i_;
  NeImuData_t  imu_data_i_;

  if (!armors_3d_c_sPtr_->Receive(armors_3d_i_))
  {
    NV_WARN("No 3D armor data received, skipping tracking");
    return;
  }

  // 更新当前IMU数据
  imu_data_i_ = armors_3d_i_.imu_data;

  // 0. 计算dt
  auto this_cap_stamp = armors_3d_i_.cap_stamp;

  const double dt =
      std::chrono::duration<double>(this_cap_stamp - last_cap_stamp_).count();
  last_cap_stamp_ = this_cap_stamp;

  // 1. 空闲判断：
  //    如果超过一定时间没有观测到装甲板了，把跟踪器删了，防止下次初始化异常
  auto now = std::chrono::steady_clock::now();
  if (now - last_detected_stamp_ >
      std::chrono::duration<double>(param_.idle_time))
  {
    current_tracking_aim_ = "NULL";
    model_ = std::monostate{};
  }

  // 如果观测到了，则更新最后观测到装甲板的时间点
  if (!armors_3d_i_.armors.empty())
    last_detected_stamp_ = now; // 更新最后观测到装甲板的时间点

  // 2.如果还没有空闲，在没识别到，或者识别到的还是当前目标，就继续跟踪
  //   下面逻辑会自动切换为仅预测
  if (armors_3d_i_.armors.empty() ||
      armors_3d_i_.aim_id == current_tracking_aim_)
  {
    goto DO_NORMAL_TRACKING;
  }

  // 3. 如果目标变化了，这里创建新的跟踪器，即使没有跟丢（尊重2D选板）
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
    goto DO_NORMAL_TRACKING;
  }

  // 4. 进行正常一次跟踪
DO_NORMAL_TRACKING:

  // 计算dt
  static auto last_time = std::chrono::steady_clock::now();

  // 分门别类观测
  if (IS_SION_MODEL(current_tracking_aim_))
  {
    try
    {
      auto& sion = std::get<sion::NeSionModel>(model_);

      sion.Predict(imu_data_i_, dt);

      // 如果没有识别到，上面给出的armors_3d_i_是空的，模型会自动处理
      sion.Update(armors_3d_i_);

      // NeRerunDebug::GetInstance().EnableRealtimeDebug();

      sion.DebugInfo(); // 输出调试信息

      auto state = sion.GetState();
      for (int id = 0; id < 4; ++id)
      {
        auto z = sion.Measure(id, state);
        aim_traj_o.debug.all_armors.emplace_back(z(sion::MEASURE_X_IDX),
                                                 z(sion::MEASURE_Y_IDX),
                                                 z(sion::MEASURE_Z_IDX));
        aim_traj_o.debug.model_dis =
            std::sqrt(std::pow(state.p.norm(), 2) +
                      std::pow((state.z1 + state.z2) / 2, 2));
        aim_traj_o.debug.model_yaw = state.yaw;
        aim_traj_o.debug.model_omega = state.omega;
      }

      // 获取并提取这期间的IMU数据
      auto imu_history = imu_data_c_sPtr_->GetDataSince(
          armors_3d_i_.cap_stamp,
          [](const interfaces::NeImuData_t& imu) { return imu.receive_stamp; });

      auto newest_imu = imu_history.empty() ? imu_data_i_ : imu_history.back();

      // 将cap_stamp处的状态后推到imu_received_stamp（还不是当前时间）
      aim_traj_o.aim_predictor = sion.GetAimPredictor(
          armors_3d_i_.cap_stamp, newest_imu.receive_stamp, imu_history);

      // 记录预测器生成时的最新IMU
      aim_traj_o.newest_imu = newest_imu;

      // 目标存在
      aim_traj_o.has_target = true;
    }
    catch (std::bad_variant_access&)
    {
      NV_ASSERT(0 && "Model type is not same as aim model type!");
    }
  }
  else
  {
    aim_traj_o.has_target = false;
  }
SEND:
  aim_traj_o.cap_stamp = armors_3d_i_.cap_stamp;
  aim_traj_c_sPtr_->Transmit(aim_traj_o);
}

} // namespace ne_vision
