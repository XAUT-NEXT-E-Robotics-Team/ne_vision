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
// ABAB 你不可能通过名字知道这是什么模型 ABAB
//
// 感谢 Sion 和 Erica 保佑模型正常
//
// 反小陀螺整车模型，文档见DOCS
//
// 注意：请不要直接为该模型创建任务，其实你也创建不了。
//      请将其放在tracker 3d运行，哪里可能还有更多模型再运行。
//
// Tip: 由SO2特殊的交换性质，因此广义加法等同于相加并重映射到PI
//
// 一些名称和符号说明
// state 或 x： 名义状态
// error_state 或 ex： 误差状态

#pragma once

#include <array>
#include <chrono>
#include <vector>

#include "Eigen/Dense"
#include "ne_vision/interfaces/ne_armors_3d.hpp"
#include "ne_vision/interfaces/ne_imu_data.hpp"
#include "ne_vision/utils/ne_math.hpp"

namespace ne_vision
{

namespace sion
{

// 定义一些常量
// constexpr 好处是含有命名空间
constexpr size_t STATE_DIM =
    10; // 状态维度：位置(3) + 速度(3) + yaw(1) + omega(1) + z1(1) + z2(1)
constexpr size_t P_X_IDX = 0;   // 位置x
constexpr size_t P_Y_IDX = 1;   // 位置y
constexpr size_t DP_X_IDX = 2;  // 速度x
constexpr size_t DP_Y_IDX = 3;  // 速度y
constexpr size_t YAW_IDX = 4;   // yaw角
constexpr size_t OMEGA_IDX = 5; // yaw角速度
constexpr size_t Z1_IDX = 6;    // 装甲板1高度
constexpr size_t Z2_IDX = 7;    // 装甲板2高度
constexpr size_t R1_IDX = 8;    // 装甲板1半径
constexpr size_t R2_IDX = 9;    // 装甲板2半径

constexpr size_t MEASURE_X_IDX = 0;   // 测量位置x
constexpr size_t MEASURE_Y_IDX = 1;   // 测量位置y
constexpr size_t MEASURE_Z_IDX = 2;   // 测量位置z
constexpr size_t MEASURE_YAW_IDX = 3; // 测量装甲板yaw

// 名义状态结构体
struct NeSionState_t
{
  using Vector10d_t = Eigen::Matrix<double, STATE_DIM, 1>;

  Eigen::Vector2d p;      // 位置
  Eigen::Vector2d v;      // 速度
  double          yaw;    // 整车yaw角
  double          omega;  // 整车yaw角速度
  double          z1, z2; // 两组装甲板高度
  double          R1, R2; // 两组装甲板半径

  // 构造函数和初始化
  NeSionState_t()
      : p(Eigen::Vector2d::Zero()), v(Eigen::Vector2d::Zero()), yaw(0),
        omega(0), z1(0), z2(0), R1(0.3), R2(0.3)
  {
    // 注意：初始半径不要给0
  }

  // 广义加法重载
  // 输入为误差状态或等同的小量
  NeSionState_t operator+(const Vector10d_t ex) const
  {
    // 只有YAW的操作有所区别，其他都是普通的加法
    NeSionState_t res;
    res.p = p + ex.segment<2>(P_X_IDX);
    res.v = v + ex.segment<2>(DP_X_IDX);
    res.yaw = math::WrapToPi(yaw + ex(YAW_IDX));
    res.omega = omega + ex(OMEGA_IDX);
    res.z1 = z1 + ex(Z1_IDX);
    res.z2 = z2 + ex(Z2_IDX);
    res.R1 = R1 + ex(R1_IDX);
    res.R2 = R2 + ex(R2_IDX);
    return res;
  }

  NeSionState_t& operator+=(const Vector10d_t ex)
  {
    *this = *this + ex;
    return *this;
  }

  // 广义减法重载
  // 输出为误差状态或等同的小量
  Vector10d_t operator-(const NeSionState_t& other) const
  {
    // 只有YAW的操作有所区别，其他都是普通的减法
    Vector10d_t ex;
    ex.segment<2>(P_X_IDX) = p - other.p;
    ex.segment<2>(DP_X_IDX) = v - other.v;
    ex(YAW_IDX) = math::WrapToPi(yaw - other.yaw);
    ex(OMEGA_IDX) = omega - other.omega;
    ex(Z1_IDX) = z1 - other.z1;
    ex(Z2_IDX) = z2 - other.z2;
    ex(R1_IDX) = R1 - other.R1;
    ex(R2_IDX) = R2 - other.R2;
    return ex;
  }
};

class NeSionModel final
{

private:
  using ErrorState_t = Eigen::Matrix<double, 10, 1>;
  using ErrorStateMat_t = Eigen::Matrix<double, 10, 10>;
  using MeasurementNoiseCov_t = Eigen::Matrix<double, 4, 4>;
  using Measurement_t = Eigen::Vector4d;
  using AccDate_t = Eigen::Vector2d;
  using NePeriodicNumber_t = NePeriodicNumber<4>;
  using HJac_t = Eigen::Matrix<double, 4, 10>;
  using K_t = Eigen::Matrix<double, 10, 4>;

public:
  explicit NeSionModel(const interfaces::NeArmors3D_t& init_armors);
  ~NeSionModel() = default;

  // 考虑加速度的预测
  void Predict(const interfaces::NeImuData_t& imu_data, const double dt);

  // 不考虑加速度的预测
  void Predict(const double dt);

  // 更新状态，输入3D装甲数据
  void Update(const interfaces::NeArmors3D_t& armors);

  // 调试信息打印函数
  void DebugInfo();

  Eigen::Vector3d
  PredictAndChoose(const interfaces::NeImuData_t& imu_data,
                   double                         b_ds,
                   std::vector<Eigen::Vector3d>&  all_pred_armors);

  NeSionState_t GetState() const
  {
    return models_.at(current_model_idx_ >= 0 ? current_model_idx_ : 0)
        .esikf_data.x;
  }

  Measurement_t Measure(const NePeriodicNumber_t id, const NeSionState_t& x)
  {
    Measurement_t z;
    h(id, x, z);
    return z;
  }

  int GetModelIdx() const { return current_model_idx_; }

  double GetModelCurrentVarA() const
  {
    return models_.at(current_model_idx_ >= 0 ? current_model_idx_ : 0)
        .current_var_a;
  }

  double GetModelCurrentVarBeta() const
  {
    return models_.at(current_model_idx_ >= 0 ? current_model_idx_ : 0)
        .current_var_beta;
  }

  static std::string GetName() { return "SionModel"; }

private:
  struct ModelStatus_t
  {
    struct
    {
      NeSionState_t   x;      // 当前名义状态
      NeSionState_t   x_pred; // 预测状态
      ErrorStateMat_t eP;     // 误差状态预测协方差
    } esikf_data;

    double cumulative_cost = 0; // 累积代价，用于多模型评估
    size_t update_count = 0;    // 更新计数器，用于判断何时进行多模型评估

    double current_var_a = 1;    // 当前自适应加速度噪声方差
    double current_var_beta = 1; // 当前自适应角加速度噪声方差
    double var_a_base = 0;
    double var_beta_base = 0;

    int  divergence_count = 0; // 连续发散计数器
    bool is_diverged = false;  // 是否发散的标志

    double last_nis = 0;
    double last_dis = 0;
  };
  using Models_t = std::array<ModelStatus_t, 3>;

  /* === 工具函数区，这里的函数只依赖参数和全局状态 === */

  // 这里的私有函数除模型状态信息ModelState_e之外，不应该访问任何成员变量，输入输出都通过参数传递
  // 以便编写观测器逻辑时，可以把每个函数当作独立的工具函数从而减少耦合

  // 预测状态
  void predictState(const double        dt,
                    const NeSionState_t x,
                    const AccDate_t&    a,
                    NeSionState_t&      x_pred);
  // 计算过程噪声协方差矩阵Q
  void computeQ(double dt, double var_a, double var_beta, ErrorStateMat_t& Q);
  // 计算测量噪声协方差矩阵R
  void computeR(MeasurementNoiseCov_t& R);
  // 计算预测雅克比（误差状态）
  void computeF(double dt, ErrorStateMat_t& F);
  // 观测方程h
  void h(const NePeriodicNumber_t id, const NeSionState_t& x, Measurement_t& z);

  // 计算H 这里的x有两种可能的取值
  // 1. 名义变量预测结果x_hat
  // 2. 迭代更新过程中不断更新的x
  // 其实这俩也没必要分这么清吧
  void computeH(const NePeriodicNumber_t id, const NeSionState_t& x, HJac_t& H);

  // ID匹配
  // cost 是对数似然代价，这里懒得写那么长了
  bool matchID(const interfaces::NeArmors3D_t::Armor3D_t& armor,
               const Measurement_t&                       z,
               const NeSionState_t&                       x_pred,
               const ErrorStateMat_t&                     eP,
               NePeriodicNumber_t&                        matched_id,
               double&                                    cost,
               double&                                    nis);

  /* === 统合函数区，这里的函数依赖上面的工具函数 === */

  void initializeModel(const interfaces::NeArmors3D_t& init_armors,
                       Models_t&                       models);

  // 单词预测更新函数，给定加速度数据、预测时间和模型状态，输出预测状态并更新预测协方差
  void predictAndUpdateOnce(const interfaces::NeImuData_t& imu_data,
                            const double                   dt,
                            const AccDate_t&               a,
                            ModelStatus_t&                 model);

  // 单次更新函数，给定一组装甲板数据，更新一次状态
  // model_is_only_one表示当前是否只有一个模型在跑，表示是否是多模型评估模式
  void updateOnce(const interfaces::NeArmors3D_t& armors,
                  ModelStatus_t&                  model,
                  bool                            model_is_only_one);

  // 发散判断 TODO: 发散判断
  void adaptiveQAndDivergenceCheck(ModelStatus_t& model, double nis);

  // 多模型评估和选择函数，输入所有模型状态，修改current_model_idx为当前选用的模型索引
  // current_model_idx为负数表示当前仍需同时维护多个模型，无法决策出唯一的模型
  // 在update里调用她
  void checkMultipleModel(const Models_t& models);

  struct Params_t
  {
    void LoadParam()
    {
      auto param = NV_PARAM["auto_aim"]["tracker_3d"]["sion_model"];

      // 初始参数
      init_R = param["init_R"].as<double>();
      init_omega = param["init_omega"].as<double>();
      init_max_count_value = param["init_max_count_value"].as<double>();
      init_alpha = param["init_alpha"].as<double>();

      // ESIKF参数
      var_a_min = param["var_a_min"].as<double>();
      var_a_max = param["var_a_max"].as<double>();
      var_beta_min = param["var_beta_min"].as<double>();
      var_beta_max = param["var_beta_max"].as<double>();
      q_scale_rate = param["q_scale_rate"].as<double>();
      var_z = param["var_z"].as<double>();
      var_R = param["var_R"].as<double>();
      omega_dead_band = param["omega_dead_band"].as<double>();

      // 发散检测参数
      divergence_threshold = param["divergence_threshold"].as<double>();
      max_divergence_count = param["max_divergence_count"].as<int>();
      max_R = param["max_R"].as<double>();
      min_R = param["min_R"].as<double>();
      max_omega = param["max_omega"].as<double>();

      // 调试参数
      debug.enable = param["debug"]["enable"].as<bool>();
      debug.yaw = param["debug"]["yaw"].as<bool>();
      debug.omega = param["debug"]["omega"].as<bool>();
      debug.R = param["debug"]["R"].as<bool>();
      debug.Q_var = param["debug"]["Q_var"].as<bool>();
      debug.nis = param["debug"]["nis"].as<bool>();
      debug.dis = param["debug"]["dis"].as<bool>();
      debug.model_idx = param["debug"]["model_idx"].as<bool>();

      // 参数合理性检查
      if (init_R <= 0)
      {
        NV_ERROR("Initial radius should be positive! Current value: {}",
                 init_R);
        abort();
      }
      if (init_max_count_value <= 0)
      {
        NV_ERROR(
            "Initial max count value should be positive! Current value: {}",
            init_max_count_value);
        abort();
      }
      if (init_alpha < 0 || init_alpha > 1)
      {
        NV_ERROR("Initial alpha should be in [0, 1]! Current value: {}",
                 init_alpha);
        abort();
      }
      if (var_a_min <= 0 || var_a_max <= 0 || var_a_min >= var_a_max)
      {
        NV_ERROR("var_a should be positive and var_a_min should be less than "
                 "var_a_max! Current values: var_a_min={}, var_a_max={}",
                 var_a_min,
                 var_a_max);
        abort();
      }
    }

    double init_R = 0.3;              // 初始半径，单位米
    double init_omega = 5.0;          // 初始角速度，单位rad/s，正反转初始化
    double init_max_count_value = 15; // 多少帧后进行评估和选择
    double init_alpha = 0.4;          // 累积代价的低通滤波系数，越大越重视历史

    /* === ESIKF === */

    // 自适应加速度噪声阈值参数
    // 需要在高速变速运动时调整min的值
    // 需要在低速，匀速或静止时调整max的值
    // 如果在高速变速或静止时计算得到的实时var并未到达min或max的值
    // 可以通过调整scale_rate来调整自适应var的变化率，增大它可以让自适应调大或调小更灵活
    // 默认为中值
    double var_a_min = 1;
    double var_a_max = 100;

    // 同理
    // 默认为中值
    double var_beta_min = 1;
    double var_beta_max = 10;

    double q_scale_rate = 0.1;

    double var_z = 0;
    double var_R = 0;

    size_t max_iter = 5;   // ESIKF 最大更新迭代次数
    double epsilon = 1e-4; // ESIKF 更新迭代收敛阈值

    // 低角速度死区
    // 由于在角速度极低时，半径不可观测，可以设置一个死区，在角速度过低时不更新半径
    double omega_dead_band = 0.1; // 一定是一个正值

    /* === 发散检测参数 === */

    // 卡方发散阈值
    // 自由度为4，显著性水平为0.99的卡方分布临界值
    double divergence_threshold = 13.28;
    // 连续多少帧发散就认为模型发散了
    int max_divergence_count = 5;
    // 最大可接受半径，单位米
    double max_R = 0.6;
    // 最小可接受半径，单位米
    double min_R = 0.1;
    // 最大可接受角速度，包括反向，单位rad/s
    double max_omega = 25.0;

    struct
    {
      bool enable;
      bool yaw;
      bool omega;
      bool R;
      bool Q_var;
      bool nis;
      bool dis;
      bool model_idx;
    } debug;

  } params_;

  // 在正常情况下只有一个模型在运行，初始化时会同时维护多个模型来进行多假设跟踪
  int      current_model_idx_ = -1;
  Models_t models_;
};

} // namespace sion

} // namespace ne_vision
