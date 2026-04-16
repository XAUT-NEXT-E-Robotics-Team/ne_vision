# ne_vision 代码规范

## 项目概述

C++20 机器人视觉项目，用于自瞄系统。核心模块：detector、tracker、planner、serial、utils。

## 命名规范

### 类型后缀系统

所有自定义类型必须带后缀，明确区分类型种类：

| 后缀 | 用途 | 示例 |
|------|------|------|
| `_e` | `enum class` | `NeAutoAimState_e`, `NeChannelType_e` |
| `_t` | `struct` / `typedef` | `NeAutoAimResult_t`, `NeArmors3D_t` |
| `_sPtr_` | `shared_ptr` 成员变量 | `detector_sPtr_`, `channel_sPtr_` |
| `_uPtr_` | `unique_ptr` 成员变量 | `detector_uPtr_` |
| `CSPtr_t` | `shared_ptr` 的类型别名 | `NeArmors3DCSPtr_t` |

### 类与结构体

- 类名：`PascalCase`，带 `Ne` 前缀：`NeAutoAim`, `NeTracker3D`, `NeChannel`
- 嵌套结构体：`PascalCase` + `_t`：`Armor3D_t`, `ModelStatus_t`, `Debug_t`
- 模板类型别名（`using`）：`PascalCase` + `_t`：`ErrorState_t`, `HJac_t`

### 成员变量

- 私有/保护成员：`snake_case` + 尾部 `_`：`name_`, `is_running_`, `channel_size_`
- 基类内部成员（双下划线，表示"更私有"）：`name__`, `mtx__`
- 公开结构体字段：`snake_case` 无下划线：`cap_stamp`, `aim_id`, `yaw`

### 函数与方法

- 公开方法：`PascalCase`：`Transmit()`, `Receive()`, `WaitForData()`, `GetName()`
- 私有方法：`camelCase`：`setupTasks()`, `predictState()`, `computeQ()`, `matchID()`
- 内联 getter：`PascalCase`，`inline` 修饰：`inline bool IsRunning() const`

### 常量

- `constexpr` 常量：`UPPER_SNAKE_CASE`：`STATE_DIM`, `P_X_IDX`, `YAW_IDX`
- 宏：`UPPER_SNAKE_CASE` + `NV_` 前缀：`NV_ASSERT`, `NV_ERROR`, `NV_PARAM`, `NV_WEAK_CPP`

### 文件命名

- 所有源文件：`snake_case`，带 `ne_` 前缀：`ne_auto_aim.hpp`, `ne_channel.hpp`
- 测试文件：`ut_`（单元测试）或 `mt_`（模块测试）前缀：`ut_channel_and_task.cpp`

### 命名空间

- 主命名空间：`ne_vision`
- 子命名空间：`snake_case`：`ne_vision::interfaces`, `ne_vision::sion`

## 代码风格

### 格式

- 缩进：2 空格
- 大括号：Allman 风格（`{` 另起一行）
- 指针/引用：靠近类型名：`const std::string& name`，`std::unique_ptr<NeTask>`
- 多参数对齐：垂直对齐参数列表

```cpp
void UpdateImu(const Eigen::Vector3d&    acc,
               const Eigen::Vector3d&    gyro,
               const Eigen::Quaterniond& quat);
```

### 头文件

- 使用 `#pragma once`
- include 顺序：标准库 → 第三方库 → 项目内部
- 项目内部 include 用引号：`"ne_vision/utils/ne_channel.hpp"`
- 第三方用尖括号：`"Eigen/Dense"`（Eigen 例外，用引号）

### 注释

- 文件头：统一的 ASCII art banner + 版权信息 + `// Description:` 说明
- 公开 API：Doxygen 风格（`/** */` + `@brief`, `@param`, `@return`, `@note`）
- 私有实现：`//` 行内注释，中文或英文均可
- 分区注释：`/* === 区域名 === */`

### 智能指针命名约定

成员变量中的智能指针必须在名称中体现指针类型：

```cpp
std::unique_ptr<NeTask>    detector_uPtr_;   // unique_ptr
std::shared_ptr<NeDetector> detector_sPtr_;  // shared_ptr
```

### 参数结构体

将参数集中到内部 `struct Params_t` 或匿名 `struct`，并提供 `LoadParam()` 方法从配置加载：

```cpp
struct
{
  double idle_time = 0.5;
} param_;
```

### 任务与线程

- 使用 `NeTask` 封装线程，不直接使用 `std::thread`
- 使用 `NeChannel<T>` 进行线程间通信，不使用裸共享变量
- 互斥锁成员命名：`mtx_`（单下划线）或 `mtx__`（双下划线，基类）

## 核心算法原理

### 整体数据流

```
相机帧 → NeDetector(2D检测) → NeTracker2D(2D跟踪/选板)
                                    ↓ NeArmors2D_t
                              NeTracker3D(3D状态估计)
                                    ↓ NeAimState_t (含 AimPredictor)
                              NeMashiroPlanner(MPC轨迹规划)
                                    ↓ NeGimbalControlRef_t
                              串口输出 → 电控
```

IMU 数据通过独立通道持续注入，各模块通过 `NeChannel<T>` 异步通信。

---

### ESIKF — 误差状态迭代卡尔曼滤波（NeSionModel）

**用途**：对旋转装甲板（小陀螺）进行整车级 3D 状态估计。

**状态向量**（10维）：
```
x = [px, py,  vx, vy,  yaw, omega,  z1, z2,  R1, R2]
     位置(2) 速度(2) 整车yaw+角速度  两组装甲板高度  两组半径
```

**核心思路**：
- 名义状态 `x` + 误差状态 `ex`（小量），yaw 用 SO2 广义加减法（`WrapToPi`）
- 预测：匀速+IMU加速度修正，计算过程噪声 Q（自适应 `var_a`, `var_beta`）
- 更新：对每块可见装甲板做 ID 匹配（对数似然代价），迭代线性化观测方程 H，ESIKF 迭代收敛（最多5次，阈值 `epsilon=1e-4`）
- 发散检测：NIS 卡方检验（自由度4，阈值13.28），连续发散超 `max_divergence_count` 次则标记发散

**多模型假设**：初始化时同时维护3个模型（正转/反转/其他），`init_max_count_value` 帧后按累积代价选出唯一模型，`current_model_idx_=-1` 表示仍在评估中。

**观测方程 h**：给定装甲板 ID（周期编号 `NePeriodicNumber<4>`）和状态，计算预测的 `[x, y, z, yaw]` 测量值。

---

### 弹道补偿（YUKINO::BallisticModel）

**模型**：考虑空气阻力的弹道模型，阻力系数 `K1` 按弹丸类型（small/large）区分。

**水平飞行距离公式**（含阻力）：
```
x_flight = ln(1 + t · K1 · v0 · cos(pitch)) / K1
```

**迭代求解 pitch**（`Cal_TargetposPitch`）：
1. 初始 `pitch=0`，用 `atan2(height, distance)` 估算
2. 代入弹道模型算出实际落点高度
3. 用误差补偿更新目标高度，循环至误差 < `PRECISION`

**飞行时间**：`ft = (exp(x·K1) - 1) / (K1 · cos(pitch) · v0)`

---

### MPC 轨迹规划（NeMashiroPlanner，使用 TinyMPC）

**用途**：在已知目标预测轨迹的情况下，规划云台最优控制序列，平滑处理小陀螺跳变。

**状态**（4维）：`[yaw, yaw_v, pitch, pitch_v]`
**控制**（2维）：`[yaw_acc, pitch_acc]`

**离散化动力学**（匀加速模型，步长 `step`）：
```
A = [[1, dt, 0,  0 ],    B = [[0.5·dt², 0      ],
     [0,  1, 0,  0 ],         [dt,       0      ],
     [0,  0, 1, dt ],         [0,        0.5·dt²],
     [0,  0, 0,  1 ]]         [0,        dt     ]]
```

**每次规划流程**：
1. 读取当前 IMU，预测 `additional_predict_time` 后的云台初始状态作为 `x0`
2. 对预测窗口内每一步（共 `horizon` 步），调用 `NeSionAimPredictor::Predict` + 弹道补偿迭代，得到参考 `[yaw, pitch, yaw_v, pitch_v]`
3. 设置箱式约束（速度/加速度上下界），调用 `tiny_solve` 求解 ADMM
4. 取轨迹第1步（`x[:,1]`）作为当前控制输出

**时间补偿**：控制时刻 = 最新IMU时刻 + `base_dt` + 1步长，`base_dt` = IMU到当前延迟 + 额外预测时间。

---

### AimPredictor 接口（`NeAimPredictorBase`）

连接 Tracker3D 与 Planner 的桥梁，封装"固化的状态副本"，保证线程安全：

- `Init()`：将状态从 `cap_stamp` 积分推进到最新 IMU 时刻
- `Predict(dt, imu_data, ...)`：在预测窗口内，给定额外时间 `dt` 和 IMU 数据，输出目标位置、yaw、速度

`NeSionAimPredictor` 是其针对 Sion 模型的具体实现。

---

## 架构约定

- 接口数据结构放在 `interfaces` 命名空间，文件在 `include/ne_vision/interfaces/`
- 模型实现放在独立命名空间（如 `sion`），文件在 `include/ne_vision/models/`
- 工具类放在 `include/ne_vision/utils/`，文件名带 `ne_` 前缀
