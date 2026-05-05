# 西安理工大学NEXT-E战队新自喵~

注意：自瞄核心部分与ROS2无关，原码可以通过修改cmake设置编译为GDExtension形式，ros2形式

## 构建

### 依赖

- CMake >= 3.22.1
- Ninja
- C++20 编译器

### 预设

项目使用 CMake Presets，两个主要预设：

| 预设 | 用途 | 构建类型 |
|------|------|----------|
| `godot_sim` | Godot 仿真（开启 GD 适配器） | Debug |
| `reality` | 真实硬件（开启串口 + 海康驱动 + Reality 适配器） | Release |

### 仿真构建

```bash
cmake --preset godot_sim
cmake --build --preset godot_sim
cmake --install build
```

### 硬件构建（Reality）

```bash
cmake --preset reality
cmake --build --preset reality
cmake --install build
```

等价于直接运行：

```bash
./build_reality.sh
```

### 可选模块

通过 CMake 选项手动开关（预设已配置好，一般无需手动设置）：

| 选项 | 说明 | 默认 |
|------|------|------|
| `DRIVER_ENABLE_NE_SERIAL_DRIVER` | 串口驱动 | OFF |
| `DRIVER_ENABLE_NE_HIK_DRIVER` | 海康相机驱动（需 MVS SDK） | OFF |
| `ADAPTER_ENABLE_NE_VISION_GD` | Godot Extension 适配器 | OFF |
| `ADAPTER_ENABLE_NE_VISION_REALITY` | Reality 硬件适配器 | OFF |
| `EXTENSION_ENABLE_NE_DEBUG_ZMQ` | ZeroMQ 调试扩展 | OFF |

### 运行测试

```bash
source install/setup.sh
python3 nv.py run -h                      # 查看可运行目标
python3 nv.py run ut                      # 所有单测
python3 nv.py run mt_auto_aim_video_test  # 视频测试
```

视频测试：`p` 暂停，`Esc` 退出，其他键下一帧（注意：逐帧操作会影响 KF 状态）。

## 文档索引

- [仿同济 MPC 轨迹规划器 (mashiro_planner)](docs/mashiro_planner.md)
- [Sion 卡尔曼滤波追踪模型 (sion_model)](docs/sion_model.md)
- [PnP 优化与协方差矩阵 (pnp_optimize_and_cov)](docs/pnp_optimize_and_cov.md)
- [开发者备忘录 / Memo (memo)](docs/memo.md)

## 项目结构

- ne_vision 视觉主库
- ne_vision_sim 视觉godot仿真
- ne_vision_web 用网页来可视化或调参

---

## 已知逻辑问题（代码审查 2026-04-26）

### Critical

#### 1. 瞄准背面装甲板 — `ne_sion_model.cpp:128`

```cpp
// 错误：+ M_PI 导致选背面装甲
double yaw_diff = std::abs(math::WrapToPi(armor_yaw - gimbal_yaw + M_PI));
// 正确：
double yaw_diff = std::abs(math::WrapToPi(armor_yaw - gimbal_yaw));
```

预测阶段始终选背面装甲板，导致瞄准方向完全相反。

#### 2. MPC 输出被丢弃 — `ne_mashiro_planner.cpp:306`

```cpp
// 正确（被注释）：
// gimbal_control_ref_o.yaw_ref = work_ptr->x(0, 1);
// 实际使用原始预测值，TinyMPC 求解结果被忽略：
gimbal_control_ref_o.yaw_ref = gimbal_control_ref_o.debug.aim_yaw;
```

轨迹规划完全失效，云台收到无平滑的阶跃参考值。

#### 6. 弹道全局状态数据竞争 — `ballistic_slove.cpp:4`

```cpp
Com_ps  g_com_ps;
Com_ps* Com_ptr_ = &g_com_ps;  // 全局可变状态，多线程下 UB
```

`ft`、`muzzle_v` 等字段被并发读写，函数不可重入。

---

### Medium

| 位置                       | 问题                                                |
| -------------------------- | --------------------------------------------------- |
| `ne_sion_model.cpp:386`    | 协方差更新用 `(I-KH)P` 而非 Joseph form，数值不稳定 |
| `ne_sion_model.cpp:410`    | 双装甲选择用欧氏距离而非马氏距离，近距离易关联错误  |
| `ne_sion_model.cpp:467`    | NIS 发散检测被注释掉，滤波器静默发散无法检测        |
| `ne_serial_driver.cpp:100` | `auto_aim_status_` 无锁写，数据竞争                 |
| `ne_channel.hpp:397`       | `Size()`/`Empty()` 无锁，非线程安全                 |
