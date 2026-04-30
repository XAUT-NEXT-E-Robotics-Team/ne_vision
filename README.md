# 西安理工大学NEXT-E战队新自喵~

注意：自瞄核心部分与ROS2无关，原码可以通过修改cmake设置编译为GDExtension形式，ros2形式

## 用

以下内容在工程根目录下执行：

```
python3 nv.py build # 编译
source install/setup.sh # 设置环境变量
python3 nv.py run -h # 查看可以运行什么
python3 nv.py run ut # 运行所有单测
python3 nv.py run mt_auto_aim_video_test # 运行视频测试
# 仿真测试先不写了
```

视频测试中，按p可以暂停，按esc退出，按除以上外的任何键转到下一帧。（注意：该操作会影响KF）

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
