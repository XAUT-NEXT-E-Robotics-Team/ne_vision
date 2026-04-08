# mashiro - 仿同济大学MPC规划器

![](imgs/mashiro.webp)

## 0. 啊巴啊吧

由于公式比较简单，尝试了一波与AI讨论，AI自动生成文档的工作流。下面全AI编写，笔者并无这个水平

## 1. 介绍

mashiro 规划器是针对带有底盘高速自旋（“小陀螺”）等高动态工况设计的两自由度云台（Yaw-Pitch）MPC 轨迹规划器。
与其在预测模型中引入复杂的非线性多体动力学（如科氏力、变惯量等），mashiro 采用了**级联控制与特征边界建模**理念：

- **外环（MPC规划层）**：在世界坐标系（绝对坐标系）下构建纯运动学模型，输出平滑的位置、速度与加速度指令。
- **内环（底层PID或力矩执行层）**：负责闭环跟随，克服转动惯量和摩擦带来的牛顿力学效应。

通过将云台的动力学极限（如最大电机力矩、由于负载变化带来的变惯量等）映射为随底盘运动状态动态平移的**运动学边界约束**，本规划器能在保证较低算力开销（支持 100Hz 以上高频）的前提下，实现云台在极端工况下避免底层饱和、防止失控跟丢的最优轨迹规划。

## 2. 方法

### 2.1 二自由度运动学

由于将控制力矩的执行交给了底层 PID，MPC 使用的预测模型简化为双积分器（Double Integrator）运动学模型。为了解耦底盘旋转的影响，我们直接在世界坐标系下建立状态方程。

定义状态变量 $\mathbf{x}$ 为世界坐标系下的绝对角度和绝对角速度，控制输入 $\mathbf{u}$ 为期望绝对角加速度：

$$
\mathbf{x} = \begin{bmatrix} \theta \\ \dot{\theta} \end{bmatrix}, \quad \mathbf{u} = \begin{bmatrix} \ddot{\theta} \end{bmatrix}
$$

设定离散化采样时间为 $T_s$，系统在世界坐标系下的离散化状态空间方程如下：

$$
\mathbf{x}_{k+1} = \mathbf{A}_d \mathbf{x}_k + \mathbf{B}_d \mathbf{u}_k
$$

其中状态转移矩阵 $\mathbf{A}_d$ 与输入矩阵 $\mathbf{B}_d$ 分别为：

$$
\mathbf{A}_d = \begin{bmatrix} 1 & T_s \\ 0 & 1 \end{bmatrix}, \quad \mathbf{B}_d = \begin{bmatrix} \frac{1}{2}T_s^2 \\ T_s \end{bmatrix}
$$

_注：如果底层速度闭环响应存在明显的滞后，可考虑采用带时间常数 $\tau$ 的等效动力学模型。但为了计算效率，最优雅的方式仍是使用纯运动学矩阵，并将动力学特性通过边界约束来界定。_

### 2.2 位置、速度、加速度约束计算

在诸如“小陀螺”模式下，底盘会以极高的角速度 $\omega_c$ 自旋。云台电机的物理能力（最大相对转速和相对扭矩）是固定的。因此，要在世界坐标系下实现有效规划，MPC 的约束必须是**动态时变（Time-Varying）** 的。

假设云台电机受物理制造约束的最值设为：最大相对角速度 $[-\omega_{max}, \omega_{max}]$，最大相对角加速度 $[-a_{max}, a_{max}]$（由力矩饱和 $\tau_{max}$ 和惯量 $J$ 决定：$a_{max} = \frac{\tau_{max}}{J}$）。

#### 1. 速度动态约束 (Velocity Constraints)

在预测时域，电机相对底盘的实际角转速必须不超过限幅，转化为世界坐标系内的绝对角速度 $\dot{\theta}_{k}$：

$$
-\omega_{max} \le \dot{\theta}_{k} - \omega_c \le \omega_{max}
$$

化简得到预测范围内各时刻的绝对硬速度约束区间：

$$
-\omega_{max} + \omega_c \le \dot{\theta}_{k} \le \omega_{max} + \omega_c
$$

_**物理意义**：当底盘逆时针高速旋转时，电机的绝对角速度能力区间也被一同平移。这让求解器清晰地“感知”到反向追踪会面临极易饱和的风险，从而提前规划合理路线。_

#### 2. 加速度动态约束 (Acceleration Constraints)

同理，电机最大输出扭矩转化为最大相对角加速度界限。考虑到当前底盘自身的角加速度 $\ddot{\theta}_c$，MPC 能下发的指令增量区间将被压缩和偏移：

$$
-a_{max} + \ddot{\theta}_c \le \ddot{\theta}_{k} \le a_{max} + \ddot{\theta}_c
$$

_这种将非线性耦合与巨大惯量 $J$ 降维打击为**动态边界映射**的思路，完美利用了系统当前可提供的加加速裕量，保证了只要 MPC 规划出的最优路径满足限制，就不至于让内环执行器超出物理能力（避免进入饱和导致的 Windup 或震荡失控）。_

#### 3. 不可行情况求解策略

当底盘自旋过快，且目标需完全静止时（计算要求 $\dot{\theta}_k=0$ 不在动态平移后的可行域中），上述硬约束之间会发生矛盾现象。在这类不可行（Infeasible）解边界上，通过将速度转化为**软约束（Soft Constraints）**，并对引入的松弛变量在代价函数中施加巨大惩罚项，能保障求解器依旧正常稳定迭代，允许极端的云台暂时跟丢，当度过极限点后立即恢复追踪。

## 3. 基于 TinyMPC 的系统实现与参数整理

TinyMPC 提供了极其轻量级的实时 ADMM 求解器，非常适合在受限设备上运行高频模型预测控制。为了将本规划器无缝对接到 TinyMPC 中，我们将系统模型与约束参数按要求整理如下：

### 3.1 状态与系统矩阵组合

针对全向云台（Yaw 轴和 Pitch 轴相互独立），我们合并构成 4 阶全解耦连续离散状态系统。定义变量下标 $y$ 代表 Yaw 轴（水平方向），$p$ 代表 Pitch 轴（俯仰方向）。

- **状态量维度 (nx) = 4**：即 $\mathbf{x} = [\theta_y, \dot{\theta}_y, \theta_p, \dot{\theta}_p]^T$
- **控制量维度 (nu) = 2**：即 $\mathbf{u} = [\ddot{\theta}_y, \ddot{\theta}_p]^T$

系统的转移与控制矩阵即两轴的参数对角合并：

$$
\mathbf{A}_{sys} = \begin{bmatrix} 1 & T_s & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & T_s \\ 0 & 0 & 0 & 1 \end{bmatrix}, \quad
\mathbf{B}_{sys} = \begin{bmatrix} \frac{1}{2}T_s^2 & 0 \\ T_s & 0 \\ 0 & \frac{1}{2}T_s^2 \\ 0 & T_s \end{bmatrix}
$$

### 3.2 目标权重矩阵 $Q$ 与 $R$

通过对角矩阵限制追踪误差与输入平滑度：

- **状态权重矩阵 $\mathbf{Q} \in \mathbb{R}^{4 \times 4}$**：设为 `diag(q_y_pos, q_y_vel, q_p_pos, q_p_vel)`。其中通常给予**位置项**极高的权重，使之能够跟紧目标；**速度项**适中，用于产生一定阻尼感缓冲。
- **控制权重矩阵 $\mathbf{R} \in \mathbb{R}^{2 \times 2}$**：设为 `diag(r_y_acc, r_p_acc)`。通过增大 $\mathbf{R}$ 矩阵可有效降低云台运动的“冲劲”(Jerk)，使输出更为**丝滑**。

### 3.3 循环步动态边界更新映射

由于目标是化解底盘所产生的时变干扰，每次执行 `tiny_solve()` 求解前，需要根据当前读取到的底盘实时角速度 $\omega_c$ 与角加速度 $\ddot{\theta}_c$ 传感器数据，动态修改状态极值和输入极值矩阵（$\mathbf{x}_{min}, \mathbf{x}_{max}$ 和 $\mathbf{u}_{min}, \mathbf{u}_{max}$）：

**1. 状态极值更新：**
通常放开绝对角度界限（例如 $[-1000, 1000]$ 规避约束失效），重点限制云台在世界坐标系下的绝对角速度极限：

$$
\mathbf{x}_{min} = \begin{bmatrix} -1000 \\ -\omega_{y,max} + \omega_{c,y} \\ -1000 \\ -\omega_{p,max} \end{bmatrix}, \quad
\mathbf{x}_{max} = \begin{bmatrix} +1000 \\ \omega_{y,max} + \omega_{c,y} \\ +1000 \\ \omega_{p,max} \end{bmatrix}
$$

_(注：实车中底盘自旋往往仅在 Yaw 轴发生，Pitch 轴无干扰项，故为通常常数)_

**2. 输入极值更新：**
控制云台最大规划角加速度，避免电机由于加速过激出现力矩饱和：

$$
\mathbf{u}_{min} = \begin{bmatrix} -a_{y,max} + \ddot{\theta}_{c,y} \\ -a_{p,max} \end{bmatrix}, \quad
\mathbf{u}_{max} = \begin{bmatrix} a_{y,max} + \ddot{\theta}_{c,y} \\ a_{p,max} \end{bmatrix}
$$

**C++ 集成伪代码：**

```cpp
// 1. 采集实时的底盘自旋转速和角加速度
double chassis_yaw_vel = get_chassis_yaw_vel();
double chassis_yaw_acc = get_chassis_yaw_acc();

// 2. 在默认能力边界上叠加上层干扰造成的映射
tinyMatrix x_min = default_x_min;
tinyMatrix x_max = default_x_max;
tinyMatrix u_min = default_u_min;
tinyMatrix u_max = default_u_max;

x_min(1, 0) += chassis_yaw_vel;
x_max(1, 0) += chassis_yaw_vel;
u_min(0, 0) += chassis_yaw_acc;
u_max(0, 0) += chassis_yaw_acc;

// 3. 将映射完成的时变边界条件刷新给 TinyMPC
tiny_set_bound_constraints(solver, x_min, x_max, u_min, u_max);

// 4. 更新与目标物体的误差状态差值后执行求解
tiny_solve(solver);
```
