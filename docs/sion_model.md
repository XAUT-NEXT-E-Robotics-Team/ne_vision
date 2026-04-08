# sion —— 反小陀螺整车模型

![](imgs/sion.jpeg)

## 1. 简介

## 2. 建系

为更加方便进行观测，我们建立如下坐标系。

## 3. 能观性验证

在自瞄跟踪任务下，跟踪目标和我们是相对运动的，即我们可以自己主动运动（跑打），跟踪目标也会主动运动。因此我们可以对系统进行如下建模。注意这里把自己的输入（加速度）纳入考虑。

$$
\dot{x} = f(x, u) \\
y = h(x)
$$

在RM规则下，地面兵种具有两个半径和两种装甲板高度，我们将平移考虑为CV模型，旋转考虑为CV模型，半径和装甲板高度都认为为定值，我们可建立如下运动学模型

$$
x =
\begin{bmatrix}
p \\
\dot{p} \\
\theta \\
\dot{\theta} \\
z_1 \\
z_2 \\
R_1 \\
R_2 \\
\end{bmatrix}


$$

$$
\dot{x} = f(x, u) =
\begin{bmatrix}
\dot{p} \\
-u \\
\dot{\theta} \\
0 \\
0 \\
0 \\
0 \\
0
\end{bmatrix}
$$

p是一个x和y向的向量。

接下来得到观测函数h，该观测函数在观测到任何一装甲板时的情况都不同，因此，以下为四个观测方程。我们定义下角编号，第一个观测到的装甲板编号为0，然后按照虚拟观测目标（车）的装甲板安装顺序逆时针依次增长。我们可以发现，第一次观测到的装甲板（ID-0）的yaw等于整车的旋转，其逆时针走过一个装甲板的yaw - 90°为整车的yaw，依此类推。注意观测方程是yaw + 90°。

$$
y =
\begin{bmatrix}
p \\
z \\
\theta_y
\end{bmatrix}
$$

$$
y_0 = h_0(x) =
\begin{bmatrix}
p_x + R_1 cos(\theta) \\
p_y + R_1 sin(\theta) \\
z_1 \\
\theta
\end{bmatrix}
$$

$$
y_1 = h_1(x) =
\begin{bmatrix}
p_x + R_2 cos(\theta + \frac{\pi}{2}) \\
p_y + R_2 sin(\theta + \frac{\pi}{2}) \\
z_2 \\
\theta + \frac{\pi}{2}
\end{bmatrix}
$$

$$
y_2 = h_1(x) =
\begin{bmatrix}
p_x + R_1 cos(\theta + \pi) \\
p_y + R_1 sin(\theta + \pi) \\
z_1 \\
\theta + \pi
\end{bmatrix}
$$

$$
y_3 = h_3(x) =
\begin{bmatrix}
p_x + R_2 cos(\theta + \frac{3 \pi}{2}) \\
p_y + R_2 sin(\theta + \frac{3 \pi}{2}) \\
z_2 \\
\theta + \frac{3 \pi}{2}
\end{bmatrix}
$$

我们使用李导数构建非线性系统能观性矩阵，检测发现，该系统能观。即说明协方差有界（系统能够收敛）

## 4. ESIKF建模

TIP：快速学习 [ESKF](https://zhuanlan.zhihu.com/p/441182819) [IEKF](https://zhuanlan.zhihu.com/p/141018958) [各种滤波器](https://zhuanlan.zhihu.com/p/2000687394115057448)

这里我们尝试采用SLAM领域常用的ESIKF观测器，它的优点是对流形友好，非线性对滤波影响小，计算速度相对快于UKF，通过简单修改可以退回普通的ESKF。著名的应用有FAST-LIO、FAST-LIO2。

### 4.1系统的连续形式

我想仿照高翔老师的写法再写一遍。这里我们列写名义变量的系统方程。

$$
\dot{p}_t = v_t \\
\dot{v}_t = a_I + \eta_a \\
\dot{R}_t = \omega_t^\wedge R_t \\
\dot{\omega_t} = 0 + \eta_\beta \\
\dot{z}_{it} = 0 + \eta_z \\
\dot{R}_{it} = 0 + \eta_r
$$

上式中带t下标的表示真值。注意，这里的 $a_I$ 是相对于IMU坐标系的加速度，**注意这里需要取负号，由于对面车是相对于我们运动的，我之前写的时候忘记了**，因此这里不考虑云台旋转的情况。i = 1 或 2

接下来我们推导误差状态方程。首先定义误差状态变量为：

$$
p_t = p + \delta p \\
v_t = v + \delta v \\
R_t = \delta R R \\
\omega_t = \omega + \delta \omega \\
z_{it} = z_i + \delta z_i \\
R_{it} = R_i + \delta R_i
$$

这里不带t下标的变量就是名义状态。除R外，我们可以将名义状态移动到左侧，并对误差状态求导，得到它们的导数形式：

$$
\delta \dot{p} = \delta v \\
\delta \dot{v} = \eta_a \\
\delta \dot{\omega} = \eta_{\beta} \\
\delta \dot{z}_i = \eta_z \\
\delta \dot{R}_i = \eta_r
$$

由于R是SO(2)所以我们需特别推导R的误差状态方程。我们可以对 $R_t$ 求导（如果诸位有数学恐惧，这里可以直接跳过，看看结论，其实是非常简单的），注意一点，这里我们需要推的是旋转的误差的导数 $\delta \dot{\theta}$

$$
\dot{R}_t =
Exp(\delta \theta)\dot{R} + \dot{Exp(\delta \theta)}R =
\omega_t^\wedge R_t
$$

$$
\dot{Exp(\delta \theta)} =
\delta \dot{\theta}^\wedge Exp(\delta \theta)
$$

带入该式并对R求导可以得到

$$
\dot{R}_t =
Exp(\delta \theta)\omega^\wedge R + \delta \dot{\theta}^\wedge Exp(\delta \theta) R =
\omega_t^\wedge R_t
$$

由于在SO(2)下，反对称和Exp具有交换律，所以可以化简

$$
Exp(\delta \theta) (\omega^\wedge + \delta \dot{\theta}^\wedge)R =
\omega_t^\wedge R_t =
\omega_t^\wedge \delta R R =
\omega_t^\wedge Exp(\delta \theta) R
$$

右乘R的转置可以消除R

$$
Exp(\delta \theta) (\omega^\wedge + \delta \dot{\theta}^\wedge)=
\omega_t^\wedge Exp(\delta \theta)
$$

进一步利用交换律，可以消去Exp

$$
\omega^\wedge + \delta \dot{\theta}^\wedge = \omega_t^\wedge
$$

显然，我们可以得到：（带入 $ \omega_t $）

$$
\delta \dot{\theta} = \omega_t - \omega = \delta \omega
$$

这里其实得到了一个显而易见的结论。然而最重要的是，我们这里将比较难以处理的旋转，转换到了误差空间，变成了一个欧几里得空间下的标量来处理，这样完美的避免了流形计算。

于是乎我们得到：

$$
\delta \dot{p} = \delta v \\
\delta \dot{v} = \eta_a \\
\delta \dot{\theta} = \delta \omega \\
\delta \dot{\omega} = \eta_{\beta} \\
\delta \dot{z}_i = \eta_z \\
\delta \dot{R}_i = \eta_r
$$

### 4.2 系统的离散形式

接下来我们来列写系统名义变量的状态方程的离散形式：

$$
p(t + \Delta t) = p + v \Delta t \\
v(t + \Delta t) = v + a \Delta t \\
R(t + \Delta t) = Exp(\omega \Delta t) R \\
\omega(t + \Delta t) = \omega \\
z_i(t + \Delta t) = z_i \\
R_i(t + \Delta t) = R_i
$$

这里偷个懒省略等式右侧的时间t。注意，a是我们的加速度输入。

然后我们可以列写误差状态方程：

$$
\delta p(t + \Delta t) = \delta p + \delta v \Delta t \\
\delta v(t + \Delta t) = \delta v + \eta_a \Delta t \\
\delta \theta (t + \Delta t) = \delta \theta +  \delta w \Delta t \\
\delta \omega(t + \Delta t) = \delta \omega + \eta_\beta \Delta t \\
\delta z_i(t + \Delta t) = \delta z_i + \eta_z \Delta t\\
\delta R_i(t + \Delta t) = \delta R_i + \eta_r\Delta t
$$

由于我们实际上得不到什么连续的噪声项（其实可以，比如加速度计的零偏，不过，由于我们的主要误差并不在加速计哪里，加速度计比pnp出来的东西精确的多）所以我们所有的噪声项，本质上都是普通的随机变量。

### 4.3 预测

关于预测和更新的步骤，我们可以参考 [FAST-LIO2](https://arxiv.org/abs/2107.06829) 这似乎是ESIKF的开山之作（虽然作者并没起这个名字）。为了和原文相符，我们修改上述符号定义。

定义$\~{x}$为误差状态。$\hat{x}$为先验，$x_k$为真实状态（注意，不带上标的均为真实状态），由于预测和更新的频率不同（往往IMU可以达到200HZ），因此我们需要区分预测和观测过程，我们记下角标i为预测过程，K为观测过程。

由4.2节，我们可以得到系统的状态方程，同EKF，我们可以简单的列写下式：

$$
\mathbf{\hat{x}_{i+1}} =
  f(\mathbf{\hat{x}_i}, \mathbf{u_i}, \mathbf{0})
$$

$$
\mathbf{\hat{P}_{i+1}} =
\mathbf{F_{\~{x}_i}\hat{P}_i}
\mathbf{F_{\~{x}_i}^\top} +
\mathbf{F_{{w_i}}\hat{Q}_i}
\mathbf{F_{{w_i}}^\top}
$$

$$
\mathbf{\hat{x}_0} = \mathbf{\bar{x}_{k-1}}
$$

$$
\mathbf{\hat{P}_0} = \mathbf{\bar{P}_{k-1}}
$$

根据EKF的套路，很显然，P为先验协方差，两个F为雅可比，Q为过程噪声。由于预测数大于观测数，所以有：$i \geq 0$，因此我们可以考虑初始值如上。我们可以计算两雅可比如下：

$$
\mathbf{F}_{\tilde{\mathbf{x}}_i} =
\frac{\partial (\mathbf{x}_{i+1} \boxminus
\hat{\mathbf{x}}_{i+1})}{\partial \tilde{\mathbf{x}}_i}
\bigg|_{\tilde{\mathbf{x}}_i=0, \mathbf{w}_i=0}


$$

$$
\mathbf{F}_{\mathbf{w}_i} =
\frac{\partial (\mathbf{x}_{i+1} \boxminus
\hat{\mathbf{x}}_{i+1})}{\partial \mathbf{w}_i}
\bigg|_{\tilde{\mathbf{x}}_i=0, \mathbf{w}_i=0}
$$

从误差状态的定义来看，我们可以发现，上述求导过程本质上是进行如下求导。

$$
\mathbf{F}_{\tilde{\mathbf{x}}_i} =
\frac{\partial F(\mathbf{\~{x}_i}, \mathbf{w_i})}
{\partial \mathbf{\~{x}}_i}
\bigg|_{\tilde{\mathbf{x}}_i=0, \mathbf{w}_i=0}
$$

$$
\mathbf{F}_{\mathbf{w}_i} =
\frac{\partial F(\mathbf{\~{x}_i}, \mathbf{w_i})}
{\partial \mathbf{w}_i}
\bigg|_{\tilde{\mathbf{x}}_i=0, \mathbf{w}_i=0}
$$

其中F为误差状态的预测函数。

### 4.4 残差计算

由于需要考虑到迭代过程，我们考虑上角标k表示迭代索引。我们假设有一个抽象的观测函数能够对真实状态进行一次观测而得到传感器的观测值（我们过后再讨论该函数）。

$$
\mathbf{z} = \mathbf{h}(\mathbf{x}_k, \mathbf{n})
$$

该式中，n为传感器噪声，我们可以对其在$\mathbf{\hat{x}}_k^\kappa$进行线性化如下：

$$
\mathbf{z} \approx \mathbf{h}(\mathbf{\hat{x}}_k^\kappa, \mathbf{0}) +
\mathbf{H}^\kappa \mathbf{\~{x}}_k^\kappa + \mathbf{v}


$$

$$
0 = -\mathbf{r}^\kappa +
\mathbf{H}^\kappa \mathbf{\~{x}}_k^\kappa + \mathbf{v}
$$

$$
\mathbf{r}^\kappa = \mathbf{z} - \mathbf{h}(\mathbf{\hat{x}}_k^\kappa, \mathbf{0})
$$

当k=0时（迭代前）：

$$
\mathbf{\hat{x}}_k^\kappa = \mathbf{\hat{x}}_k
$$

由误差状态的定义，我们可以同样在第k轮迭代处定义：

$$
\mathbf{\~{x}}_k^\kappa = \mathbf{x}_k \boxminus \mathbf{\hat{x}}^\kappa_k
$$

因此我们可以得到真值的表达式，同时，我们可以如下计算雅可比矩阵H：

$$
\mathbf{H}^\kappa =
\frac{\partial \mathbf{h}(\mathbf{\~{x}}_k^\kappa \boxplus
\mathbf{\hat{x}}^\kappa_k , \mathbf{n})}
{\partial \mathbf{\~{x}}_k^\kappa}
\bigg|_\mathbf{\~{x}^\kappa_k=0, \mathbf{n}=0}
$$

其中：$\mathbf{r}^\kappa$即我们要求的残差。

### 4.5 迭代更新

为迭代更新，我们需要获得先验估计（误差状态）的协方差。由于是对于每一次迭代而言，我们需要获得的是：$\mathbf{\~{x}}^\kappa_k$的协方差，由于我们已知$\mathbf{\~{x}}_k$的协方差。我们可以进行如下推导：

$$
\mathbf{\~{x}}_k = \mathbf{x}_k \boxminus \hat{\mathbf{x}}_k
=  (\hat{\mathbf{x}}_k^\kappa \boxplus \tilde{\mathbf{x}}_k^\kappa)
\boxminus \hat{\mathbf{x}}_k \sim \mathcal{N}(\mathbf{0}, \hat{\mathbf{P}}_k)
$$

对其进行线性化（在$\mathbf{~{x}}^\kappa_k = 0$处进行泰勒展开），可以得到：

$$
\mathbf{\~{x}}_k \approx \hat{\mathbf{x}}_k^\kappa \boxminus
\mathbf{\hat{x}}_k + \mathbf{J}^\kappa \mathbf{\~{x}}^\kappa_k
$$

注意我们要求的是$\mathbf{\~{x}}^\kappa_k$的协方差，我们已知：

$$
\hat{\mathbf{x}}_k^\kappa \boxminus
\mathbf{\hat{x}}_k + \mathbf{J}^\kappa \mathbf{\~{x}}^\kappa_k
\sim \mathcal{N}(\mathbf{0}, \hat{\mathbf{P}}_k)
$$

根据高斯分布的性质，我们可以将加号左侧放入分布中：

$$
\mathbf{J}^\kappa \mathbf{\~{x}}^\kappa_k
\sim \mathcal{N}(-(\hat{\mathbf{x}}_k^\kappa \boxminus
\mathbf{\hat{x}}_k), \hat{\mathbf{P}}_k)
$$

由均值和方差的基本性质，我们可以把雅可比放入分布中：

$$
\mathbf{\~{x}}^\kappa_k
\sim \mathcal{N}(-(\mathbf{J}^\kappa)^{-1}(\hat{\mathbf{x}}_k^\kappa
\boxminus
\mathbf{\hat{x}}_k),
(\mathbf{J}^\kappa)^{-1} \hat{\mathbf{P}}_k
{(\mathbf{J}^\kappa)^\top}^{-1} )
$$

显然我们可以得到$\mathbf{\~{x}}^\kappa_k$的协方差，由于每次迭代都需要计算，此处略去迭代索引记为：

$$
\mathbf{P} = (\mathbf{J}^\kappa)^{-1} \hat{\mathbf{P}}_k
{(\mathbf{J}^\kappa)^\top}^{-1}
$$

对于观测噪声，我们将观测方程中的噪声提前可得：

$$
-\mathbf{v} = \mathbf{r}^\kappa +
\mathbf{H}^\kappa \tilde{\mathbf{x}}_k^\kappa
\sim \mathcal{N}(\mathbf{0}, \mathbf{R})
$$

接下来我们构建最小二乘问题并使用类似高斯牛顿法求解（我也不是很懂）总之，可以按照如下方式计算卡尔曼增益并更新观测值：

$$
\mathbf{K} = \left( \mathbf{H}^\top \mathbf{R}^{-1} \mathbf{H}
 + \mathbf{P}^{-1} \right)^{-1} \mathbf{H}^\top \mathbf{R}^{-1}
$$

$$
\hat{\mathbf{x}}_k^{\kappa+1} = \hat{\mathbf{x}}_k^\kappa \boxplus
\bigl( \mathbf{K} \mathbf{r}^\kappa_k - (\mathbf{I} - \mathbf{K} \mathbf{H})(\mathbf{J}^\kappa)^{-1}
(\hat{\mathbf{x}}_k^\kappa \boxminus \hat{\mathbf{x}}_k) \bigr)
$$

我们计算两次更新差值，如果小于一定值则停止更新。我们最终得到后验信息：

$$
\mathbf{x} = \mathbf{x}_e, \quad \mathbf{P} = (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P}
$$
