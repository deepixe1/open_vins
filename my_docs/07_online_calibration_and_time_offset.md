# 07. 在线标定与时间同步 (Online Calibration & Time Synchronization)

在真实的无人机、手持设备或车载机器人部署中，传感器硬件不可能处于完全理想的环境中：
1. **机械结构变形与热胀冷缩**：会导致离线标定的相机与 IMU 之间的空间刚体相对位姿（外参）产生微小漂移；
2. **硬件传输延迟与时钟晶振漂移**：相机的图像曝光、传导与采集时间戳，通常与 IMU 的高频中断采样时间戳存在毫秒（ms）级的相对时间偏差（Time Offset）；
3. **镜头变焦/温漂**：可能导致相机内参（焦距、畸变）发生微弱变化。

OpenVINS 具备强大的全在线自标定（Online Self-Calibration）能力，能够在滤波运行过程中实时联合估计外参、时间偏移、相机内参以及 IMU 内部结构非正交度。本章详细解析其数学雅可比推导与工程实现。

---

## 1. 相机-IMU 空间外参在线标定 (Spatial Extrinsics)

空间外参定义为从 IMU 机体系 $\{I\}$ 到第 $i$ 个相机系 $\{C_i\}$ 的刚体空间变换：
$$
\mathbf{x}_{ext, i} = \left\{ {}^{C_i}_I\bar{q}, \; {}^I\mathbf{p}_{C_i} \right\}
$$
对应切空间误差状态：
$$
\tilde{\mathbf{x}}_{ext, i} = \begin{bmatrix} {}^{C_i}_I\delta\boldsymbol{\theta}^\top & {}^I\tilde{\mathbf{p}}_{C_i}^\top \end{bmatrix}^\top \in \mathbb{R}^6
$$

### 1.1 外参误差在投影方程中的传播推导
三维路标点在相机系下的坐标表达为：
$$
{}^{C_i}\mathbf{p}_f = {}^{C_i}_I\mathbf{R} \left( {}^I\mathbf{p}_f - {}^I\mathbf{p}_{C_i} \right) = {}^{C_i}_I\mathbf{R} {}^I\mathbf{p}_f + {}^C\mathbf{p}_I
$$
在 OpenVINS 源码规范中，外参平移存储为相机在 IMU 系下的平移向量，因此在第 $j$ 个克隆时刻，点坐标为：
$$
{}^{C_i}\mathbf{p}_f = {}^{C_i}_I\mathbf{R} \cdot {}^I\mathbf{p}_{f, j} + {}^I\mathbf{p}_{C_i} \quad (\text{具体以代码定义方向为准})
$$
对旋转四元数施加左乘扰动 ${}^{C_i}_I\mathbf{R} \approx (\mathbf{I}_3 - \lfloor {}^{C_i}_I\delta\boldsymbol{\theta} \times \rfloor) {}^{C_i}_I\hat{\mathbf{R}}$，对平移施加加性扰动 ${}^I\mathbf{p}_{C_i} = {}^I\hat{\mathbf{p}}_{C_i} + {}^I\tilde{\mathbf{p}}_{C_i}$。

### 1.2 外参雅可比矩阵
点 ${}^{C_i}\mathbf{p}_f$ 关于外参误差的导数为：
- 关于外参姿态角误差 ${}^{C_i}_I\delta\boldsymbol{\theta}$：
  $$
  \frac{\partial {}^{C_i}\mathbf{p}_f}{\partial {}^{C_i}_I\delta\boldsymbol{\theta}} = \lfloor \left( {}^{C_i}\mathbf{p}_f - {}^I\mathbf{p}_{C_i} \right) \times \rfloor
  $$
- 关于外参平移误差 ${}^I\tilde{\mathbf{p}}_{C_i}$：
  $$
  \frac{\partial {}^{C_i}\mathbf{p}_f}{\partial {}^I\tilde{\mathbf{p}}_{C_i}} = \mathbf{I}_3
  $$
代入视觉成像链式法则，像素残差关于相机外参的整体雅可比矩阵为：
$$
\mathbf{H}_{ext} = \frac{\partial \mathbf{z}}{\partial \mathbf{z}_n} \cdot \frac{\partial \mathbf{z}_n}{\partial {}^{C_i}\mathbf{p}_f} \cdot \begin{bmatrix} \lfloor \left( {}^{C_i}\mathbf{p}_f - {}^I\mathbf{p}_{C_i} \right) \times \rfloor & \mathbf{I}_3 \end{bmatrix} \in \mathbb{R}^{2\times 6}
$$
> **源码对照**：参见 [ov_msckf/src/update/UpdaterHelper.cpp](../ov_msckf/src/update/UpdaterHelper.cpp#L390-L405)：
> ```cpp
> if (state->_options.do_calib_camera_pose) {
>   Eigen::MatrixXd dpfc_dcalib = Eigen::MatrixXd::Zero(3, 6);
>   dpfc_dcalib.block(0, 0, 3, 3) = skew_x(p_FinCi - p_IinC);
>   dpfc_dcalib.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
>   H_x.block(2 * c, map_hx[calibration], 2, calibration->size()).noalias() += dz_dpfc * dpfc_dcalib;
> }
> ```

---

## 2. 相机-IMU 时间偏移标定 (Temporal Calibration $t_d$)

在低成本传感器系统中，相机触发信号与 IMU 采样时钟往往没有进行纳秒级硬件电平同步（Hardware Genlock / Trigger）。
两者的物理时钟关系建模为：
$$
t_{imu} = t_{cam} + t_d
$$
其中 $t_d$ 为待标定的时间延迟标量（切空间误差维度为 1）。若在载体高速移动时忽略 $10\text{ms}$ 的时间偏差，会引入高达数十厘米的几何投影错位！

### 2.1 时间偏移在重投影误差中的链式求导 (Li & Mourikis, 2014)
当相机时钟存在微小时间偏差 $\tilde{t}_d$ 时，相当于相机的实际成像物理时刻发生了偏移。根据刚体运动学，空间点在相机坐标系下的相对速度为：
$$
{}^C\dot{\mathbf{p}}_f = \frac{d \, {}^C\mathbf{p}_f}{d t} = -{}^C\mathbf{v}_I - \lfloor {}^C\boldsymbol{\omega} \times \rfloor {}^C\mathbf{p}_f
$$
其中 ${}^C\mathbf{v}_I$ 与 ${}^C\boldsymbol{\omega}$ 为当前相机系下的线速度与角速度。

因此，归一化像素坐标关于时间偏移 $\tilde{t}_d$ 的导数为：
$$
\frac{\partial \mathbf{z}_n}{\partial \tilde{t}_d} = \frac{\partial \mathbf{z}_n}{\partial {}^C\mathbf{p}_f} \cdot {}^C\dot{\mathbf{p}}_f
$$
在离散图像处理中，也可以直接利用特征点在连续图像帧之间的**像素光流运动速度**进行近似：
$$
\frac{\partial \mathbf{z}}{\partial \tilde{t}_d} \approx \begin{bmatrix} \dot{u} \\ \dot{v} \end{bmatrix} = \begin{bmatrix} \frac{u(t) - u(t-\Delta t)}{\Delta t} \\ \frac{v(t) - v(t-\Delta t)}{\Delta t} \end{bmatrix}
$$
结合第 04 章所述的位姿克隆时刻对时间偏移的雅可比扩增修正，OpenVINS 能够以亚毫秒（Sub-millisecond）级精度快速收敛时间偏差 $t_d$。

---

## 3. 相机内参在线标定 (Camera Intrinsics)

OpenVINS 支持在线微调相机的内参系数 $\boldsymbol{\zeta}$（焦距、主点与畸变参数）：
$$
\mathbf{H}_{int} = \frac{\partial \mathbf{z}}{\partial \boldsymbol{\zeta}} \in \mathbb{R}^{2\times \dim(\boldsymbol{\zeta})}
$$
- 对于 Radtan 模型：针对焦距 $f_x, f_y$、主点 $c_x, c_y$ 及多项式畸变 $k_1, k_2, p_1, p_2$ 显式求偏导；
- 对于 Equi 鱼眼模型：针对角度多项式系数求偏导。
在实际应用中，如果相机镜头固定且出厂标定精确，建议默认关闭内参在线标定（`calib_cam_intrinsics: false`），以防止低纹理场景下内参与外参发生不可观测耦合；若镜头受环境温漂较大，可开启该选项。

---

## 4. IMU 轴系正交度与重力敏感度标定 (IMU Intrinsics)

高端惯性导航系统中，MEMS 芯片内部的三个正交敏感轴并非绝对 90° 垂直，且加工精度可能存在尺度瑕疵。OpenVINS 支持基于 Kalibr 模型与 RPNG 模型的 IMU 内部参数标定：

### 4.1 标度因数与轴系非正交矩阵 $\mathbf{D}_m$
对于陀螺仪与加速度计，真实角速度与比力与传感器原始读数之间满足：
$$
\begin{aligned}
\mathbf{a}_{corr} &= \mathbf{D}_a (\mathbf{a}_m - \mathbf{b}_a) \\
\boldsymbol{\omega}_{corr} &= \mathbf{D}_w (\boldsymbol{\omega}_m - \mathbf{b}_g - \mathbf{T}_g \mathbf{a}_{corr})
\end{aligned}
$$
其中：
- $\mathbf{D}_a \in \mathbb{R}^{3\times 3}$ 和 $\mathbf{D}_w \in \mathbb{R}^{3\times 3}$ 为上三角或下三角标度与非正交矩阵（参数化为 6 维向量）；
- $\mathbf{T}_g \in \mathbb{R}^{3\times 3}$（9 维参数）为**陀螺仪重力敏感度矩阵（G-Sensitivity）**，表征外在线加速度对陀螺仪微机械谐振梁产生的额外串扰力矩！
> **源码对照**：见 [ov_msckf/src/state/State.h](../ov_msckf/src/state/State.h#L90-L125) 中的静态函数 `Dm` 与 `Tg`。

---

## 5. 可观测性激发与标定收敛条件分析

在线标定是一把“双刃剑”。根据非线性系统可观测性理论，若载体没有进行充分的几何运动，标定参数在数学上处于**不可观测流形（Unobservable Manifold）**上，强行在线估计会导致参数随机漂移并破坏定位精度！

| 标定参数 | 必须具备的几何机动激发条件 | 典型退化运动（不可观场景） |
|:---|:---|:---|
| **相机-IMU 外参旋转 ${}^C_I\mathbf{R}$** | 绕至少两个不共线的轴发生充分的角速度转动（Pitch / Roll / Yaw） | 纯直线平移、静止悬停 |
| **相机-IMU 外参平移 ${}^I\mathbf{p}_C$** | 剧烈的旋转伴随线加速度耦合机动（产生可感测的哥氏力与向心力） | 匀速旋转、纯平移运动 |
| **时间偏移 $t_d$** | 视觉场景存在明显光流，载体具有剧烈角速度变化 | 平台静止、低速匀速巡航 |
| **IMU 重力敏感度 $\mathbf{T}_g$** | 大机动剧烈加速与大离心力环境 | 常规低动态巡航 |

### 工程实操防护建议
1. **起飞前激活动作**：在开启外参在线标定前，操纵无人机或手持设备在空间中执行“8”字形空间翻滚旋转动作（充分激发各轴角速度与向心加速度），观察日志中外参方差收敛平稳后再执行自主任务；
2. **先验紧约束保护**：在线标定参数的初始协方差不能设得过大，应基于离线标定工具（如 Kalibr）的结果设置合理的置信区间，避免突发外点将外参拉偏。
