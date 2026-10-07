# 05. SLAM 特征更新与第一估计雅可比理论 (SLAM Features & FEJ Theory)

在视觉惯性里程计中，纯粹基于滑动窗口的 MSCKF 算法虽然计算效率极高，但其本质上只利用了瞬态相对位姿约束。当载体进行低动态悬停或缓慢移动时，缺乏长期路标会导致累积漂移加速。为此，OpenVINS 引入了**SLAM 长期特征更新**与**第一估计雅可比（First Estimate Jacobians, FEJ）一致性保证理论**。

本章系统剖析 SLAM 路标的几何参数化形式、延迟初始化与锚点迁移算法，并从系统可观测性分析出发，推导标准 EKF 不一致性的数学根源及 FEJ 的解决方案。

---

## 1. SLAM 长期特征与 MSCKF 特征的互补机制

OpenVINS 采用“MSCKF 特征 + SLAM 特征”的混合后端架构：

```
+-------------------------------------------------------------+
|                      前端提取的视觉特征                      |
+-------------------------------------------------------------+
                               |
               +---------------+---------------+
               |                               |
               v                               v
       [跟踪寿命短 / 丢失]             [跟踪寿命长 / 稳定跨窗]
               |                               |
               v                               v
      【MSCKF 边缘化消元】             【SLAM 状态变量常驻】
      - 不占状态协方差维度             - 直接进入状态向量与大协方差
      - 提取多状态约束后丢弃           - 提供长期空间几何锚定
      - 快速约束滑动窗口高频运动       - 强力抑制中长程位置漂移
```

- **MSCKF 特征**：适用于纹理快速更迭的短寿命特征，几何消元后不留在状态向量中，保证算力恒定；
- **SLAM 特征**：挑选视野中跟踪寿命极长、视差充分的优质特征，显式注入状态向量中，作为空间绝对路标约束。

---

## 2. SLAM 路标的几何参数化推导

在 EKF 中，路标点状态的参数化形式直接决定了重投影测量方程的非线性程度与滤波更新稳定性。OpenVINS 在 [ov_core/src/types/LandmarkRepresentation.h](../ov_core/src/types/LandmarkRepresentation.h) 中实现了四种主流表征：

### 2.1 全局三维笛卡尔坐标 (`GLOBAL_3D`)
直接用特征在全局系下的三维坐标表示：
$$
\mathbf{x}_f = {}^G\mathbf{p}_f = \begin{bmatrix} X_G \\ Y_G \\ Z_G \end{bmatrix} \in \mathbb{R}^3
$$
- **优点**：物理意义直观，与机体运动完全解耦；
- **缺点**：对于大景深或远距离特征，由于初次三角化深度不确定度极大，在笛卡尔空间下的高斯分布假设被严重扭曲，容易导致滤波器更新发散。

### 2.2 锚定三维笛卡尔坐标 (`ANCHORED_3D`)
将特征点表示在首次观测到它的“锚定克隆位姿”相机坐标系 $\{A\}$ 中：
$$
\mathbf{x}_f = {}^A\mathbf{p}_f = \begin{bmatrix} X_A \\ Y_A \\ Z_A \end{bmatrix} \in \mathbb{R}^3
$$
其与全局坐标的关系为：
$$
{}^G\mathbf{p}_f = {}^G_A\mathbf{R} {}^A\mathbf{p}_f + {}^G\mathbf{p}_A
$$
- **优点**：相对于局部相机的坐标尺度较小，数值稳定性优于全局坐标。

### 2.3 锚定逆深度参数化 (`ANCHORED_INVERSE_DEPTH`)
针对室外远景与大视差场景，将特征点参数化为方向射线加上逆深度 $\rho = 1/Z_A$（Montiel et al., RSS 2006）：
$$
\mathbf{x}_f = \begin{bmatrix} \alpha \\ \beta \\ \rho \end{bmatrix} = \begin{bmatrix} X_A / Z_A \\ Y_A / Z_A \\ 1 / Z_A \end{bmatrix} \in \mathbb{R}^3
$$
特征在锚定系下的三维坐标恢复为：
$$
{}^A\mathbf{p}_f = \frac{1}{\rho} \begin{bmatrix} \alpha \\ \beta \\ 1 \end{bmatrix}
$$
- **数学优势**：当空间点距离相机极远（$Z_A \to \infty$）时，$\rho \to 0$，逆深度协方差仍能保持平滑有界的高斯特性，彻底解决了标准 3D 坐标在远景处协方差无穷发散的问题。

### 2.4 锚定 MSCKF 逆深度 (`ANCHORED_MSCKF_INVERSE_DEPTH`)
在 OpenVINS 中被设为默认的极简高性能表征：
- 在首次三角化时，直接将特征点在锚定相机归一化平面的观测射线方向 $[u_0, v_0, 1]^\top$ 固化为常数向量；
- 状态向量中仅维护该特征点的**单自由度逆深度** $\rho \in \mathbb{R}^1$！
$$
{}^A\mathbf{p}_f = \frac{1}{\rho} \begin{bmatrix} u_0 \\ v_0 \\ 1 \end{bmatrix}
$$
- **计算优势**：每个 SLAM 路标在协方差矩阵中仅占用 1 个维度，协方差矩阵尺寸大幅压缩，计算速度极快且收敛极其稳定。

---

## 3. SLAM 特征生命周期管理与锚点迁移

### 3.1 延迟初始化 (Delayed Initialization)
当某个特征被选为 SLAM 特征候选时，不能立即简单地将其强行插入状态向量，因为其初始深度不确定度可能极大。
OpenVINS 在 [ov_msckf/src/update/UpdaterSLAM.cpp](../ov_msckf/src/update/UpdaterSLAM.cpp#L110-L190) 中实现了**延迟初始化**：
1. 收集该特征在当前滑窗内的全部历史观测；
2. 构建关于新路标状态 $\tilde{\mathbf{x}}_L$ 与现有系统状态 $\tilde{\mathbf{x}}_R$ 的联合线性方程：
   $$
   \mathbf{r} = \mathbf{H}_R \tilde{\mathbf{x}}_R + \mathbf{H}_L \tilde{\mathbf{x}}_L + \mathbf{n}
   $$
3. 利用 Givens 旋转将方程解耦为两部分：一部分用于更新现有状态，另一部分形成可直接求解新特征初值及其协方差的独立子系统；
4. 唯有通过卡方一致性检验后，才调用 `StateHelper::initialize` 将其正式扩增进大协方差矩阵。

### 3.2 锚点平滑迁移 (Change Anchors)
对于基于锚定坐标系表达的特征，其几何基准依赖于某个特定的历史克隆位姿 $\{A_{old}\}$。
随着滑动窗口向前推进，老克隆位姿最终必须被边缘化剔除。若其锚定特征仍处于活跃跟踪状态，必须将其锚点坐标系无缝迁移到最新的克隆位姿 $\{A_{new}\}$ 上！

设空间点在旧锚点系下为 ${}^{A_{old}}\mathbf{p}_f$，新锚点相对于旧锚点的位姿为 $\{{}^{A_{new}}_{A_{old}}\mathbf{R}, {}^{A_{old}}\mathbf{p}_{A_{new}}\}$。
空间坐标变换关系为：
$$
{}^{A_{new}}\mathbf{p}_f = {}^{A_{new}}_{A_{old}}\mathbf{R} \left( {}^{A_{old}}\mathbf{p}_f - {}^{A_{old}}\mathbf{p}_{A_{new}} \right)
$$
通过求导构建坐标迁移雅可比矩阵 $\mathbf{J}_{anchor}$，不仅重新计算特征的均值，而且严格变换相关联的协方差子块，从而在保证全局几何约束完全等价的前提下，安全剔除老克隆位姿！
> **源码对照**：详见 [ov_msckf/src/update/UpdaterSLAM.cpp](../ov_msckf/src/update/UpdaterSLAM.cpp#L480-L580) 的 `change_anchors` 与 `perform_anchor_change`。

---

## 4. 第一估计雅可比 (FEJ) 理论与系统不一致性剖析

标准 EKF-VIO 在长距离运行时常出现发散或不可解释的剧烈漂移，其根本原因在于**系统不可观测子空间的虚假信息增益（Spurious Information Gain）**。

### 4.1 VIO 系统的可观测性分析与 4-DoF 零空间

对于一个标准单目/双目 VIO 系统，当没有外部绝对定位（如 GPS、MoCap）辅助时，以下自由度在物理上是不可测量的（不可观测的）：
1. **全局三维平移位置（3 DoF）**：平移传感器无法改变内部相对运动与相对视觉视差；
2. **绕重力矢量的偏航角 Yaw（1 DoF）**：加速度计能够感应重力矢量的大小和俯仰/横滚角（Roll/Pitch），但绕重力轴旋转时，加速度计测得的重力分量完全不变。

因此，VIO 系统具有 **4 个不可观测自由度（4-DoF Unobservable Subspace）**。

定义线性化系统的系统可观测性矩阵为：
$$
\mathcal{O} = \begin{bmatrix}
\mathbf{H}_0 \boldsymbol{\Phi}_{(0, 0)} \\
\mathbf{H}_1 \boldsymbol{\Phi}_{(1, 0)} \\
\mathbf{H}_2 \boldsymbol{\Phi}_{(2, 0)} \\
\vdots \\
\mathbf{H}_k \boldsymbol{\Phi}_{(k, 0)}
\end{bmatrix}
$$
其理论右零空间矩阵 $\mathcal{N} \in \mathbb{R}^{D \times 4}$ 必须满足：
$$
\mathcal{O} \mathcal{N} \equiv \mathbf{0} \quad \Longleftrightarrow \quad \mathbf{H}_k \boldsymbol{\Phi}_{(k, 0)} \mathcal{N} = \mathbf{0}, \quad \forall k \ge 0
$$
对于包含 IMU 状态与路标点的简化系统，该零空间基底显式表达为（Huang et al., IJRR 2010）：
$$
\mathcal{N} = \begin{bmatrix}
{}^I_G\mathbf{R}_0 {}^G\mathbf{g} & \mathbf{0}_{3\times 3} \\
-\lfloor {}^G\mathbf{p}_{I_0} \times \rfloor {}^G\mathbf{g} & \mathbf{I}_{3} \\
-\lfloor {}^G\mathbf{v}_{I_0} \times \rfloor {}^G\mathbf{g} & \mathbf{0}_{3\times 3} \\
-\lfloor {}^G\mathbf{p}_f \times \rfloor {}^G\mathbf{g} & \mathbf{I}_{3}
\end{bmatrix}
$$
其中第一列对应绕重力轴的 Yaw 旋转，后三列对应全局三维平移。

---

### 4.2 标准 EKF 产生不一致性（Inconsistency）的数学机理

标准扩展卡尔曼滤波在每个时间步总是使用**当前最新的状态估计值**重新线性化：
1. **状态转移矩阵连续性断裂**：
   在时间区间 $[t_{k-1}, t_{k+1}]$ 内，理想复合状态转移矩阵应满足 $\boldsymbol{\Phi}_{(k+1, k-1)} = \boldsymbol{\Phi}_{(k+1, k)} \boldsymbol{\Phi}_{(k, k-1)}$。
   但在标准 EKF 中：
   - 前一段积分 $\boldsymbol{\Phi}_{(k, k-1)}$ 是在 $t_{k-1}$ 更新后估计值处展开；
   - 在 $t_k$ 时刻，执行了测量更新，状态均值发生跳变：$\hat{\mathbf{x}}_k \to \hat{\mathbf{x}}_{k|k}$；
   - 后一段积分 $\boldsymbol{\Phi}_{(k+1, k)}$ 却在更新后的新状态 $\hat{\mathbf{x}}_{k|k}$ 处展开！
   这导致：
   $$
   \boldsymbol{\Phi}_{(k+1, k-1)}(\hat{\mathbf{x}}_{k+1|k}, \hat{\mathbf{x}}_{k-1|k-1}) \neq \boldsymbol{\Phi}_{(k+1, k)}(\hat{\mathbf{x}}_{k+1|k}, \hat{\mathbf{x}}_{k|k}) \cdot \boldsymbol{\Phi}_{(k, k-1)}(\hat{\mathbf{x}}_{k|k-1}, \hat{\mathbf{x}}_{k-1|k-1})
   $$
2. **观测雅可比与状态转移零空间失配**：
   测量雅可比 $\mathbf{H}_k(\hat{\mathbf{x}}_{k|k})$ 采用最新估计值计算，而历史可观测矩阵基底累积自旧状态估计值。
   将零空间带入每一行时：
   $$
   \mathbf{H}_k(\hat{\mathbf{x}}_{k|k}) \boldsymbol{\Phi}_{(k, 0)} \mathcal{N} \neq \mathbf{0} !
   $$
   零空间约束被数学硬性破坏！不可观测子空间的维度从 4 错误衰减为 3！

#### 物理后果
- 滤波器错误地认为原本不可观测的 Yaw 角变得“可观测”了；
- 滤波器在 Yaw 方向非法提取到了虚假信息增益（Spurious Information Gain）；
- 协方差矩阵中对应方向的方差被异常缩小，滤波器变得“过度自负（Overconfident）”；
- 真实估计误差远远超出滤波器计算出的 $3\sigma$ 置信区间，滤波器无法再信任后续正确的视觉测量，最终发生剧烈发散！

---

### 4.3 FEJ 解决方案与代码落地点

第一估计雅可比（First Estimate Jacobians - FEJ）策略的核心思想极为优雅简洁：
**“系统状态变量在整个滤波生命周期中，一旦其计算了第一次雅可比矩阵，该状态作为线性化点的取值就被永久固化（First Estimate），所有后续涉及该状态的转移矩阵与观测雅可比，必须且只能在该初次估计值处计算！”**

```
状态演化:   x_0 ----更新----> x_1 ----更新----> x_2 ----更新----> x_k
状态均值:   \hat{x}_0         \hat{x}_1         \hat{x}_2         \hat{x}_k  (持续更新)
FEJ 线性点: x_0_fej           x_1_fej           x_2_fej           x_k_fej    (永久固化!)
```

#### OpenVINS 中的 FEJ 代码实现与工程保障

1. **状态变量内部双存储机制**：
   在基类 [ov_core/src/types/Type.h](../ov_core/src/types/Type.h) 及派生类中，均维护两个变量：
   - 当前更新均值：如 `_q`, `_p`, `_value`；
   - 初次估计均值：如 `_q_fej`, `_p_fej`, `_fej`。
2. **Propagator 中的 FEJ 姿态采样**：
   文件路径：[ov_msckf/src/state/Propagator.cpp](../ov_msckf/src/state/Propagator.cpp#L730-L735)
   ```cpp
   Eigen::Matrix3d R_k = state->_imu->Rot();
   Eigen::Vector3d v_k = state->_imu->vel();
   Eigen::Vector3d p_k = state->_imu->pos();
   if (state->_options.do_fej) {
     R_k = state->_imu->Rot_fej();
     v_k = state->_imu->vel_fej();
     p_k = state->_imu->pos_fej();
   }
   Eigen::Matrix3d dR_ktok1 = quat_2_Rot(new_q) * R_k.transpose();
   ```
   计算状态转移矩阵时，若开启 `do_fej`，严格使用 `_fej` 姿态构建 $\mathbf{F}$ 与 $\mathbf{G}$！
3. **视觉观测雅可比中的 FEJ 投影**：
   文件路径：[ov_msckf/src/update/UpdaterHelper.cpp](../ov_msckf/src/update/UpdaterHelper.cpp#L350-L362)
   ```cpp
   if (state->_options.do_fej) {
     R_GtoIi = clone_Ii->Rot_fej();
     p_IiinG = clone_Ii->pos_fej();
     p_FinIi = R_GtoIi * (p_FinG_fej - p_IiinG);
     p_FinCi = R_ItoC * p_FinIi + p_IinC;
   }
   ```
   雅可比计算中采用固化的 `clone_Ii->Rot_fej()` 与 `p_FinG_fej`，从根本上保证了 $\mathbf{H}_k \boldsymbol{\Phi}_{(k, 0)} \mathcal{N} \equiv \mathbf{0}$ 在浮点精度下严格成立！
