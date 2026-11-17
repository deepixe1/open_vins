**能在 RViz2 中直接作为图像显示的，有下面这些。** 添加 `Image` 显示项即可。

| 话题 | 显示内容 | 主要用途 |
|---|---|---|
| `/camera/image_raw` | 相机原始灰度图，由相机驱动发布 | 检查曝光、清晰度、运动模糊 |
| `/trackhist` | 原图上叠加特征点及历史跟踪轨迹 | **最适合检查视觉跟踪**：特征是否充足、分布是否均匀、移动时是否持续丢失；也可能显示 `init`、`zvupt` 状态文字 |
| `/loop_depth` | `16UC1` 稀疏深度图，单位毫米；无深度的位置为 0 | 读取实际深度数值、检查深度估计；画面大部分为黑色是正常的 |
| `/loop_depth_colored` | 图像上用彩色方块标出特征点的深度远近 | **直观看深度是否合理**；当前代码限制最高约 5 Hz |

其中，OpenVINS 自己输出的是后三路。`loop_depth_colored` 只有特征点位置带有深度颜色，整个画面并没有逐像素深度。

**其他发布器对应的是三维显示或参数数据：**

| 话题 | RViz2 显示类型 | 内容与用途 |
|---|---|---|
| `/poseimu` | `PoseWithCovariance` | 视觉更新后的 IMU 位姿及不确定度，观察当前位置和朝向 |
| `/odomimu` | `Odometry` | 利用 IMU 向较新时刻传播得到的位姿、速度，适合连续运动状态输出 |
| `/pathimu` | `Path` | 累积估计轨迹，观察运动路线、尺度和静止漂移 |
| `/points_msckf` | `PointCloud2` | 近期用于 MSCKF 更新的三维特征点，观察视觉约束的空间分布 |
| `/points_slam` | `PointCloud2` | 滤波器持续维护的 SLAM 特征点，观察局部稀疏地图 |
| `/points_aruco` | `PointCloud2` | ArUco 特征点，启用相应功能后使用 |
| `/points_sim` | `PointCloud2` | 仿真地图中的真值点，实机运行通常没有数据 |
| `/posegt`、`/pathgt` | `Pose`、`Path` | 真值位姿和轨迹，需要提供真值文件或运行仿真，用来对比估计误差 |
| `/loop_pose` | `Odometry` | 与输出特征对应的历史图像时刻的 IMU 位姿，供外部回环模块使用 |
| `/loop_feats` | `PointCloud` | 三维特征点，并附带像素位置、特征 ID 等信息，供外部匹配使用 |
| `/loop_extrinsic` | `Odometry` | IMU 与相机之间的外参，供外部模块解释坐标关系 |
| `/loop_intrinsics` | 通常由程序读取 | `CameraInfo` 相机内参、畸变参数，本身不含图像 |

`Loop closure publishers` 表示**提供给外部回环模块的数据接口**；这些话题的存在不代表 OpenVINS 已经进行了全局回环优化。

你现在日常调试可以先配置这几个显示项：

1. `Image → /trackhist`：看特征跟踪。
2. `Path → /pathimu`：看运动轨迹和漂移。
3. `PointCloud2 → /points_slam`：看三维特征分布。
4. 需要检查深度时，再加 `Image → /loop_depth_colored`。

这些输出的 **Reliability 设为 `Best Effort`**，接收深度可先设为 1。三维轨迹、点云的 `Fixed Frame` 设为 `global`；单独查看 `Image` 不依赖这套三维 TF。

按当前 launch 的默认配置，话题名是 `/trackhist` 这样的形式。仓库 RViz 示例中的 `/ov_msckf/trackhist` 等名称需要按实际话题修改。我刚查询到的 ROS 图目前只有相机和 IMU 话题，启动 OpenVINS 后才能选择上述输出。