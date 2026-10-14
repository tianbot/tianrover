# TianRover ROS 2 平面里程计融合

## 输入核验与选择

2026-10-08 NX 原始输入 `/mars/odom` 和 `/mars/imu` 均约 50 Hz。
20 秒输入采样包含 1000 帧 odom、1001 帧 IMU，底盘 XY 无位移；
IMU 角速度 Z 标准差约 0.00448 rad/s、加速度 Z 均值约 9.798 m/s²，
IMU 航向与原始 odom 航向相差约 1.286 rad。驱动输出的全部协方差为零。
数据保存于 NX `/tmp/tianrover-ekf-input-audit.json`，仅凭轮速近零不构成物理静止或坐标方向验收。

参考本机 `tianracer_core/param/tianbot_ekf_params.yaml` 的 `two_d_mode` 和底盘 VX/VY 选择。
采用 ROS 2 `robot_localization/ekf_node`，而不是 ROS 1 `ekf_localization_node`。
固件已有姿态四元数，不重复运行 TianRacer 的互补滤波器。

- 底盘只融合 VX、VY，避免把同源累计位置与速度重复计入；不强制横向速度为零。
- IMU 只融合相对 yaw，保留运动航向变化但去除启动角度偏移。
  `imu0_differential=false`，避免对唯一姿态来源再做差分使航向协方差无界增长。
- 不融合未标定的加速度，不重复融合姿态和同源陀螺积分。
- `ekf_inputs` 给全零协方差提供可调初值，保留有效测量及时间戳。
  默认速度和航向方差均为 0.0025；这不是从静止数据获得的完整传感器误差模型。
  缺失 IMU 姿态、无效四元数、非有限速度、异常帧或被选通量的无效协方差会被拒绝。

依据：[上游 Humble 配置指南](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/configuring_robot_localization.rst)、
[上游状态估计参数](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml)。

## 启动、切换与回退

已有底盘时先运行 `ros2 launch tianrover_localization ekf.launch.py`。
默认读取 `<namespace>/odom`、`<namespace>/imu`，输出 `<namespace>/odometry/filtered`，
`publish_tf=false`，可并行比较；输出 frame 为 `<namespace>/odom_ekf`，
原始和融合坐标原点不相同。正式融合 bringup 则使用 `<namespace>/odom`。

正式启用 `ros2 launch tianrover_bringup bringup.launch.py use_ekf:=true`，或外部配置 `TIANROVER_USE_EKF=true`。
原始 odom 改到 `odom_raw`；EKF 输出 `odom` 并独占 `odom → base_footprint` TF。
SLAM、Nav2、RViz 的默认帧/话题保持兼容。先停旧 bringup 和并行 EKF，避免重复输入及 TF 发布。
切换在车停稳、导航停止时进行，融合从新原点开始，需要重新初始化 AMCL。
回退：设置 false，停止融合 bringup，重新启动原模式。
单独 `ekf.launch.py publish_tf:=true` 仅用于底盘 TF 已关闭的场景。

外部 `TIANROVER_EKF_PARAMS` 可覆盖滤波设置，`TIANROVER_EKF_VELOCITY_VARIANCE` 和
`TIANROVER_EKF_YAW_VARIANCE` 可设置缺失协方差的初值。改输入选通量时必须同时调整
输入适配器的协方差检查/默认值，不能给新选通量继续使用零协方差。

## 验证要求

`tianrover_localization/test/test_ekf_inputs.py` 覆盖协方差保留、补值和无效输入拒绝。
`tianrover_tests/test/ekf_runtime_check.py` 在 ROS_DOMAIN_ID=182 使用真实 bringup 与 EKF、关闭硬件适配器，
输入合成的静止、1 m 直行、0.8 rad 转向，检查输出、相对航向、单一融合话题及 TF。
此测试验证融合链路，不证明实车精度。

实车验收需要记录原始 odom、IMU、融合输出和 TF，核对：

1. 静止漂移、输出频率、诊断、协方差和 IMU 坐标轴方向。
2. 已知距离的前进/后退与左右转向；在相同起点对齐坐标后比较终点位置/航向误差。
3. 优先使用独立定位真值或测量标记；轮速与 IMU 可能已经在固件中相关，不能把两者当作独立真值。

只有实际误差比较才能证明精度改善；EKF 不能消除轮径误差、严重打滑或 IMU 航向漂移。

## 本次验证结果

NX 隔离域的实际 EKF bringup 测试通过：1 m 合成直行输出约 0.994 m，0.8 rad 合成转向、
相对启动航向和动态 TF 均通过；没有连接硬件或发布运动指令。
本机输入处理测试通过，NX 包构建和测试通过。
日志：`/tmp/tianrover-ekf-build.log`、`/tmp/tianrover-ekf-fixture.log`、
`/tmp/tianrover-ekf-test-result.log`。

现场保留原 bringup，另启 `ekf.launch.py`，PID 起始为 301506，默认不发布 TF。
30 秒检查窗口内：原始 odom 1409 帧、约 50 Hz；融合 884 帧、约 30 Hz；
两者都只有一个发布者，融合状态有限、协方差正值，EKF 两项诊断均 OK。
该段轮速近零，原始 XY 位移约 0.061 mm；原始航向变化 -0.02607 rad（约 -1.49°），
融合航向变化 -0.00228 rad（约 -0.13°）。这是该输入数据段的变化比较，
不是经过物理静止确认或外部真值验证的精度结论。
数据保存于 `/tmp/tianrover-ekf-live-audit.json`，实时日志为 `/tmp/tianrover-ekf-live.log`。

未接管实际底盘 TF，未将部署默认改为融合模式；后续需人工已知距离/转角测试，
验证 IMU 正负方向、运动误差及协方差后再切换。
