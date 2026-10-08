# 验证记录（2026-10-07）

目标设备：192.168.1.42，tianbot-nx，Ubuntu 22.04.5 / ARM64 / ROS 2 Humble。
新工作区：`~/tianrover_ws`。配置：`~/.config/tianrover/my_tianrover.sh`，默认机器人名 `mars`。
复用原 `~/tianbot_ws` 的底盘驱动和模型网格；新工作区有独立的 RoboSense SDK 副本及配置路径兼容补丁。
原 Tianbot 与雷达源码中的已有未提交改动保持原样。

## 可复用入口与部署配置

- 固定入口：bringup、base、lidar、description、scan、slam、map_save、navigation、rviz。
- 通过环境选择组件名或外部适配器目录，无需增加入口中的车型分支。
- 测试外部环境文件先设置 `hook_a`，修改为 `hook_b` 后再次 source 同一 install setup，立即生效；未重新编译。
- 部署文件和地图位于用户目录，不写版本库。

## NX 自动验证

`colcon build --symlink-install --packages-select tianrover rslidar_sdk --allow-overriding rslidar_sdk` 成功。
`colcon test --packages-select tianrover` 中配置契约与 launch_testing 集成检查均通过。
最终测试日志：NX `/tmp/tianrover-final-test3.log`；构建日志 `/tmp/tianrover-review-build.log`。

配置检查包含空、根、嵌套命名空间，非法名字校验，两个机器人隔离与外部硬件适配器发现。
独立 ROS_DOMAIN_ID=179、localhost 合成测试证明：

- 两台机器人分别将点云转换为 scan，产生各自 SLAM 地图并保存。
- 两套 Nav2 节点进入 active，map_server 继续使用各自带前缀的 map frame。
- 服务和 action 路径隔离，规划器分别返回对应 map frame 下的路径。
- `/tf`、`/tf_static` 共享；没有全局 scan、points、map、odom 或 cmd_vel 泄漏。
- 没有启动底盘或真实雷达，没有下发 NavigateToPose 运动目标。

## 火星车真实传感器静态检查

在独立 ROS_DOMAIN_ID=180、`TIANBOT_NAME=mars_check` 下启动新 bringup 和 SLAM：

- 底盘串口验证设备类型 `tianrover` 成功，里程计 frame 为 `mars_check/odom`，child 为 `mars_check/base_footprint`。
- RoboSense SDK 真正读取临时配置，点云发布到 `/mars_check/points`。
- scan frame 为 `mars_check/laser`，一次样本含 720 个方向、706 个有限距离。
- SLAM 地图 frame 为 `mars_check/map`，一次样本为 170×128、4070 个已知栅格。
- 地图保存到 NX `/tmp/tianrover-real-map/map.yaml` 和 `.pgm`，保存命令退出码为 0。
- 停止 SLAM 后启动 AMCL/Nav2并设置初始位姿，导航节点进入 active。
- ComputePathToPose 静态检查返回 `mars_check/map` 下的两点路径；未发送运动目标。

对应日志位于 NX `/tmp/tianrover-real-bringup2.log`、`/tmp/tianrover-real-slam2.log`、
`/tmp/tianrover-real-save.log`、`/tmp/tianrover-real-navigation.log`。
本次启动的测试进程已停止。静态测试地图没有设成默认生产地图。

## 验证边界

以上证明包可部署、数据链与命名隔离有效、地图可保存且导航规划服务可用。
没有进行真实行驶建图、定位精度、动态避障、目标到达或停车验收。
默认车体包络与速度限制是初始值，后续实测事项及三维建图保存在 TODO.md。
本机 ROS 1 代码只作为接口设计参考；本机历史 Humble 安装不作为 NX 导航的完整兼容性结论。

## 2026-10-08 模型修正验证

参考本机月球车的实际 Gazebo 模型与驱动，修正火星车 CAD 模型的左中轮转向原点、
右后轮悬架连接与活动关节坐标系；尺寸、网格、质量、惯性和零位传感器位置保持不变。
详见 [模型说明](MODEL.md)。

- 本机 `check_urdf` 成功；三项模型测试通过。
- 原始与修正模型的 visual/collision/inertial 世界变换最大误差约 5e-16。
- NX 构建成功，完整包测试结果为 19 tests、0 errors、0 failures、0 skipped。
- 安装后 description.launch.py 的带前缀 TF、六轮正负转向、滚动及两侧主摇臂运动通过真实 robot_state_publisher 检查。
- 测试仅发布隔离域中的合成 JointState，不连接串口、不发送运动命令；测试进程已停止。

该证据验证模型运动学和 TF 发布，不代表已接入真实关节反馈，也不证明火星车 Gazebo 动力学或实车运动完成验收。
