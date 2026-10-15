# TianRover Mini ROS 2 仿真

本包从本地 ROS 1 `tianrover_gazebo` 移植，与实车功能同仓维护，在开发电脑单独构建，
不作为实车 `tianrover` 元包的依赖。
目标环境与当前车辆功能仓库一致：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。
保留 Mini 车型网格、障碍场地、六轮驱动、四轮转向、摇臂差动、Ackermann
接口和可选轮胎滑移诊断。Mini 模型资源归本包所有，与实车
`tianrover_description/urdf/mars.urdf` 分开。

## 构建和运行

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_tianbot_ws
rosdep install --from-paths src/tianrover --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-up-to tianrover_gazebo
source install/setup.bash
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ros2 launch tianrover_gazebo gazebo.launch.py namespace:=sim_mars
```

无图形界面时加 `gui:=false`。默认加载随包提供的障碍场地；可用
`world:=/absolute/path/to/world` 指定其他场地。初始位置参数为
`spawn_x:=1 spawn_y:=4 spawn_z:=0.05 spawn_yaw:=0`。
`pause:=true` 暂停启动，`verbose:=true` 打开 Gazebo 调试输出。
`tianrover_mini.launch.py` 是同一入口的兼容名称。

仿真在开发电脑运行，使用独立 ROS 域和 `sim_mars` 命名空间，避免与实车共用速度或 TF 话题。
当前模型没有雷达/相机数据源，不包含完整建图导航链路。

在另一个已加载开发工作区环境的交互终端中执行：

```bash
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ros2 run tianrover_teleop teleop_key --ros-args -r __ns:=/sim_mars
```

遥控器发布全向 Twist；此六轮 Mini 模型的运动控制沿用 ROS 1 的纵向速度和
偏航控制，侧向速度不参与驱动。`rover_motion_mode` 接收 `ackermann`、`rotate`、
`omni` 名称，后两种名称与 ROS 1 一样不切换为独立全向物理模型。
在此模型中使用 `i` / `,` 前后行驶、`a` / `d` 转动、空格停车。

## ROS 接口

以上述 `namespace:=sim_mars` 为例：

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `/sim_mars/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x` 纵向速度、`angular.z` 偏航速度 |
| `/sim_mars/ackermann_cmd` | `ackermann_msgs/msg/AckermannDrive` | 速度和转向角 |
| `/sim_mars/rover_motion_mode` | `std_msgs/msg/String` | 兼容 ROS 1 的模式名称接口 |
| `/sim_mars/odom` | `nav_msgs/msg/Odometry` | 仿真真值里程计，速度在车体坐标系表达 |
| `/sim_mars/abs_path` | `nav_msgs/msg/Path` | 最近 2000 个仿真位置 |
| `/sim_mars/joint_states` | `sensor_msgs/msg/JointState` | 仿真关节反馈 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时钟 |

TF 为 `sim_mars/odom -> sim_mars/base_footprint -> ...`，状态发布器使用仿真时间。
`namespace` 默认沿用 `TIANBOT_NAME`，设置 `namespace:=''` 可使用根命名空间。
驱动在最后一条运动命令超过 0.5 秒后停车。
可用 `enable_skid_rate:=true` 启用 `SkidRate`、各轮 `W*_VelPose`
和 `WheelPoseInBody` 诊断话题。

## 迁移边界与回退

这是 ROS1 → ROS2 的原生接口移植，无 `ros1_bridge` 依赖。
插件节点、发布订阅、TF、参数和 launch 均已换为 ROS 2；不加载 ROS 1
`gazebo_ros_control`，也不从 Gazebo 插件内部读取终端或手柄。
插件参数来自模型 SDF；`max_steering_angle` 为驱动 ROS 2 节点参数，默认 π/4。

迁移参考：[gazebo_ros_pkgs ROS 2 迁移指南](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Plugins)。
Gazebo Classic 是旧版仿真栈，本次不包含 Gazebo Harmonic / Jazzy 插件移植。
原 ROS 1 源包保持在原工作区，可停止 ROS 2 仿真后在独立终端加载 Noetic
环境启动原包回退；不要在同一终端混合加载两个 ROS 版本。
ROS 1 Noetic 已于 2025-05-31 结束官方支持，仅作为历史回退使用。
