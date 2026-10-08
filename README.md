# TianRover ROS 2 二维建图导航

ROS2-only：当前验证目标为 NX 的 Ubuntu 22.04 / ROS 2 Humble / ARM64。
本包位于 ROS 1 Tianbot 仓库的 `ros2/` 内，`CATKIN_IGNORE` 避免旧 catkin 工作区将它作为 ROS 1 包编译。
六轮六转 MarsRover 的底层接受 `Twist`，本包按差速机器人使用 NavFn + DWB 和 AMCL DifferentialMotionModel。
不复制底盘协议实现；复用现场 ROS 2 `tianbot_core`、`rslidar_sdk` 驱动。
Mars URDF 来源于 192.168.1.42 当前部署模型，其网格仍由 `tianbot_description` 提供。
2026-10-08 已修正转向原点、悬架父子关系和关节坐标系；见 [模型说明](MODEL.md)。

## 构建

独立工作区构建，保留原工作区作为驱动和网格资源 underlay：

```bash
source /opt/ros/humble/setup.bash
source ~/tianbot_ws/install/setup.bash
mkdir -p ~/tianrover_ws/src
# 将本目录复制或链接为 ~/tianrover_ws/src/tianrover
cd ~/tianrover_ws
colcon build --symlink-install --packages-select tianrover
source install/setup.bash
```

基础系统依赖：

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-pointcloud-to-laserscan
```

NX 原 RoboSense SDK（`ea5c55ada263f8e0cfb75e438424d1a8a9dcbe1d`）在 ROS 2 下忽略
`config_path`，只读取编译时源码目录的 config。新工作区使用原驱动源码快照的独立副本，
加上 `patches/rslidar-config-path.patch`，并保留已有 ROS 2 编译适配
`patches/rslidar-ros2-build.patch`。原工作区没有修改。
如果重新部署这版驱动，将源码复制到新工作区，应用配置路径补丁后执行：

```bash
cd ~/tianrover_ws/src/rslidar_sdk
patch -p1 < ../tianrover/patches/rslidar-config-path.patch
cd ~/tianrover_ws
colcon build --symlink-install --packages-select rslidar_sdk tianrover \
  --allow-overriding rslidar_sdk
source install/setup.bash
ros2 pkg prefix rslidar_sdk  # 应指向 tianrover_ws/install
```

对未适配的原始 SDK，另需应用 ROS 2 编译补丁并准备其 `rs_driver` 源码依赖。
已打过补丁的部署副本不要重复应用补丁。

可初始化 rosdep 后使用 `rosdep install --from-paths src --ignore-src --rosdistro humble`。
现场驱动是源码 underlay 包；rosdep 并不提供定制底盘驱动或设备配置。

## 部署配置

```bash
mkdir -p ~/.config/tianrover
cp src/tianrover/config/my_tianrover.sh.example ~/.config/tianrover/my_tianrover.sh
# 编辑这个外部文件，再重新 source；无需重新编译。
source ~/tianrover_ws/install/setup.bash
```

`TIANROVER_CONFIG_FILE` 可指定其他 shell 配置文件。环境 hook 在每次 source 时读取它。
launch 从环境中读取配置；本机文件只负责 export，Tianbot Studio 今后可以维护同一配置契约。
不需要逐车修改版本库内的默认 env-hook。若要临时覆盖配置，在 source 之后 export。

所有入口共享 `TIANBOT_NAME`，空值或 `/` 表示单机根命名空间；例如 `mars_a`、`mars_b`。
节点、话题、参数、服务和 action 均位于对应 namespace。
TF transport 保持共享 `/tf`、`/tf_static`，frame 使用 `mars_a/map`、`mars_a/odom` 等前缀。
不要将 TF topic 再移入机器人 namespace。两台雷达的 UDP 端口、网络配置仍需分别规划。
如果接入全局 `world`，由部署方根据真实初始位姿发布 `world -> mars_a/map`，本包不会虚构位姿。

## 固定入口

每个新终端都执行：

```bash
source ~/tianrover_ws/install/setup.bash
```

分别启动：

```bash
ros2 launch tianrover bringup.launch.py
ros2 launch tianrover slam.launch.py
ros2 launch tianrover rviz.launch.py
```

`bringup` 包含 description、base、lidar、scan。也可单独启动同名入口，避免重复驱动。
雷达的三维点云先转换为二维 scan；高度带默认是 laser frame 中的 [-0.10, 0.10]m。
这属于二维建图的输入适配，三维建图见 TODO。

保存地图，然后停止 SLAM，再启动 AMCL 导航（两者不能同时发布 map -> odom）：

```bash
ros2 launch tianrover map_save.launch.py
ros2 launch tianrover navigation.launch.py
```

地图默认位于 `~/.local/share/tianrover/maps/<namespace>/map.yaml`，不写源码树或 install 目录。
也可设置 `TIANROVER_MAP` 或 `map:=/absolute/map.yaml`；保存支持 `map_name:=/absolute/map`。
RViz 的初始位姿和目标话题会跟随同一 namespace。

### 在 RViz 看模型

配套 RViz 默认启用 RobotModel，从对应 namespace 的 `robot_description` 话题加载 URDF，
TF Prefix 与机器人名称一致。仅查看模型时，两个终端分别运行：

```bash
ros2 launch tianrover description.launch.py
ros2 launch tianrover rviz.launch.py fixed_frame:=mars/base_footprint
```

`mars` 替换为当前 `TIANBOT_NAME`。导航时保留默认 map 固定坐标系并确保 AMCL 已初始化。
已有 bringup 包含 description，无需重复启动。RViz 的 Views 可选择 Orbit 以旋转查看三维模型。
在其他电脑运行 RViz 时，该电脑也需要 `tianbot_description` 包及对应 STL；TF 话题不会传输网格文件。

## 增加硬件而不增加 launch 分叉

`TIANROVER_BASE`、`TIANROVER_LIDAR`、`TIANROVER_DESCRIPTION` 选择组件名，`none` 表示关闭。
默认分别是 `tianbot_core`、`rslidar`、`mars`。
`TIANROVER_COMPONENT_DIR` 下的 `base/<name>.launch.py`、`lidar/<name>.launch.py`、
`description/<name>.launch.py` 优先于内置组件。新增设备仅增加适配器及其配置，固定入口不变。
适配器必须遵守 `Config.namespace`、`Config.frame()`、`Config.topic()` 的契约。
驱动如果硬编码全局话题或 frame，需在适配器内修正，不能只改 Node.namespace。

硬件专用变量见示例文件；复杂驱动参数通过 `TIANROVER_LIDAR_CONFIG` YAML 修改，
导航和 SLAM 通过 `TIANROVER_NAV_PARAMS`、`TIANROVER_SLAM_PARAMS` 修改，
模型通过 `TIANROVER_URDF` 修改。通用入口不需要为每个驱动参数增加命令行参数。

RoboSense 使用 UDP 接收，`TIANROVER_LIDAR_HOST` 指 NX 的接收接口地址，默认监听全部接口；
MSOP/DIFOP 接收端口也可通过环境配置。雷达设备自身的目标 IP/端口需要在设备上配置。
现场旧配置中的 `device_ip` 并不被这版 SDK 读取，本包使用其真正支持的 `host_address`。

速度默认 0.20 m/s、角速度 0.40 rad/s，默认外接圆半径 0.65m；
这是初始配置，必须按实际车体包络和现场刹车能力验收，不能当作已经标定。
`TIANROVER_SCAN_MIN_HEIGHT/MAX_HEIGHT` 设置过滤带，`TIANROVER_POINTS_TOPIC` 可接入已有点云。
实车所有入口默认 `use_sim_time=false`；仿真可显式传 `use_sim_time:=true`。

## 验证与回退

```bash
colcon test --packages-select tianrover
colcon test-result --verbose
python3 src/tianrover/test/runtime_check.py
```

集成检查使用独立 ROS_DOMAIN_ID、合成点云和两个 namespace，验证 scan、地图保存、Nav2 lifecycle、
服务、action 与共享 TF 前缀，不连接串口、不发送车辆速度。
合成检查不能替代实际行驶建图、定位、避障、到点和停车验收。

回退：停止本包启动的终端；在新的终端仅 source 原 `~/tianbot_ws/install/setup.bash`，
继续使用原入口。本包不覆盖原驱动、launch 或 shell 配置。
