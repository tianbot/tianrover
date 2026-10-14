# MarsRover 部署与硬件适配参考

本文供部署和研发维护人员查阅，保留原部署方案及现场问题记录。日常操作见[客户使用说明](../../README.md)，文档分类见[文档目录](../README.md)。

> 2026-10-09：支持可选 FAST-LIO2（H16 + ZED 或底盘 IMU）前端，保留 KISS。现场已完成三维建图和离线数据库保存；三维定位导航仍有输入掉帧与 TF 时效问题，未完成运动交付验收。使用入口和待标定项见 [FASTLIO.md](../guides/FASTLIO.md)，后续 FAST-LIO2 + GICP 定位方案见 [TODO.md](TODO.md)。

ROS2-only：当前验证目标为 NX 的 Ubuntu 22.04 / ROS 2 Humble / ARM64。
独立功能包仓库：`tianbot/tianrover`。本次发布保留 NX 的实际源码与已有 Git 历史，推送在维护者本机完成，客户设备不需要维护者 GitHub 凭据。原 ROS 1 Tianbot 仓库的 `ros2/` 目录是历史开发位置。
六轮六转 MarsRover 的底层接受 `Twist`，本包使用 NavFn + DWB；二维定位采用 AMCL DifferentialMotionModel，三维定位采用 RTAB-Map。
不复制底盘协议实现；复用现场 ROS 2 `tianbot_core`、`rslidar_sdk` 驱动。
Mars URDF 来源于 192.168.1.42 当前部署模型，当前模型与网格由 `tianrover_description` 统一提供。
2026-10-08 已修正转向原点、悬架父子关系和关节坐标系；见 [模型说明](../guides/MODEL.md)。

三维入口支持 H16 XYZIRT、KISS-ICP 或 FAST-LIO2、RTAB-Map 与 Nav2；KISS 不依赖 IMU，FAST-LIO2 使用选定 IMU。
录包、六轮轮径/转向零偏求解、独立验证、应用/回滚和动作预演的命令见 [三维链路](../guides/THREED.md)。
真实主动关节反馈目前缺失，不能执行实车标定；二维入口仍可独立使用。

二维 Gmapping、Cartographer 的交付入口、保存地图与导航步骤见 [二维建图](../guides/SLAM_2D.md)。

## 构建

独立工作区构建，保留原工作区作为驱动和网格资源 underlay：

```bash
source /opt/ros/humble/setup.bash
source ~/tianbot_ws/install/setup.bash
mkdir -p ~/tianrover_ws/src
# 将本目录复制或链接为 ~/tianrover_ws/src/tianrover
cd ~/tianrover_ws
colcon build --symlink-install --packages-up-to tianrover
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
colcon build --symlink-install --packages-up-to rslidar_sdk tianrover \
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
cp src/tianrover/tianrover_bringup/config/my_tianrover.sh.example ~/.config/tianrover/my_tianrover.sh
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
ros2 launch tianrover_bringup bringup.launch.py
ros2 launch tianrover_slam slam.launch.py
ros2 launch tianrover_rviz rviz.launch.py
```

`bringup` 包含 description、base、lidar、scan、ZED2i。相机为 HD720/15 Hz，深度 PERFORMANCE，点云 5 Hz；关闭相机定位 TF，避免与底盘/SLAM 冲突。
单独启动：`ros2 launch tianrover_bringup zed.launch.py`；禁用：`ros2 launch tianrover_bringup bringup.launch.py use_zed:=false`。
需要安装 `zed_wrapper` 和对应 ZED SDK；外部配置可设置 `TIANROVER_USE_ZED=false` 或 `TIANROVER_ZED_PARAMS=/path/to/config.yaml`。
NX 当前 ZED SDK 的图像有效性检查会阻塞采集（调用栈停在 `CorruptedFrameDetectorImpl::waitForResults`）；配置暂设 `general.enable_image_validity_check: 0`，因此不提供 SDK 坏帧检测。关闭后已收到 RGB、深度和 IMU 数据；升级 SDK 后再评估恢复。
相机话题前缀为 `/mars/zed2i`；其光学 TF 由官方相机 URDF 提供，通过 `mars/zed_camera_link -> zed2i_camera_link` 静态变换接入车辆 TF，默认复用现有 URDF 安装位姿，连接偏移为零，尚未做实物标定。可用 `TIANROVER_ZED_MOUNT_X/Y/Z/ROLL/PITCH/YAW` 配置相对安装帧的偏移（米/弧度）。RViz 使用 `mars/map`、`mars/odom` 或 `mars/base_link` 时可显示相机彩色点云。
也可单独启动同名入口，避免重复驱动。
雷达的三维点云先转换为二维 scan；高度带默认是 laser frame 中的 [-0.10, 0.10]m。
这属于二维建图的输入适配，三维建图见 [三维链路](../guides/THREED.md)。

保存地图，然后停止 SLAM，再启动 AMCL 导航（两者不能同时发布 map -> odom）：

```bash
ros2 launch tianrover_slam map_save.launch.py
ros2 launch tianrover_navigation navigation.launch.py
```

地图默认位于 `~/.local/share/tianrover/maps/<namespace>/map.yaml`，不写源码树或 install 目录。
也可设置 `TIANROVER_MAP` 或 `map:=/absolute/map.yaml`；保存支持 `map_name:=/absolute/map`。
RViz 的初始位姿和目标话题会跟随同一 namespace。

### 在 RViz 看模型

配套 RViz 默认启用 RobotModel，从对应 namespace 的 `robot_description` 话题加载 URDF，
TF Prefix 与机器人名称一致。仅查看模型时，两个终端分别运行：

```bash
ros2 launch tianrover_description description.launch.py
ros2 launch tianrover_rviz rviz.launch.py fixed_frame:=mars/base_footprint
```

`mars` 替换为当前 `TIANBOT_NAME`。导航时保留默认 map 固定坐标系并确保 AMCL 已初始化。
已有 bringup 包含 description，无需重复启动。RViz 的 Views 可选择 Orbit 以旋转查看三维模型。
在其他电脑运行 RViz 时，该电脑也需要 `tianrover_description` 包及对应 STL；TF 话题不会传输网格文件。

### 实车关节运动与整车位姿

默认 description 中的 `joint_state_publisher` 只发布零位用于展示，不代表电机反馈。
实车左右各两个 rocker-bogie 被动关节没有编码器，保持中立展示姿态；
Gazebo 提供的关节状态属于仿真数据，不能用于证明实车有电机位置上传接口。
真实驱动接入同一 namespace 的 `sensor_msgs/JointState` 后，使用：

```bash
ros2 launch tianrover_bringup bringup.launch.py use_joint_state_publisher:=false
```

也可对独立 `description.launch.py` 使用同一参数。该参数只关闭展示零位，
不会生成电机反馈；没有真实数据时，可动关节 TF 不会更新。不要同时运行独立 description 和 bringup，
否则会重复启动模型及关节发布节点。接入要求与现场检查见 [MODEL.md](../guides/MODEL.md#实车反馈接入)。
若硬件只反馈主动关节，关闭默认发布者前需单独补齐四个被动关节的中立 TF。

观看整车位移时，确保底盘发布有效的 `odom → base_footprint` TF，并将 RViz Fixed Frame
设置为 `<namespace>/odom`；导航使用 `<namespace>/map`。以车体自身作为 Fixed Frame 时整车位置保持不变。

### 轮式里程计与 IMU 的二维 EKF

使用 ROS 2 `robot_localization/ekf_node`，参考本机 TianRacer 的平面速度融合；
IMU 已有固件姿态四元数，使用相对航向以消除启动航向偏差，且唯一姿态源的
`imu0_differential` 为 false。不融合底盘累计位置/航向、IMU 加速度及重复的陀螺积分。
已有 bringup 时可先启动并行检查：

```bash
ros2 launch tianrover_localization ekf.launch.py
ros2 topic echo /mars/odometry/filtered --once
```

这时输入为 `/mars/odom`、`/mars/imu`，输出为 `/mars/odometry/filtered`，不发布 TF，
不改变原底盘或导航。并行输出的 frame 为 `mars/odom_ekf`；融合坐标原点及相对航向在 EKF
启动时建立，不能直接叠加到已有 odom TF 上。
确认输入方向、帧和移动表现后，停止并行 EKF 与旧 bringup，再运行：

```bash
ros2 launch tianrover_bringup bringup.launch.py use_ekf:=true
```

此模式底盘发布 `/mars/odom_raw` 并关闭底盘 TF；EKF 发布 `/mars/odom` 及
`mars/odom → mars/base_footprint`，现有 SLAM/Nav2 继续使用原话题和帧。
也可在外部配置设置 `TIANROVER_USE_EKF=true`；回退为 false 并重新启动 bringup 即恢复原驱动里程计。
切换/重启会建立新的融合原点，应在车辆静止且导航停止时操作，并重新初始化定位。
自定义底盘适配器也必须实现相同的原始话题和唯一 TF 发布者契约。

`ekf_inputs` 只为全零协方差设置明确的可调初值，保留已有有效协方差、时间戳和原始话题，
拒绝无效四元数/缺失姿态；不会凭控制指令制造数据。默认速度方差和航向方差均为 0.0025，
单位分别为 `(m/s)^2` 和 `rad^2`，它们是调试初值，不是完成标定的测量精度。
实际精度需以已知距离、转角或独立定位真值比较；滤波启动成功不代表精度提高。
详见 [EKF.md](../guides/EKF.md)。

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
colcon test --base-paths src/tianrover
colcon test-result --verbose
python3 src/tianrover/tianrover_tests/test/runtime_check.py
```

集成检查使用独立 ROS_DOMAIN_ID、合成点云和两个 namespace，验证 scan、地图保存、Nav2 lifecycle、
服务、action 与共享 TF 前缀，不连接串口、不发送车辆速度。
合成检查不能替代实际行驶建图、定位、避障、到点和停车验收。

回退：停止本包启动的终端；在新的终端仅 source 原 `~/tianbot_ws/install/setup.bash`，
继续使用原入口。本包不覆盖原驱动、launch 或 shell 配置。
