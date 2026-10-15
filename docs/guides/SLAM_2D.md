# 二维 Gmapping / Cartographer 交付入口

ROS2-only，部署目标为 NX Ubuntu 22.04 / ROS 2 Humble / ARM64。
沿用本机 Tianbot 的 scan + odom + TF 输入及建图后保存、再 AMCL 导航的流程。
原 `slam.launch.py` 仍是 SLAM Toolbox；不要把算法名混用。

## 使用

每个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/tianbot_ws/install/setup.bash
source ~/tianrover_ws/install/setup.bash
export TIANBOT_NAME=mars
```

停止之前的建图/定位节点再选择一种算法，同一车不能同时由多个节点发布 map→odom。
已有 bringup 时不重复开启底盘/雷达；独立开启二维硬件入口使用：

```bash
ros2 launch tianrover_bringup bringup.launch.py
```

默认采用底盘 wheel odom；已运行 bringup_3d 时也可复用其 KISS odom/TF，
但必须先停止 mapping_3d 或三维定位，不能重复开启底盘或 TF。

建图（二选一）：

```bash
ros2 launch tianrover_slam tianrover_gmapping.launch.py
ros2 launch tianrover_slam tianrover_cartographer.launch.py
```

同义短入口为 gmapping.launch.py、cartographer.launch.py。
统一入口为 `tianrover_slam.launch.py slam_methods:=gmapping` 或 `cartographer`。

另开 RViz：

```bash
ros2 launch tianrover_rviz rviz.launch.py
```

低速遥控走一圈。检查 /mars/map、/mars/scan 和 mars/map→mars/odom TF。
建图时停车保存，保持建图节点运行；输出目录在源码之外：

```bash
# Gmapping：PGM + YAML
ros2 launch tianrover_slam map_save.launch.py slam_methods:=gmapping \
  map_path:=$HOME/maps map_file:=delivery_gmapping
# Cartographer：PGM + YAML + pbstream；仍可继续建图
ros2 launch tianrover_slam map_save.launch.py slam_methods:=cartographer \
  map_path:=$HOME/maps map_file:=delivery_cartographer
```

旧 `map_name:=$HOME/maps/name` 完整前缀写法仍可用。Cartographer 拒绝覆盖已有输出。
Gmapping 和 SLAM Toolbox 的栅格保存默认使用空闲阈值 `0.196`；可在启动保存命令前
通过 `export TIANROVER_MAP_FREE_THRESH=0.196` 调整（取值应在 0 与占用阈值之间）。
该配置控制保存时的空闲像素分类，不改变在线建图参数，也不影响 Cartographer 的保存流程。
Cartographer 保存 include_unfinished_submaps=true，不结束活动轨迹；保存过程中保持停车。
若栅格保存失败但 pbstream 已保存，错误会提示；换新文件名重试，不把部分输出当作完成。

保存后停止建图，用同一套 Nav2 + AMCL 导航，两个算法均可：

```bash
ros2 launch tianrover_navigation navigation.launch.py map:=$HOME/maps/delivery_gmapping.yaml
# 或 map:=$HOME/maps/delivery_cartographer.yaml
```

RViz 用 2D Pose Estimate 设置初始位置，再用 2D Goal Pose 测试目标。
Cartographer 原生定位另提供：

```bash
ros2 launch tianrover_slam cartographer_localization.launch.py \
  state:=$HOME/maps/delivery_cartographer.pbstream
```

原生定位与 AMCL 不能同时运行。该入口提供 map 与 map→odom，可接无 AMCL 的导航入口，
交付的默认导航流程仍采用上面的 navigation.launch.py，避免两套定位争抢 TF。
Cartographer 配置关闭 IMU，使用二维 scan、已有 odom，provide_odom_frame=false，
published_frame=odom；不会再发布 odom→base_footprint。

## 依赖与构建

```bash
sudo apt install ros-humble-cartographer-ros
cd ~/tianrover_ws
vcs import src < src/tianrover/dependencies/dependencies-2d.repos
source /opt/ros/humble/setup.bash
source ~/tianbot_ws/install/setup.bash
patch --dry-run -p1 -d src/pointcloud_to_laserscan_vendor < src/tianrover/patches/pointcloud-reliable-cloud.patch
patch -p1 -d src/pointcloud_to_laserscan_vendor < src/tianrover/patches/pointcloud-reliable-cloud.patch
colcon build --packages-up-to pointcloud_to_laserscan openslam_gmapping slam_gmapping tianrover \
  --allow-overriding pointcloud_to_laserscan --symlink-install --parallel-workers 1 --cmake-force-configure
source install/setup.bash
```

已有 gmapping_vendor 时不要重复覆盖。Gmapping 使用锁定的 Adlink ROS 2 移植源码，
包含 openslam_gmapping 核心；保留上游许可声明。配置来自本机 Tianbot，
将 particles/lskip 改为 ROS 2 要求的整数类型，并匹配 namespace、scan 与帧名。
`TIANROVER_GMAPPING_PARAMS` 可指定 YAML；`TIANROVER_CARTOGRAPHER_CONFIG` 可指定 Lua 模板。
Lua 中 @MAP_FRAME@ 等占位符由入口替换，配置保留外部 odom TF。

构建/隔离域软件检查与实车地图质量验收分别记录；实车运动由操作者执行。

## 已执行的软件检查（2026-10-09）

NX 的 Cartographer Debian 包已安装，openslam_gmapping/slam_gmapping 与 TianRover 构建通过。
实际节点在隔离 ROS_DOMAIN_ID=189、ROS_LOCALHOST_ONLY=1 中检查通过：
两套 pointcloud→scan→建图均产生有已知栅格的地图，帧与 namespace 隔离；
map_save.launch.py 的两套保存入口执行成功，Cartographer 额外保存非空 pbstream；
保存状态重新加载后存在 FROZEN 轨迹；两套保存地图交给 AMCL/Nav2 后节点激活，
ComputePathToPose 均返回各自 map frame 的路径。未发送 NavigateToPose 或底盘指令。

日志 NX `/tmp/tianrover-slam2d-runtime.log`，产物
`/tmp/tianrover_slam2d__r9hs2sq/`。复查脚本：
`python3 src/tianrover/tianrover_tests/test/slam_2d_runtime_check.py`；已注册 launch_testing。
实车行驶建图质量、回环效果、定位精度和目标跟踪由交付现场验收，不能由合成数据证明。

## Cartographer 地图重影排查（2026-10-09）

现场只有一套 Cartographer/map 发布者，没有 AMCL 或 RTAB-Map 争抢地图。
当前输入是底盘 odom（50 Hz）及 H16 点云投影扫描。8 秒观测中原始点云
10.0025 Hz、最大间隔 0.100944 秒，但扫描只有 6.8654 Hz、最大间隔 0.499232 秒。
已对官方 Humble pointcloud_to_laserscan 增加可配置 reliable_cloud，
本包扫描入口默认 true，匹配 H16 SDK 的可靠发布者。补丁保存在 patches 中，
源码锁定在 dependencies/dependencies-2d.repos，系统二进制保持不改。
使用尽力发布的其他点云源时设置 TIANROVER_RELIABLE_CLOUD=false 并重新检查 QoS。
已应用补丁的源码不要重复打补丁。

投影 LaserScan 的 time_increment 为零，不能通过虚构 time_increment 补偿运动；
运动去畸变、轮式里程计转角/尺度和物理外参仍需行驶数据判断。
本次修复输入丢帧，未清空已有地图，也未盲改扫描匹配权重。
受损子地图不能保证仅靠更新参数恢复，建议保存旧图后用修复输入重新建图。

可靠输入修复后再次实测：扫描按测量时间为 10.0001 Hz，最大间隔 0.101021 秒，
消息年龄 P95 为 0.029353 秒；证据 NX /tmp/tianrover-scan-reliable-audit.json。
隔离回归通过，日志 /tmp/tianrover-scan-qos-regression.log。
现场独立扫描 launch PID=165097。若重启整个 bringup，应先停止它，避免重复发布扫描。
