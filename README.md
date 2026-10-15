# TianRover 火星车使用说明

本仓库提供 TianRover 火星车的 ROS 2 设备启动、模型显示、建图和导航功能，各功能由 `tianrover_bringup`、`tianrover_description`、`tianrover_slam` 等 ROS 2 功能包提供。
当前部署环境为 **Ubuntu 22.04 / ROS 2 Humble / Jetson NX**，以下命令适用于已安装驱动和本功能包的车辆。

日常使用按下方步骤操作。首次安装、驱动适配和源码构建见[部署说明](docs/development/DEPLOYMENT.md)；更多功能见[文档目录](docs/README.md)。

## 1. 打开终端并检查环境

已部署火星车的新 Bash 终端会通过 `~/.bashrc` 自动加载 ROS 2 Humble 和
`~/tianrover_ws/install/setup.bash`，后者会串联加载底盘驱动工作区。
**日常打开终端后即可执行本说明中的命令，无需再次手动 source。**

这里 `~/tianbot_ws` 是驱动工作区，`~/tianrover_ws` 是车辆功能工作区。
其他电脑或重新部署的车辆需按实际安装位置配置环境。需要检查自动加载是否正常时，执行：

```bash
echo "$ROS_DISTRO"
ros2 pkg prefix tianrover_bringup
```

应分别显示 `humble` 和车辆工作区中的功能包路径。

<details>
<summary>环境未自动加载或在非交互脚本中运行时</summary>

仅在这些情况下显式加载环境，再执行对应命令：

```bash
source /opt/ros/humble/setup.bash
source ~/tianrover_ws/install/setup.bash
```

这不是日常每开一个终端都要重复的步骤。安装位置不同的电脑需替换实际工作区路径。

</details>

首次使用时检查机器人配置：

```bash
cat ~/.config/tianrover/my_tianrover.sh
```

如果文件不存在，可从已安装功能包复制示例：

```bash
mkdir -p ~/.config/tianrover
cp "$(ros2 pkg prefix --share tianrover_bringup)/config/my_tianrover.sh.example" \
  ~/.config/tianrover/my_tianrover.sh
```

按车辆配置修改该文件后，重新打开终端，再启动对应程序。无需修改源码或重新编译。
本文使用示例中的 `TIANBOT_NAME=mars`；如果名称不同，检查话题和坐标系时也要替换 `mars`。

## 2. 启动设备

终端 1：

```bash
ros2 launch tianrover_bringup bringup.launch.py
```

该命令启动底盘、雷达、二维激光扫描、车辆模型和 ZED2i 相机。启动后保留终端运行。
如果本次不使用相机：

```bash
ros2 launch tianrover_bringup bringup.launch.py use_zed:=false
```

两条命令选择一条。已有设备程序运行时，先确认是否已启动底盘、雷达或相机，避免重复启动。

只查看车辆模型时，可以不启动设备，在两个终端分别执行：

```bash
ros2 launch tianrover_description description.launch.py
```

```bash
ros2 launch tianrover_rviz rviz.launch.py fixed_frame:=mars/base_footprint
```

已运行 `bringup` 时，直接打开 RViz 即可，无需再运行 `description`。

## 键盘遥控

设备启动后，在另一个交互终端中运行（已部署车辆会自动加载环境）：

```bash
ros2 run tianrover_teleop teleop_key
```

默认沿用 `TIANBOT_NAME`，例如 `mars` 对应 `/mars/cmd_vel`。
键盘直接发布到底盘速度话题，使用前先取消导航并退出导航程序，避免与导航同时控制底盘。
也可显式指定：

```bash
ros2 run tianrover_teleop teleop_key --ros-args -r __ns:=/mars -p speed:=0.2 -p turn:=0.4
```

`u/i/o` 为前左/前/前右，`j/l` 为左右平移，`m/,/.` 为后左/后/后右，
`a/d` 为原地左/右转，`q/e` 为前进并左/右转，`z/c` 为后退并左/右转；
弧线键同时发布 `linear.x` 和 `angular.z`，`linear.y=0`。左转表示正角速度，
右转表示负角速度。`k` 或空格停车，`Ctrl+C` 退出。
全向按键无需 Shift；斜向速度按总线速度归一化。
默认线速度 0.2 m/s、角速度 0.4 rad/s；`t/b`、`w/x`、`r/v`
分别增减全部速度、线速度、角速度，上限分别为 0.5 m/s、1.0 rad/s。
无新方向键 0.5 秒后发布停车，持续行驶需按住方向键。
可通过 ROS 参数 `speed`、`turn`、`speed_limit`、`turn_limit`、
`key_timeout`、`repeat_rate` 调整。底盘是否执行横移取决于其运动模式和硬件能力。

## Gazebo 仿真

```bash
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ros2 launch tianrover_gazebo gazebo.launch.py namespace:=sim_mars
```

提供从 ROS 1 移植的 Mini 六轮车型与障碍场地，使用 ROS 2 Humble / Gazebo Classic 11。
仿真在开发电脑运行，使用独立 ROS 域和 `sim_mars` 命名空间；配套遥控终端使用相同设置。
当前提供底盘运动、里程计和关节反馈，不包含雷达/相机及完整建图导航链路。
启动参数、仿真控制接口及模型运动边界见[仿真说明](tianrover_gazebo/README.md)。

## 3. 二维建图

保持终端 1 的设备程序运行。打开终端 2 启动建图：

```bash
ros2 launch tianrover_slam slam.launch.py
```

此入口使用 SLAM Toolbox。终端 3 打开 RViz：

```bash
ros2 launch tianrover_rviz rviz.launch.py
```

使用随车遥控器低速行驶，让雷达逐步扫描需要建图的区域；在 RViz 中查看地图和激光扫描是否正常更新。
本说明不改变随车遥控器的操作方式。

如需使用 Gmapping 或 Cartographer，按[二维建图指南](docs/guides/SLAM_2D.md)操作。每次只运行一种建图程序。

## 4. 保存地图

车辆停车后，保持设备和建图程序运行。在新终端执行：

```bash
ros2 launch tianrover_slam map_save.launch.py slam_methods:=slam_toolbox \
  map_path:=$HOME/maps map_file:=site_01
```

确认命令成功，并检查地图文件：

```bash
ls -lh ~/maps/site_01.yaml ~/maps/site_01.pgm
```

地图由描述文件 `.yaml` 和图像文件 `.pgm` 配套组成，复制地图时请一并复制。
保存其他场地时，修改 `site_01` 为新的名称。

SLAM Toolbox 和 Gmapping 保存栅格地图时，空闲阈值默认为 `0.196`，可用
`TIANROVER_MAP_FREE_THRESH` 调整；它只影响保存时的像素分类，不改变在线建图。
Cartographer 使用独立保存流程，详见[二维建图指南](docs/guides/SLAM_2D.md)。

## 5. 使用地图导航

先在终端 2 按 `Ctrl+C` 停止建图。保留设备程序和 RViz，再在终端 2 执行：

```bash
ros2 launch tianrover_navigation navigation.launch.py map:=$HOME/maps/site_01.yaml
```

在 RViz 中依次操作：

1. 确认显示的是刚才保存的地图。
2. 使用 **2D Pose Estimate** 在地图上标出车辆当前位置，并拖动箭头指定朝向。
3. 检查激光扫描与地图中的墙面、障碍物是否对齐。
4. 使用目标工具 **2D Goal Pose**（部分界面显示为 **2D Nav Goal**）设置目标位置和朝向。
5. 观察车辆行驶及到点停车情况。

建图和导航要分开运行。示例配置的导航速度上限为 `0.20 m/s`、角速度上限为 `0.40 rad/s`；实际运行以车辆配置文件为准。
首次在新场地使用时，请在留有停车空间的区域测试定位与到点行为。

日后直接使用已有地图，可以跳过建图和保存步骤：打开终端、启动设备、打开 RViz，再启动导航并设置初始位姿。

### 使用三维地图导航

正常退出三维建图，保留对应的三维整车驱动。以下以 FAST-LIO2 为例，
将数据库路径替换为保存后的地图：

```bash
export TIANROVER_MAX_SPEED=0.10
ros2 launch tianrover_navigation localize_and_navigate_3d.launch.py frontend:=fastlio \
  use_rgbd:=true database:=$HOME/maps/mars3d_v1_saved.db
```

另开图形终端：

```bash
ros2 launch tianrover_rviz rviz.launch.py fixed_frame:=mars/map
```

使用 **2D Pose Estimate** 设置车辆位置与朝向，确认实时点云与地图对齐后，
使用 **2D Goal Pose** 设置地面目标。未启用相机时使用 `use_rgbd:=false`；
KISS-ICP 使用 `frontend:=kiss` 和对应地图。此入口同时启动 RTAB-Map 定位和 Nav2，无需另开 AMCL。
驱动启动、地图准备与完整步骤见[自主导航](https://docs.tianbot.com/tianrover/guide-mars/navigation#navigation-3d)。

## 6. 停止使用

先让车辆停车，使用随车停止功能确认底盘停止，再在导航或建图终端按 `Ctrl+C` 退出。
随后退出 RViz 和设备程序。车辆停止状态应以实际底盘表现为准。

## 常用配置

配置文件为 `~/.config/tianrover/my_tianrover.sh`。修改后，重新打开终端并启动对应程序。

| 配置项 | 用途 | 示例值 |
| --- | --- | --- |
| `TIANBOT_NAME` | 机器人名称，决定话题及坐标系前缀 | `mars` |
| `TIANROVER_BASE_PORT` | 底盘串口 | `/dev/tianbot_base` |
| `TIANROVER_USE_ZED` | 是否启动相机 | `true` 或 `false` |
| `TIANROVER_MAP` | 不指定 `map:=` 时使用的地图路径 | `$HOME/maps/site_01.yaml` |
| `TIANROVER_MAX_SPEED` | 导航线速度上限，单位 m/s | `0.20` |
| `TIANROVER_MAX_YAW_RATE` | 导航角速度上限，单位 rad/s | `0.40` |
| `TIANROVER_SCAN_MIN_HEIGHT` / `TIANROVER_SCAN_MAX_HEIGHT` | 雷达坐标系中的二维投影高度带，单位 m | `-0.35` / `0.35` |
| `TIANROVER_SCAN_RANGE` | 二维扫描距离上限，单位 m | `20.0` |
| `TIANROVER_MAP_FREE_THRESH` | SLAM Toolbox / Gmapping 保存地图的空闲阈值 | `0.196` |

配置文件使用 `export 配置项=值` 的格式，例如：

```bash
export TIANBOT_NAME=mars
export TIANROVER_USE_ZED=false
export TIANROVER_MAP=$HOME/maps/site_01.yaml
```

已有车辆升级源码时会保留外部配置，新的默认值不会覆盖它。若原文件仍设置投影高度带为
`-0.10` / `0.10`，需按本车雷达安装情况调整；Helios 16 近水平束线在该高度带内
约到 5.7 m 就会被过滤。当前默认 ±0.35 m 用于覆盖约 20 m 的束线高度偏移，
不代表已完成 20 m 实测验收。详见[部署说明](docs/development/DEPLOYMENT.md)。
修改后先退出对应程序，再从新终端启动；仅修改文件不会更新已有终端或已运行节点。

## 常见问题

| 现象 | 检查方法 |
| --- | --- |
| 提示找不到 `tianrover` | 执行 `echo "$ROS_DISTRO"` 和 `ros2 pkg prefix tianrover_bringup`；若自动加载失效，按上面的排错说明手动加载环境。 |
| 提示地图不存在 | 检查 `map:=` 指定的 YAML 路径，以及与它配套的地图图像是否存在。 |
| RViz 有模型但没有地图 | 确认建图或导航程序正在运行；建图和导航时 Fixed Frame 应为 `mars/map`。 |
| 尚未建图，只想查看设备 | 使用 `ros2 launch tianrover_rviz rviz.launch.py fixed_frame:=mars/odom`。 |
| 模型在其他电脑显示不完整 | RViz 所在电脑也需要安装本功能包和包含模型网格的 `tianrover_description`。 |
| 没有扫描或里程计 | 在新终端分别执行 `ros2 topic hz /mars/scan`、`ros2 topic hz /mars/odom`；检查设备终端的报错。 |
| 相机启动失败 | 检查相机连接和驱动；仅使用二维雷达建图时，可用 `use_zed:=false` 启动设备。 |
| 导航时扫描与地图不对齐 | 重新设置初始位置和朝向；仍不对齐时，停车并记录定位程序日志。 |

## 其他功能

- [Gmapping / Cartographer 二维建图](docs/guides/SLAM_2D.md)：其他建图算法及地图保存方式。
- [车辆模型与关节显示](docs/guides/MODEL.md)：模型查看和真实关节反馈接入说明。默认关节显示为零位，不代表电机反馈。
- [轮式里程计与 IMU 融合](docs/guides/EKF.md)：可选里程计融合，启用前需检查传感器方向和数据。
- [三维建图与地图保存](docs/guides/THREED.md)、[FAST-LIO2](docs/guides/FASTLIO.md)：三维前端、地图保存和定位导航使用说明。
