# TianRover 火星车使用说明

本仓库提供 TianRover 火星车的 ROS 2 设备启动、模型显示、建图和导航功能，各功能由 `tianrover_bringup`、`tianrover_description`、`tianrover_slam` 等 ROS 2 功能包提供。
当前部署环境为 **Ubuntu 22.04 / ROS 2 Humble / Jetson NX**，以下命令适用于已安装驱动和本功能包的车辆。

日常使用按下方步骤操作。首次安装、驱动适配和源码构建见[部署说明](docs/development/DEPLOYMENT.md)；更多功能见[文档目录](docs/README.md)。

## 1. 加载运行环境

在火星车上打开终端。**每个新终端都先执行：**

```bash
source /opt/ros/humble/setup.bash
source ~/tianbot_ws/install/setup.bash
source ~/tianrover_ws/install/setup.bash
```

这里 `~/tianbot_ws` 是驱动工作区，`~/tianrover_ws` 是车辆功能工作区。若交付时使用其他路径，请替换为实际安装位置。

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

按车辆配置修改该文件，然后重新执行上面的环境加载命令。无需修改源码或重新编译。
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

## 3. 二维建图

保持终端 1 的设备程序运行。在加载环境后，终端 2 启动建图：

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

车辆停车后，保持设备和建图程序运行。在新终端加载环境，执行：

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

日后直接使用已有地图，可以跳过建图和保存步骤：加载环境、启动设备、打开 RViz，再启动导航并设置初始位姿。

## 6. 停止使用

先让车辆停车，使用随车停止功能确认底盘停止，再在导航或建图终端按 `Ctrl+C` 退出。
随后退出 RViz 和设备程序。车辆停止状态应以实际底盘表现为准。

## 常用配置

配置文件为 `~/.config/tianrover/my_tianrover.sh`。修改后，在新终端加载环境并重新启动对应程序。

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
修改后在启动程序的终端重新加载工作区环境，再重启对应程序；仅修改文件不会更新已运行节点。

## 常见问题

| 现象 | 检查方法 |
| --- | --- |
| 提示找不到 `tianrover` | 在当前终端重新加载三层环境，执行 `ros2 pkg prefix tianrover_bringup` 检查安装路径。 |
| 提示地图不存在 | 检查 `map:=` 指定的 YAML 路径，以及与它配套的地图图像是否存在。 |
| RViz 有模型但没有地图 | 确认建图或导航程序正在运行；建图和导航时 Fixed Frame 应为 `mars/map`。 |
| 尚未建图，只想查看设备 | 使用 `ros2 launch tianrover_rviz rviz.launch.py fixed_frame:=mars/odom`。 |
| 模型在其他电脑显示不完整 | RViz 所在电脑也需要安装本功能包和包含模型网格的 `tianrover_description`。 |
| 没有扫描或里程计 | 在加载环境后分别执行 `ros2 topic hz /mars/scan`、`ros2 topic hz /mars/odom`；检查设备终端的报错。 |
| 相机启动失败 | 检查相机连接和驱动；仅使用二维雷达建图时，可用 `use_zed:=false` 启动设备。 |
| 导航时扫描与地图不对齐 | 重新设置初始位置和朝向；仍不对齐时，停车并记录定位程序日志。 |

## 其他功能

- [Gmapping / Cartographer 二维建图](docs/guides/SLAM_2D.md)：其他建图算法及地图保存方式。
- [车辆模型与关节显示](docs/guides/MODEL.md)：模型查看和真实关节反馈接入说明。默认关节显示为零位，不代表电机反馈。
- [轮式里程计与 IMU 融合](docs/guides/EKF.md)：可选里程计融合，启用前需检查传感器方向和数据。
- [三维建图与地图保存](docs/guides/THREED.md)、[FAST-LIO2](docs/guides/FASTLIO.md)：进阶使用说明。三维定位导航仍有输入掉帧和 TF 时效问题，尚未完成实车运动验收。
