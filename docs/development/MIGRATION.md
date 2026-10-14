# 单包到多功能包迁移

本次仅调整 ROS 2 包职责、资源归属和调用路径。逐车环境配置、话题名称、命名空间、TF 前缀和算法参数沿用原契约。

## 包职责

仓库根目录不再是 ROS 功能包，`colcon` 递归发现以下并列功能包：

| 包 | 职责 |
| --- | --- |
| `tianrover` | ROS 2 元包，仅聚合产品功能包依赖并安装仓库文档 |
| `tianrover_common` | 共用环境配置、命名空间/TF 规则和 launch 辅助函数 |
| `tianrover_description` | 火星车 URDF、所需的 21 个 STL 和模型发布入口 |
| `tianrover_bringup` | 底盘、雷达、相机适配器和整车启动 |
| `tianrover_localization` | EKF、KISS/FAST-LIO 前端、TF 发布者检查和三维定位入口 |
| `tianrover_slam` | 二维/三维建图、RTAB-Map 后端和地图保存工具 |
| `tianrover_navigation` | Nav2 配置及定位导航组合入口 |
| `tianrover_calibration` | 六轮标定、录包、参数应用及回退工具 |
| `tianrover_rviz` | 二维/三维 RViz 入口和显示配置 |
| `tianrover_tests` | 独立 ROS 域内的集成测试，不由产品元包依赖 |

`tianrover` 使用 `ament_cmake` 构建并通过 `exec_depend` 聚合功能包。不会调用 ROS 1 的 `catkin_metapackage()`。
各功能包内的同名目录是 Python 模块目录，例如 `tianrover_localization/tianrover_localization/`；`scripts/` 是供 `ros2 run` 调用的短入口。

RTAB-Map 建图和地图定位共用 `tianrover_slam` 中的后端实现；`tianrover_localization` 提供定位启动入口。依赖方向为 localization → slam，不形成循环。
驱动仍复用 `tianbot_core`、`rslidar_sdk` 和 `zed_wrapper`，模型网格不再依赖 `tianbot_description`。

## 构建和测试

将整个仓库放在 `~/tianrover_ws/src/tianrover`。原来的 `--packages-select tianrover` 只会构建元包，必须改为构建其依赖：

```bash
source /opt/ros/humble/setup.bash
source ~/tianbot_ws/install/setup.bash
cd ~/tianrover_ws
colcon build --symlink-install --packages-up-to tianrover --event-handlers console_direct+
source install/setup.bash
```

在准备好的驱动和算法 underlay 上构建集成测试：

```bash
colcon build --symlink-install --packages-up-to tianrover_tests --event-handlers console_direct+
source install/setup.bash
colcon test --base-paths src/tianrover --event-handlers console_direct+
colcon test-result --verbose
```

集成测试默认包含 Gmapping、KISS 和 RTAB-Map，需先准备依赖清单及部署文档指定的上游包。
仅检查已安装的基础软件时，可在构建 `tianrover_tests` 时指定
`--cmake-args -DTIANROVER_TEST_OPTIONAL_FRONTENDS=OFF`，明确关闭这三组测试；这不代表它们已通过。

已有工作区升级时，先备份配置、地图、源码及旧构建产物。建议在新工作区构建，避免旧的单包安装遗留可执行文件和 launch。

## 客户命令变更

| 原命令所属包 | 启动文件 | 新包名 |
| --- | --- | --- |
| `tianrover` | `bringup.launch.py`、`bringup_3d.launch.py`、`base.launch.py`、`lidar.launch.py`、`scan.launch.py`、`zed.launch.py` | `tianrover_bringup` |
| `tianrover` | `description.launch.py` | `tianrover_description` |
| `tianrover` | `slam.launch.py`、`gmapping.launch.py`、`cartographer.launch.py`、`mapping_3d.launch.py`、`map_save.launch.py`、`map_save_3d.launch.py` | `tianrover_slam` |
| `tianrover` | `ekf.launch.py`、`laser_odom.launch.py`、`fastlio.launch.py`、`localization_3d.launch.py` | `tianrover_localization` |
| `tianrover` | `navigation.launch.py`、`navigation_3d.launch.py`、`localize_and_navigate_3d.launch.py` | `tianrover_navigation` |
| `tianrover` | `rviz.launch.py`、`rviz_3d.launch.py` | `tianrover_rviz` |
| `tianrover` | `calibration_motion.launch.py` | `tianrover_calibration` |

例如：

```bash
ros2 launch tianrover_bringup bringup.launch.py
ros2 launch tianrover_slam slam.launch.py
ros2 launch tianrover_rviz rviz.launch.py
```

旧 `ros2 launch tianrover ...` 和 `ros2 run tianrover ...` 命令不再作为入口。
完整客户流程见[README](../../README.md)。

## 资源与配置

- 逐车配置仍为 `~/.config/tianrover/my_tianrover.sh`，配置名和环境变量前缀保持不变。
- 示例配置位于 `tianrover_bringup/config/my_tianrover.sh.example`。
- URDF 位于 `tianrover_description/urdf/mars.urdf`，网格 URI 为 `package://tianrover_description/meshes/...`。
- 网格取自火星车原 `~/tianbot_ws/src/tianbot/tianbot_description/meshes`，保留原始文件内容，只纳入 URDF 使用的资源。
- 若外部配置引用仓库内的旧 `config/`、`urdf/` 路径，需修改为对应新包路径；仓库外的自定义 YAML 路径继续有效。
- 在另一台电脑运行 RViz，也要安装 `tianrover_description`，以便解析模型网格。
- 默认话题和 TF 名称保持原契约，例如 `/mars/scan`、`/mars/odom` 和 `mars/base_footprint`。

## 回退

升级车辆时，保留旧工作区，不在其目录内直接覆盖源码。停止新入口后，在新终端仅加载旧工作区并使用旧包名启动。
地图和外部配置保留在工作区之外；只回退 Git 提交不能清理旧 install 目录，因此不应在混有两种布局的安装目录中验证回退。

本次本机重构不等于车辆部署完成，也不替代实车定位、行驶、避障和停车验收。
