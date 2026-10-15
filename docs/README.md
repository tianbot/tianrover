# 文档目录

日常操作从[客户使用说明](../README.md)开始：打开终端并检查环境、启动设备、二维建图、保存地图和导航。

## 功能指南

以下文档包含进阶操作、配置细节及对应功能的验证边界。

| 文档 | 内容 |
| --- | --- |
| [键盘遥控](../README.md#键盘遥控) | 全向键位、命名空间、速度和停车 |
| [Gazebo 仿真](../tianrover_gazebo/README.md) | Mini 车型、障碍场地、ROS 2 控制接口 |
| [二维建图](guides/SLAM_2D.md) | Gmapping、Cartographer、保存地图和定位 |
| [车辆模型](guides/MODEL.md) | 模型显示、关节反馈接入及模型修正依据 |
| [里程计融合](guides/EKF.md) | 轮式里程计与 IMU 融合、切换与回退 |
| [三维链路](guides/THREED.md) | 三维建图、数据库保存、定位和底盘标定 |
| [FAST-LIO2](guides/FASTLIO.md) | H16 与 IMU 接入、录包、回放及地图保存 |

三维定位导航尚未完成实车运动验收，具体限制见对应指南。

## 部署与研发维护

| 文档 | 内容 |
| --- | --- |
| [多包结构与迁移](development/MIGRATION.md) | 包职责、新旧命令和工作区升级方式 |
| [部署与硬件适配](development/DEPLOYMENT.md) | 源码构建、驱动补丁、配置契约、组件扩展及回退 |
| [三维依赖构建](development/DEPENDENCIES_3D.md) | 三维依赖导入、补丁和编译 |
| [多包重构验证](development/VALIDATION_REFACTOR.md) | 当前拆包、网格和跨包测试证据及未执行范围 |
| [雷达投影范围检查](development/SLAM_CORRIDOR_RANGE_AUDIT.md) | Helios 16 高度过滤、静态采样、配置重载及未验证范围 |
| [基础验证记录](development/VALIDATION.md) | 历史软件与传感器检查及其边界 |
| [三维验证记录](development/VALIDATION_3D.md) | 三维链路和标定相关验证 |
| [研发待办](development/TODO.md) | 后续定位方案、标定与现场验收事项 |

验证记录保留当时的日期和现场信息，不代表当前车辆已重新验收。

## 仓库目录

功能代码放在并列的 `tianrover_*` 包中，具体职责见[多包结构与迁移](development/MIGRATION.md)。

| 目录 | 内容 |
| --- | --- |
| `tianrover/` | 元包的 `package.xml` 和 `CMakeLists.txt` |
| `tianrover_*/launch/`、`tianrover_*/config/` | 对应功能的启动文件和参数模板 |
| `tianrover_*/tianrover_*/`、`scripts/`、`src/` | 对应功能的 Python 模块、可执行入口和 C++ 节点 |
| `tianrover_description/urdf/`、`meshes/` | 车辆模型及其全部网格 |
| `tianrover_*/test/` | 各包的单元测试；集成测试位于 `tianrover_tests/test/` |
| [dependencies/](../dependencies/) | 锁定源码版本的 `.repos` 清单 |
| `patches/` | 配套上游驱动和算法补丁 |
| `docs/guides/` | 功能指南 |
| `docs/development/` | 部署、研发和验证资料 |

逐车配置放在 `~/.config/tianrover/`，地图和录包放在仓库外。
