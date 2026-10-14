# 多功能包重构验证（2026-10-14）

本次对照本机 Tianbot 的并列功能包结构，将原单包拆为产品元包、八个功能包和一个独立集成测试包。职责与迁移步骤见[迁移说明](MIGRATION.md)。

## 已验证

- `colcon list --base-paths .` 识别 10 个 ROS 2 `ament_cmake` 包，内部依赖可按顺序构建。
- 显式加载 ROS 2 Humble，使用 `/usr/bin/python3`，在 `/tmp/tianrover-multipackage` 构建，未使用本机 ROS 1 环境。
- 10 个包使用 `--symlink-install` 构建成功。`tianrover` 元包不安装 Python 实现、launch 或可执行节点。
- 21 个 STL 从火星车 `192.168.2.225` 原模型目录取回，与车上 SHA-256 逐一一致；校验清单为 `tianrover_description/meshes.sha256`。
- URDF 除网格所属包 URI 外，其几何、关节、质量和惯性内容与重构前一致。
- 安装后的所有启动文件（含组件适配器）可加载；21 个模型网格均可从安装后的 `tianrover_description` 解析。
- 原有配置、点云前端、地图数据库、六轮标定单元测试通过。
- 模型关节 TF、EKF、两机器人 scan → SLAM → 地图保存 → AMCL/Nav2 路径规划、TF 发布者冲突与恢复，以及标定运行时检查通过。
- RViz 配置生成测试确认模型、扫描和固定坐标系使用同一机器人命名空间；没有运行 RViz 图形窗口。
- `colcon test-result --verbose`：92 tests、0 errors、0 failures、0 skipped。
- 文档本地链接、Python 未定义名称检查和 `git diff --check` 通过。

## 测试环境与复查

本机没有安装 `robot_localization`。测试时仅在 `/tmp/tianrover-test-deps` 构建了官方 Humble 源码：

- 仓库：https://github.com/cra-ros-pkg/robot_localization
- 分支：`humble-devel`
- 提交：`8696ee5a9e4f959fcaae37835dcf2ed12ead581b`

没有安装系统包，也没有切换车辆程序。产品仍通过 `package.xml` 声明正常的 `robot_localization` 依赖，不依赖这些临时路径。

本次复查环境：

```bash
source /opt/ros/humble/setup.bash
source /tmp/tianrover-test-deps/install/setup.bash
source /tmp/tianrover-multipackage/install/local_setup.bash
export ROS_LOG_DIR=/tmp/tianrover-multipackage/ros-logs
export ROS_HOME=/tmp/tianrover-multipackage/ros-home
colcon test-result --test-result-base /tmp/tianrover-multipackage/build --verbose
```

日志与产物：

- `/tmp/tianrover-multipackage/final-build.log`
- `/tmp/tianrover-multipackage/test.log`：首次执行，包含后来已修正的路径错误和当时缺失 EKF 依赖的失败记录。
- `/tmp/tianrover-multipackage/retest.log`：安装入口和标定运行时复测。
- `/tmp/tianrover-multipackage/ekf-retest.log`：补齐临时依赖后的 EKF 复测，安装入口随后又单独复测。
- `/tmp/tianrover-multipackage/installed-retest.log`：最终安装入口、组件和 RViz 配置检查。
- `/tmp/tianrover-multipackage/build/*/test_results/`：汇总读取的最终测试结果。

## 未执行范围

本机缺少 Gmapping、KISS-ICP 和 RTAB-Map 上游运行包，本次显式使用
`TIANROVER_TEST_OPTIONAL_FRONTENDS=OFF`，关闭对应三组集成测试。上述 92 项结果不包含这三组；源码中该选项默认仍为 ON。

没有部署重构版到车辆，没有测试真实传感器性能、地图质量、运动定位、避障或停车，也没有完成三维导航交付验收。
