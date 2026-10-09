# 三维依赖构建（NX Humble）

> 2026-10-09：新增可选 FAST-LIO2（H16 + ZED 内置 IMU）前端，入口、待标定项、录包回放与离线保存见 [FASTLIO.md](FASTLIO.md)。`frontend:=kiss` 保留；每次只启动一个前端。FAST-LIO本轮仅编译和传感器检查，建图/保存验收由操作人员执行。旧文中只讨论KISS的部分不能当成FAST-LIO数据契约。

已验证平台：Ubuntu 22.04、ROS 2 Humble、ARM64。使用现有
`~/tianbot_ws/install` 的底盘与网格资源作为 underlay。
KISS 源码锁定在 dependencies-3d.repos；本包的点云可靠 QoS 补丁必须一起应用。
Jazzy / Ubuntu 24.04 尚未验证，不将 NX 的构建结果外推。

```bash
source /opt/ros/humble/setup.bash
source ~/tianbot_ws/install/setup.bash
sudo apt install python3-vcstool python3-pip libtbb-dev libeigen3-dev libfmt-dev \
  ros-humble-rtabmap-slam ros-humble-rtabmap-util ros-humble-rtabmap-msgs \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rosbag2 \
  ros-humble-pointcloud-to-laserscan ros-humble-robot-localization
cd ~/tianrover_ws
# 新部署使用下面命令导入；已有源码目录不要重复导入/覆盖。
vcs import src < src/tianrover/dependencies-3d.repos
patch --dry-run -p1 -d src/kiss_icp_vendor < src/tianrover/patches/kiss-reliable-cloud.patch
patch -p1 -d src/kiss_icp_vendor < src/tianrover/patches/kiss-reliable-cloud.patch
```

现场独立 rslidar_sdk 副本已经包含配置路径与 ROS 2 构建适配；新复制源码按 README 的
rslidar-config-path.patch、rslidar-ros2-build.patch 说明处理，然后应用 XYZIRT 补丁。
补丁已应用时不要重复执行：

```bash
patch --dry-run -p1 -d src/rslidar_sdk < src/tianrover/patches/rslidar-xyzirt.patch
patch -p1 -d src/rslidar_sdk < src/tianrover/patches/rslidar-xyzirt.patch
```

KISS 锁定版本依赖的 Sophus 要求 CMake ≥3.24；NX 系统 CMake 3.22.1 不足。
以下将工具与下载放在 /tmp，不替换系统 CMake。GitHub archive 下载在 NX 失败过，
使用 codeload 获取上游指定版本并通过 FetchContent 源码目录覆盖下载步骤：

```bash
python3 -m pip install --target /tmp/tianrover-cmake-tool 'cmake==3.28.4'
mkdir -p /tmp/tianrover-kiss-deps
curl -fL --retry 2 --max-time 60 \
  https://codeload.github.com/strasdat/Sophus/tar.gz/refs/tags/1.24.6 \
  -o /tmp/tianrover-kiss-deps/sophus.tar.gz
curl -fL --retry 2 --max-time 60 \
  https://codeload.github.com/Tessil/robin-map/tar.gz/refs/tags/v1.4.0 \
  -o /tmp/tianrover-kiss-deps/robin-map.tar.gz
tar -xzf /tmp/tianrover-kiss-deps/sophus.tar.gz -C /tmp/tianrover-kiss-deps
tar -xzf /tmp/tianrover-kiss-deps/robin-map.tar.gz -C /tmp/tianrover-kiss-deps
PATH=/tmp/tianrover-cmake-tool/cmake/data/bin:$PATH colcon build \
  --packages-select kiss_icp rslidar_sdk tianrover --allow-overriding rslidar_sdk \
  --symlink-install --parallel-workers 1 --cmake-force-configure \
  --cmake-args -DPOINT_TYPE=XYZIRT \
  -DFETCHCONTENT_SOURCE_DIR_SOPHUS=/tmp/tianrover-kiss-deps/Sophus-1.24.6 \
  -DFETCHCONTENT_SOURCE_DIR_TESSIL=/tmp/tianrover-kiss-deps/robin-map-1.4.0 \
  --event-handlers console_direct+
source install/setup.bash
ros2 pkg prefix kiss_icp
ros2 pkg prefix rslidar_sdk
colcon test --packages-select tianrover --event-handlers console_direct+
colcon test-result --verbose
```

现场本次构建复用了 `build/kiss_icp/_deps/sophus-src`（SOPHUS_VERSION=1.24.6）
和 `/tmp/robin-map-1.4.0`，两者对应上述相同版本。不要把 build 缓存作为新设备依赖来源。
使用 tar 同步源码时保留的旧 mtime 可能不触发 CMake 重配置，因此部署命令显式 force-configure。

SDK 发布端现场已确认 RELIABLE。同一时段 raw 订阅对比：可靠接收100帧/10秒，
尽力接收30帧/10秒，后者最大时间间隔1.4秒。因此本包首版的点云检查、KISS 输入使用
可靠 QoS；KISS 补丁增加 reliable_cloud 参数，上游默认 false 不变，本包 YAML 设 true。
如果替换雷达驱动，其发布端只有 BEST_EFFORT，必须同步修改配置并重新验收完整链路，
不能单独把检查端改成可靠订阅造成 QoS 不兼容。当前不修改全局 RMW 或内核网络参数。
