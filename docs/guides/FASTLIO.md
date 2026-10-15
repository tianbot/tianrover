# H16 + ZED 2i IMU → FAST-LIO2 → RTAB-Map（ROS 2 Humble / NX）

实现范围为前端和现有建图后端；不包含导航或自动标定。首次接入仅编译和订阅传感器。随后按用户要求在NX实际启动bringup，修复Humble缺少LogError以及ROS参数不支持YAML别名两个启动错误；启动初期TF守卫尚未发现时显示初始化等待，超时/冲突仍报错。输出队列改为同扫描末端时间戳的odom、去畸变点云和轮式TF齐备后成对发布，避免缺少一个odom的点云堵住后续帧。静止订阅8秒收到51条激光odom和51帧去畸变点云，末条时间戳一致且单调，TF守卫正常，IMU覆盖最大间隔7.54ms。未运行运动、RTAB累计节点、录包回放或保存验收，以下实车验收由操作人员执行。

## 固定版本与构建

- 上游：Ericsii/FAST_LIO_ROS2，`ros2` 分支提交 `2fffc570a25d0df172720bac034fbdb6a13d2162`。
- ikd-Tree：上游 gitlink 固定 `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`。
- 依赖清单：`dependencies/dependencies-fastlio.repos`；补丁：`patches/fast-lio-h16.patch`。
- 补丁保留 FAST-LIO2 估计器，只让 Livox CustomMsg 成为可选构建项、增加命名空间坐标配置、禁用内置 TF/合成点时间/扫描时长兜底、提高 IMU 队列并修正发布协方差的顺序。
- H16 使用 PointCloud2，`FASTLIO_WITH_LIVOX=OFF`，不用安装或启动 Livox 硬件驱动。启用 Livox 时另行安装 `livox_ros_driver2`，它不是本次范围。

NX 使用现有 Humble/Ubuntu 22.04 ARM64 环境，不切换系统或 ROS 发行版。重新获取源码时：

```bash
source /opt/ros/humble/setup.bash
source ~/tianbot_ws/install/setup.bash
cd ~/tianrover_ws
vcs import src < src/tianrover/dependencies/dependencies-fastlio.repos
git -C src/fast_lio_vendor submodule update --init --recursive
git -C src/fast_lio_vendor apply ../tianrover/patches/fast-lio-h16.patch
rosdep install --from-paths src/fast_lio_vendor src/tianrover --ignore-src -r -y --rosdistro humble
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-up-to fast_lio tianrover \
  --symlink-install --executor sequential --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF -DFASTLIO_WITH_LIVOX=OFF
source install/setup.bash
```

当前 NX 使用上述固定源码快照和补丁部署，源码目录的 `UPSTREAM_REVISION` 记录版本；重新获取命令面向新的源码目录，不要对已经补丁过的目录再应用补丁。

## 实际数据契约和待标定项

2026-10-09 只订阅数据的检查：

- H16 型号 `RSHELIOS_16P`；`/mars/points`，`mars/laser`，28800 点/帧，point_step=26。x/y/z/intensity FLOAT32；ring UINT16，offset=16，范围0..15；timestamp FLOAT64，offset=18。
- timestamp 是绝对秒，实测扫描跨度约0.099917秒，消息头减末点时间约−0.000036秒，属于扫描结束时刻。驱动 `use_lidar_clock=false`，使用主机时间。
- 转换器用真实最早逐点时间作为开始消息头，减去它生成 FLOAT32 相对秒 `time`，按时间排序；FAST-LIO `lidar_type=2`、`timestamp_unit=0`。末端时刻取原始有效点的真实最大时间，不能用滤波后最后一部分点或历史平均扫描时长代替。
- 没有有效 timestamp/ring、字段类型不符、时钟回退、时间跨度异常、消息头不符合扫描结束契约时，明确报错并停止向前端送该帧。不用方位角、扫描频率或接收时刻伪造时间。
- ZED IMU `/mars/zed2i/imu/data`，frame=`zed2i_imu_link`。真实加速度模长约9.78m/s²，角速度约0.001rad/s量级；wrapper 直接使用 SDK 加速度 m/s²、角速度度/s转换成rad/s。检查时现有配置100Hz，收到约97Hz，时间戳单调，但最大间隔0.256秒。新配置请求200Hz且不同步到图像曝光；仅重新启动传感器后，实收约199.19Hz，单独IMU订阅最大间隔7.54ms，两路同时订阅最大间隔10.04ms，均未发现时间戳回退。
- 本轮负载下收到H16约3.37Hz，消息最大间隔约1.10秒，IMU收到的最后一条时间比主机当前时间约落后1.41秒。这是当前接收表现，不能当成雷达—IMU标定偏差；需要在停止旧KISS/RTAB链路后复查。默认拒绝覆盖扫描区间的IMU间隔超过0.1秒，等待覆盖超时会报错。

最终只运行传感器和模型静态TF的复查（没有底层控制/FAST-LIO/RTAB）：H16实收约9.36Hz，消息最大间隔约0.201秒，仍有一次漏帧表现；每帧逐点跨度0.100053秒，消息头接近扫描末端，接收延迟约5.93ms。IMU约199.19Hz，最大间隔10.04ms，接收延迟约1.26ms。此前差的接收表现没有用作时间偏差，也没有声称已经完成负载/运动测试。

ZED RGB、注册深度、camera_info均收到真实数据，均为640×360、frame=`zed2i_left_camera_frame_optical`，观测的三者时间戳相同；RGB编码bgra8，深度32FC1。相机出厂IMU静态TF已连接，能查询base_link↔ZED IMU以及ZED IMU↔laser。该连接来自CAD/SDK，仍不等于实测外参标定。

`tianrover_localization/config/fastlio_calibration.yaml` 中外参、时间偏差默认 null、状态 false。默认拒绝启动，避免把猜测当成标定值。实测配置必须填写：

```yaml
calibration_confirmed: true
extrinsic_T: [填写三个实测米值]
extrinsic_R: [填写九个实测旋转矩阵值，行优先]
time_offset_lidar_to_imu: 填写实测秒值
max_imu_gap: 0.1
```

方向定义：`p_imu = R * p_lidar + T`；上游应用 `t_imu_corrected = t_imu - time_offset_lidar_to_imu`。`common.time_sync_en` 始终 false，它不是 H16 通用自动同步机制。自动外参估计也关闭。

仅用于初步评估，可显式传 `allow_unverified:=true`：从现有 CAD 挂载和 ZED 出厂静态 TF 求名义外参，未填写的偏差临时用0，并在日志/有效参数文件中标记未验证。它不代表交付标定完成。ZED 只允许出厂相机→IMU静态TF；ZED odom/map TF仍关闭。

## 坐标、里程计与后端

- 轮式 `/mars/odom` 和 `mars/odom → mars/base_footprint → mars/base_link` 保留，FAST-LIO不回灌控制器/EKF。
- FAST-LIO原始位姿是IMU位姿；适配器计算 `T_lidarodom_base = T_lidarodom_imu * inverse(T_base_imu)`，发布独立 `/mars/odom_lidar`，child为 `mars/base_link`，包含位置协方差的杆臂转换。
- 不给base_link第二个TF父节点；用 `T_lidarodom_wheelodom = T_lidarodom_base * inverse(T_wheelodom_base)` 发布唯一 `mars/odom_lidar → mars/odom` 对齐边。
- RTAB-Map使用连续的 `/mars/odom_lidar`，发布 `mars/map → mars/odom_lidar`。完整链为 `map → odom_lidar → odom → base_footprint → base_link`。
- 输入/输出会检查唯一前端和雷达发布者；TF守卫按发布者GID检查wheel odom的父变换，旧KISS/旧RTAB/重复FAST-LIO混开时阻断数据，不继续静默建图。
- `/mars/lidar/cloud_deskewed` 是FAST-LIO已去畸变的末端点云，经外参变换到base_link；消息头与对应激光odom相同。不使用世界坐标 `cloud_registered` 当本地点云，也不再做一次去畸变。
- RTAB保持ICP配准，追加ZED RGB、注册深度和RGB camera_info；`use_rgbd:=false`可只做激光图。ZED不提供另一份位姿给后端。
- KISS通过 `frontend:=kiss` 保留原入口；它不读取ZED IMU。每次只启一个前端，切换前停止原前端、其TF守卫和原RTAB后端。

## 最少实车步骤

先在原终端正常 Ctrl+C 停止旧KISS/RTAB/bringup，等退出；不要与以下入口混开。新建数据库名，不复用那份只有一个Node的旧文件。
已部署火星车的新 Bash 终端会自动加载运行环境及车辆配置，无需重复 source。
以下使用配置中的 `TIANBOT_NAME=mars`；自动加载说明及排错见[README](../../README.md)。

终端1（静止启动，等待IMU初始化；底层控制方式不变）：

```bash
ros2 launch tianrover_bringup bringup_3d.launch.py frontend:=fastlio \
  calibration:=$HOME/fastlio_calibration.yaml
```

尚未标定，仅初步评估时替换为：

```bash
ros2 launch tianrover_bringup bringup_3d.launch.py frontend:=fastlio allow_unverified:=true
```

终端2：

```bash
ros2 launch tianrover_slam mapping_3d.launch.py frontend:=fastlio \
  database:=$HOME/maps/fastlio_delivery_v1.db use_rgbd:=true
```

RViz（本机具备ROS/显示环境时运行；固定坐标默认odom_lidar，后端地图展示可切换mars/map）：

```bash
ros2 launch tianrover_rviz rviz_3d.launch.py
```

先检查，不直接移动：

```bash
ros2 topic echo /mars/lidar/fastlio_status
ros2 topic hz /mars/points
ros2 topic hz /mars/zed2i/imu/data
ros2 topic hz /mars/odom_lidar
ros2 topic hz /mars/lidar/cloud_deskewed
ros2 topic echo /mars/odom_lidar --once
ros2 run tf2_ros tf2_echo mars/odom_lidar mars/base_link
ros2 topic echo /mars/info
ros2 topic echo /mars/mapGraph --once
```

确认持续有效状态、无丢失IMU覆盖/时钟/TF冲突，再由操作人员遥控低速走一段和转弯；检查静止不漂、轨迹连续、点云墙面不重影，RTAB当前节点ID和图的poses_id数量随有位移的关键帧累计。长时间不移动只存一个节点可以正常；真实行驶后仍只有一个节点必须看同步、odom变化、ICP链接和后端日志，不能直接声称保存成功。

## 原始录包与离线回放

仅录原始传感器、轮式odom、相机和必要TF，避免回放时重复发布算法输出：

```bash
ros2 bag record -o $HOME/bags/fastlio_raw_v1 \
  --qos-profile-overrides-path $HOME/tianrover_ws/src/tianrover/tianrover_localization/config/fastlio_bag_qos.yaml \
  /mars/points /mars/zed2i/imu/data /mars/odom /mars/fastlio/wheel_tf /tf_static \
  /mars/zed2i/rgb/color/rect/image /mars/zed2i/rgb/color/rect/camera_info \
  /mars/zed2i/depth/depth_registered
```

`fastlio/wheel_tf`是从原/tf筛出的轮式odom→base_footprint；不要录整个/tf再回放算法已经产生的对齐TF。非mars命名空间请对应修改录包topic和QoS文件。

离线时先停止实车所有节点，以下三个独立终端不要运行bringup（它会打开真实驱动）：

```bash
ros2 launch tianrover_localization laser_odom.launch.py frontend:=fastlio use_sim_time:=true \
  calibration:=$HOME/fastlio_calibration.yaml
```

```bash
ros2 launch tianrover_slam mapping_3d.launch.py frontend:=fastlio use_sim_time:=true \
  database:=$HOME/maps/fastlio_replay_v1.db use_rgbd:=true
```

```bash
ros2 bag play $HOME/bags/fastlio_raw_v1 --clock --rate 0.5 \
  --qos-profile-overrides-path $HOME/tianrover_ws/src/tianrover/tianrover_localization/config/fastlio_bag_qos.yaml \
  --remap /mars/fastlio/wheel_tf:=/tf
```

FAST-LIO预备节点会等静态TF，回放需在30秒内启动。原始数据topic名称保持一致，不回放KISS、FAST-LIO原输出或RTAB map TF。回放前端只使用一个新时钟周期；重新播放前重启前后端，不让时间倒退后沿用旧状态。

## 地图落盘：保留严格校验

此前日志显示RTAB内存图曾累计184个节点；在线backup关闭/复制/重载后归为1个节点，旧delivery.db实际Node=1、Link=0、.back=0字节。能确认重置发生在backup链路附近，尚不能仅靠已有日志确定具体底层写入失败原因。不是“从来没有收到点云”，也不能靠关掉校验解决。

现有保存入口已改为正常退出后离线SQLite备份：不调用危险的在线backup服务，拒绝数据库仍被进程打开、输出覆盖和少于两个Node。切换FAST-LIO本身并不证明数据库保存问题已经验收通过。

完成行驶建图后，先在终端2正常Ctrl+C退出RTAB-Map，等它完成数据库关闭，不用kill -9；保留终端1。检查源库：

```bash
python3 - "$HOME/maps/fastlio_delivery_v1.db" <<'PY'
import sqlite3,sys
from pathlib import Path
with sqlite3.connect(Path(sys.argv[1]).resolve().as_uri()+'?mode=ro',uri=True) as db:
    print('integrity',db.execute('pragma quick_check').fetchone()[0])
    for table in ('Node','Link','Data'):
        print(table,db.execute('select count(*) from '+table).fetchone()[0])
    print('scan bytes',db.execute('select sum(length(scan)) from Data').fetchone()[0])
PY
ros2 launch tianrover_slam map_save_3d.launch.py \
  database:=$HOME/maps/fastlio_delivery_v1.db \
  output:=$HOME/maps/fastlio_delivery_saved_v1.db offline:=true
```

同样支持：

```bash
ros2 run tianrover_slam map_save_3d --offline \
  --database $HOME/maps/fastlio_delivery_v1.db --output $HOME/maps/fastlio_delivery_saved_v1.db
```

验收必须同时确认源库和保存库Node>=2、存在链接和非空扫描数据；仅有文件或PCD不算RTAB三维地图保存成功。旧文件保留，不删除、不用它作为新图的输出路径。

2026-10-09 最终修正后静止观察20秒：收到131个相同时间戳的odom/点云对，137次输入审计全部有效，未发现时间戳回退；观察期间未见持续错误、Traceback或进程退出；首扫描早于首条IMU的正常初始化丢帧保留校验并改为初始化提示。没有发送车辆运动指令，也没有启动RTAB后端或测试地图保存。

## 已保存地图的地面导航（FAST-LIO）

2026-10-09 用户完成实车建图、正常退出及离线保存。NX只读核查：fastlio_test_01.db与fastlio_test_01_saved.db均为221163520字节，SQLite quick_check=ok，Node=63、Link=124、Data=63，扫描数据21599091字节。保存链路本次成功；导航运动尚未验收。Mem/SaveDepth16Format=false与RVL不兼容时自动使用PNG压缩，此警告不代表数据库保存失败。

已部署火星车的新 Bash 终端会自动加载环境和车辆配置，以下以 `TIANBOT_NAME=mars` 为例。

保留正在运行的FAST-LIO bringup；如果已停止，在终端1启动（不要重复启动）：

```bash
ros2 launch tianrover_bringup bringup_3d.launch.py frontend:=fastlio allow_unverified:=true
```

停止mapping_3d，只运行一个RTAB-Map后端。在终端2加载保存库并启动Nav2：

```bash
export TIANROVER_MAX_SPEED=0.1
ros2 launch tianrover_navigation localize_and_navigate_3d.launch.py \
  frontend:=fastlio database:=$HOME/maps/fastlio_test_01_saved.db
```

导航入口按frontend选择避障点云：FAST-LIO订阅/mars/lidar/cloud_deskewed，KISS订阅/mars/kiss/frame；沿用轮式odom与现有TF链。RTAB-Map负责地图定位与map TF，不另开AMCL。未知外参与时间偏差仍未标定，allow_unverified只供此次调试。

终端3在有图形桌面的会话启动RViz：

```bash
ros2 launch tianrover_rviz rviz.launch.py fixed_frame:=mars/map
```

检查模型、/mars/map和TF均正常；可添加PointCloud2 /mars/cloud_map查看RTAB累积三维地图，添加/mars/lidar/cloud_deskewed查看当前避障点云。用2D Pose Estimate标注当前实际位置与朝向（/mars/initialpose），待扫描/点云与地图对齐、定位稳定，再用2D Goal Pose发送附近空旷地面的目标（/mars/goal_pose）。

发送目标前检查：

```bash
ros2 lifecycle get /mars/controller_server
ros2 lifecycle get /mars/planner_server
ros2 topic info /mars/lidar/cloud_deskewed
ros2 topic echo /mars/map --once --no-arr
```

两个生命周期节点应为active，避障点云应有发布者和导航订阅者，地图应有有效尺寸和分辨率。没有定位或地图未对齐时先排查后端日志，不发送目标。

此链路使用三维地图定位、三维点云避障，Nav2在地面代价地图中规划：Navfn Dijkstra全局规划+DWB局部控制，不支持任意三维轨迹/楼梯导航。此次仅修正导航前端参数传递与点云订阅并检查Python语法，未启动导航或发送运动目标，实车验收由用户执行。

### 定位阶段无地图消息的已知阻断与修复（2026-10-09）

NX只读排查：/mars/map存在RTAB-Map发布者，RViz/global_costmap订阅QoS匹配，但没有消息；RTAB日志明确加载保存栅格474x725、63个节点，随后持续报Did not receive data。原始雷达、ZED IMU/RGB-D有数据，FAST-LIO原始odom/body cloud仍有输出且观察窗口没有时间倒退，但适配后的odom_lidar/cloud_deskewed均为0条。fastlio_output持续报discontinuous FAST-LIO stamp，旧判断dt<=1秒把正向丢帧间隔误判为时钟不连续，并因previous不更新永久拒绝后续帧。

修复：保留严格的倒退/重复时间、非有限值、过期或未来输出、同一扫描odom/点云配对以及轮式TF检查；允许真实正向帧间隔超过1秒，诊断twist按真实间隔求平均并提示输出gap。已同步NX源码，Python语法检查通过，安装文件是源码符号链接，无需重新编译。未重启进程、未测试地图恢复或导航运动。

用户需先正常退出定位导航、RViz和bringup，再启动bringup；确认/mars/odom_lidar和/mars/lidar/cloud_deskewed能收到数据，之后启动localize_and_navigate_3d，最后启动RViz。检查地图使用transient_local订阅：ros2 topic echo /mars/map --once --no-arr --qos-durability transient_local。保存库不删除、不重新建图。若地图仍无消息，继续检查RTAB同步日志，不直接发送目标。

## 可选底盘 IMU，不启动 ZED（2026-10-09）

H16→FAST-LIO2支持imu_source:=zed或chassis；use_zed独立控制相机驱动，默认保持ZED方案。chassis选择/mars/imu与mars/imu_link，读取fastlio_chassis_calibration.yaml；zed选择原相机IMU与原校准文件。文件的imu_source若与选择不一致则明确拒绝，未知外参/时间差仍要求allow_unverified，不复用ZED校准。

NX静止数据评估：283条底盘IMU，约47.96Hz，间隔17.0～260.2ms，没有倒退/重复；加速度模长均值9.8064m/s²，静止角速度接近零。TF base_link→imu_link与imu_link→laser均存在，采用CAD挂载值而非实测标定值。底盘ROS2驱动直接转发固件加速度/角速度，并在串口包接收时用host clock now打时间戳；没有硬件采样时刻。加速度尺度与SI重力一致；静止数据不能证明陀螺单位/转动轴符号，需运动录包确认。50Hz及串口抖动对动态去畸变不如已观测约200Hz的ZED方案，当前仅支持低速实验，未证明交付质量。100ms IMU gap检查保留，260ms缺口覆盖的扫描会被拒绝。上游支持外部IMU但要求激光/IMU时间对齐：https://github.com/Ericsii/FAST_LIO_ROS2/blob/2fffc570a25d0df172720bac034fbdb6a13d2162/README.md

先正常退出旧bringup、单独base/description、RTAB后端和RViz，避免重复驱动/TF/前端。
已部署火星车打开新 Bash 终端即可执行以下命令，环境和车辆配置会自动加载。

终端1：
```bash
ros2 launch tianrover_bringup bringup_3d.launch.py \
  frontend:=fastlio imu_source:=chassis use_zed:=false allow_unverified:=true
```

车辆静止等待初始化。日志应显示IMU source=chassis、topic=/mars/imu、frame=mars/imu_link；不应有zed_container/zed2i节点。检查：
```bash
ros2 node list
ros2 topic hz /mars/imu
ros2 topic echo /mars/lidar/fastlio_status --once
ros2 topic echo /mars/odom_lidar --once --no-arr
ros2 topic echo /mars/lidar/cloud_deskewed --once --no-arr
```

观察持续valid:true与连续odom/点云后，再低速直行与转弯，检查固定墙面不双影、轨迹连续。偶发gap拒帧应有明确日志；持续报IMU measurement gap/coverage timeout则先停止运动并录包，不放宽gap掩盖缺样。用录包中的角速度积分与实际慢转90°作粗检，正向yaw与ROS轴向一致、积分约pi/2；正式动态轴向/单位/外参/时间差标定仍是待办。

已有地图定位导航终端2（此时不要同时启动mapping）：
```bash
export TIANROVER_MAX_SPEED=0.1
ros2 launch tianrover_navigation localize_and_navigate_3d.launch.py \
  frontend:=fastlio use_rgbd:=false \
  database:=$HOME/maps/fastlio_test_01_saved.db
ros2 topic echo /mars/map --once --no-arr \
  --qos-durability transient_local --qos-reliability reliable
```

use_rgbd:=false已贯通组合入口；关闭相机时不要沿用默认true，否则RTAB等待相机输入。新建图的替代终端2（使用新文件名、不覆盖旧图）：
```bash
ros2 launch tianrover_slam mapping_3d.launch.py frontend:=fastlio use_rgbd:=false \
  database:=$HOME/maps/chassis_imu_test_01.db
```

终端3最后再开RViz，先只显示地图与实时点云，确认定位对齐再发目标：
```bash
ros2 launch tianrover_rviz rviz.launch.py fixed_frame:=mars/map
```

原始录包（终端4，命名每次换新）：
```bash
ros2 bag record -o $HOME/bags/chassis_imu_test_01 \
  --qos-profile-overrides-path $HOME/tianrover_ws/src/tianrover/tianrover_localization/config/fastlio_bag_qos.yaml \
  /mars/points /mars/imu /mars/odom /mars/fastlio/wheel_tf /tf_static
```

完成新建图后正常Ctrl+C退出RTAB，等待落盘，再保存：
```bash
ros2 launch tianrover_slam map_save_3d.launch.py offline:=true \
  database:=$HOME/maps/chassis_imu_test_01.db \
  output:=$HOME/maps/chassis_imu_test_01_saved.db
```

底盘数据离线回放：先停止真实驱动，分三个终端依次启动laser_odom（frontend:=fastlio imu_source:=chassis allow_unverified:=true use_sim_time:=true）、mapping_3d（frontend:=fastlio use_rgbd:=false use_sim_time:=true，新database路径）及bag play（--clock --rate 0.5 --remap /mars/fastlio/wheel_tf:=/tf），遵守前端30秒TF等待。原始包必须含/tf_static与筛出的轮式TF，不回放算法输出或整个/tf。

回退时正常退出当前链路，重启bringup_3d frontend:=fastlio imu_source:=zed use_zed:=true allow_unverified:=true；对应RTAB入口恢复use_rgbd:=true。不要在线切换运行中的IMU，也不要同时启动两个FAST-LIO前端。
