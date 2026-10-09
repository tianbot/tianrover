# 三维建图、定位、导航与底盘标定

> 2026-10-09：新增可选 FAST-LIO2（H16 + ZED 内置 IMU）前端，入口、待标定项、录包回放与离线保存见 [FASTLIO.md](FASTLIO.md)。`frontend:=kiss` 保留；每次只启动一个前端。FAST-LIO本轮仅编译和传感器检查，建图/保存验收由操作人员执行。旧文中只讨论KISS的部分不能当成FAST-LIO数据契约。

目标链路：H16 → rslidar_sdk XYZIRT → 点云检查 → KISS-ICP → RTAB-Map → Nav2。
第一版不把 IMU 或轮式 odom 接入激光轨迹。现场 NX 为 Ubuntu 22.04 / ROS 2 Humble。

## 2026-10-09 保存流程修订（未执行测试）

NX 昨夜日志显示 backup 前 local map=184，backup 后从节点 1 重启；
当前 delivery.db 仅一个节点、无连接，delivery.db.back 为零字节。
该次现场 backup/重载发生数据丢失，具体底层原因未确认。
在线 backup 保存已禁用，不再通过该服务关闭并重载 SLAM。
本次仅修改代码、部署并读取文件；未运行保存或定位测试。

保存改为：停车 → 在 mapping_3d 的终端 Ctrl+C → 等待 RTAB-Map 正常退出 → 保存闭合数据库。
保留 bringup_3d，不要 kill -9，不删除旧 delivery.db，也不要复用损坏旧库当作完整地图。
用新的工作数据库重新建图，例如 delivery_3d_v2.db。

```bash
# mapping 正常退出后，另一终端执行；输出必须是新文件：
ros2 launch tianrover map_save_3d.launch.py \
  database:=$HOME/maps/delivery_3d_v2.db output:=$HOME/maps/delivery_3d_v2_saved.db
# ros2 run 对应入口，必须明确 --offline：
ros2 run tianrover map_save_3d --offline \
  --database "$HOME/maps/delivery_3d_v2.db" --output "$HOME/maps/delivery_3d_v2_saved_run.db"
```

脚本拒绝仍被本机进程打开的数据库，保留 SQLite 完整性/图节点检查及不覆盖输出的约定。
地图不足两个节点仍拒绝，不能把一次静态扫描当成已完成的导航地图。
ROS 参数用 remove_ros_args 过滤，业务参数保持严格 parse_args。
成功后可停止原 mapping 并使用 localize_and_navigate_3d 读取 saved.db；
该数据库包含 RTAB-Map 图和扫描，不是 PCD 文件。
旧软件测试记录中的在线保存场景不再代表这一版实现；测试更新/执行留待后续。

## 当前状态

已增加激光里程计、三维建图/定位、导航 launch 和配置；KISS-ICP、XYZIRT SDK 和 TianRover 包在 NX 构建通过；软件集成测试通过，真实 H16 静态链路约 10 Hz；运动中的精度和实车导航尚未验收。
独立激光入口默认不发 TF，输出 `/<namespace>/odom_lidar`，便于先检查数据。
三维 bringup 将底盘 wheel odom 改为诊断话题 `odom_raw`，关闭底盘 odom TF，
由 KISS 发布 `odom → base_footprint`，速度适配器只对激光位姿求导。
不要与原二维 bringup、EKF 或另一套 KISS 同时启动；已通过真实发布者 GID 的冲突拒绝与恢复测试。

现场原 SDK 发布 XYZI，没有逐点时间。`patches/rslidar-xyzirt.patch` 改为可选择 XYZIRT，
配置使用主机时间、末点作为扫描时间。必须实测确认字段、扫描时间跨度与 header 一致。
点云检查默认拒绝缺失时间、无效跨度、坐标异常和缺少雷达到底盘 TF。
整数逐点时间未声明单位时拒绝，不能隐式当作秒。
外参沿用 CAD，仅表示 TF 存在，尚不代表实测标定完成。

KISS 源码锁定见 `dependencies-3d.repos`。当前版本依赖的 Sophus 需要 CMake ≥3.24；
NX 系统为 3.22.1，正在使用 `/tmp` 中的独立 CMake 3.28.4 工具构建，未替换系统工具。
RTAB-Map 相关 Humble 包已经安装，真实 SDK 已切换为 XYZIRT。
依赖导入、补丁及构建命令见 [依赖构建](DEPENDENCIES_3D.md)。

以下入口已在隔离域用合成数据验证。现场当前仍保留原底盘 wheel TF，
独立 KISS 以不发布 TF 的 shadow 模式运行；尚未切换为下述完整三维 bringup。
每个终端先 source 三层环境；命令依次执行，建图与定位不能同时启动：

```bash
source /opt/ros/humble/setup.bash
source ~/tianbot_ws/install/setup.bash
source ~/tianrover_ws/install/setup.bash
export TIANBOT_NAME=mars
# 停止旧 bringup 后启动；不要重复开启底盘和雷达。
ros2 launch tianrover bringup_3d.launch.py
ros2 launch tianrover mapping_3d.launch.py database:=$HOME/maps/mars3d.db
# 停车，先 Ctrl+C 正常退出 mapping；保留 bringup_3d，再保存：
ros2 launch tianrover map_save_3d.launch.py database:=$HOME/maps/mars3d.db \
  output:=$HOME/maps/mars3d_saved.db
# 使用保存副本定位与导航，bringup_3d 保持运行：
ros2 launch tianrover localize_and_navigate_3d.launch.py database:=$HOME/maps/mars3d_saved.db
```

导航复用 Nav2 Dijkstra 全局规划和 DWB 局部控制；三维入口由 RTAB-Map 提供地图及定位，
不再启动 AMCL/map_server。当前 footprint 使用 CAD 包络加余量，仍需实测运动包络与制动距离。
已验证实际 KISS 节点、完整 bringup、RTAB-Map 回环/保存/重新加载，以及组合定位导航入口
的 Nav2 路径规划，输入均为隔离域合成数据。尚未发送导航目标或验证 DWB 控车。
具体范围及日志见 [验证记录](VALIDATION_3D.md)。

仅检查独立激光里程计时，在已有唯一雷达及模型 TF 的前提下启动：

```bash
ros2 launch tianrover laser_odom.launch.py
# 默认 output=/mars/odom_lidar、publish_tf=false，不替换当前底盘 TF。
ros2 topic echo /mars/lidar/timing --once
ros2 topic hz /mars/odom_lidar
```

不要在现有 shadow KISS 上重复启动该入口。完整三维 bringup 必须先停旧 bringup、
独立雷达、shadow KISS 和同 namespace 的 EKF；底盘的 publish_tf 只在构造时读取，
不能用运行时 param set 代替停止与重启。四个被动悬架的默认展示不影响独立激光轨迹。

## 标定求解与数据边界

只标定十二个主动关节对应的六个有效轮径和六个转向零偏。
四个 rocker-bogie 被动关节没有编码器，模型保留中立展示；标定要求平地并确认几何。
实车当前 joint_states 是默认全零展示，没有实际反馈。不能用它或 cmd_vel 当作测量。

实现中的离线数据格式为一个目录，含 `samples.npz` 和 `manifest.json`。
NPZ 数组：`time[N]`（秒）、`velocity[N,3]`（纯激光机体系 vx/vy/wz）、
`omega[N,6]`（实际轮轴 rad/s）、`steer[N,6]`（实际转向 rad），轮序 LF/LM/LB/RF/RM/RB。
Manifest 必须含 reference=kiss_icp_only、dataset_kind=hardware 或 synthetic、recording_id、
原始录包 source_sha256、samples_sha256 和 profile_sha256。
Profile 包含六轮实际关节名称、转向中心 xy、名义半径，以及反馈来源、确认过的外参、
平地、反馈测量属性、时钟偏移和最大时间抖动；任何未确认项拒绝求解。

时间对齐模块使用激光 SE(2) 位姿增量和真实反馈插值，禁止外推；反馈间隔上限 50 ms，
轮轴测量必须明确为速度、连续位置或环绕位置。它不读轮式 odom。
已实现录包和 rosbag 提取入口，录包只接受确认过的真实反馈节点及唯一 KISS 发布者。
提取检查逐点时间审计、平地姿态、坐标系、反馈覆盖范围，禁止外推；
已通过实际 rosbag2 存储/反序列化的合成消息测试，仍待真实录包验收。

离线命令入口：

```bash
ros2 run tianrover calibrate solve --dataset train --profile profile.yaml --output candidate
ros2 run tianrover calibrate validate --dataset independent --profile profile.yaml \
  --candidate candidate/parameters.yaml --output validation
```

求解输出 parameters.yaml 与 report.json；校验输出绑定候选文件 SHA-256 的报告。
验证必须使用不同原始录包、不同 recording_id 和不同样本文件，拒绝用训练集验证。
结果目录不得覆盖，以免遗留旧审批报告。合成数据验证明确标记不可用于硬件应用。
不足 15 秒、平移不足 2 m、转动不足 0.5 rad、单一曲率、缺少轮轴正反转，
以及滑移/误差或置信度超限均拒绝结果。训练和独立验证均要求相对 RMS ≤8%，
同时限制绝对 RMS ≤0.04 m/s；避免低速时大比例轮径误差被绝对阈值掩盖。
已实现应用/回滚和有限标定动作入口，均默认预览。
应用会重算独立验证，核对候选文件校验和及 hardware 数据属性，先备份再写参数，
要求读回一致；失败时尝试恢复原参数。回滚要求与备份相同的适配器文件及校验和。
实际固件参数接口未知，不能声称已经应用参数。适配器契约为：
`adapter read` 输出包含六轮参数的 JSON；`adapter write` 从标准输入接收同格式 JSON，
必须完成硬件确认后返回。适配器负责单位、持久化和硬件语义，不得用普通文件替代硬件。
实际运动必须显式启用，默认预演；本轮没有发送任何运动命令。

## 本轮软件验证

在干净的 ROS 2 Humble 环境执行标定、时间对齐、激光输入、应用事务和实际 rosbag2 提取测试，此前本机完整执行 41 项通过；新增的真实子进程适配器读回测试单独通过。
NX 此前整包回归为 16 个 CTest 组通过，汇总 91 tests，零错误/失败。
本轮动作拦截新增第 17 组，并通过动作预览与运行时两组窄测试。
统计范围及后端结果见验证记录。
覆盖六轮不同参数恢复、独立验证失败、训练数据重复使用拒绝、校验和篡改拒绝、
未知硬件/外参/时钟拒绝、时间间断及外推拒绝、带行填充的组织点云完整记录复制、
缺失/异常逐点时间拒绝和激光位姿跳变拒绝。
应用事务使用测试替身；子进程契约测试使用临时目录中的模拟适配器，
覆盖写入成功读回一致，以及写入返回成功却未改变参数的拒绝。未调用真实硬件适配器。
这些测试使用合成数据，不证明实车轨迹精度、轮径、零偏、回环、定位或导航成功。

```bash
PYTHONPATH=ros2/tianrover /usr/bin/python3 -m pytest -q \
  ros2/tianrover/test/test_calibration*.py ros2/tianrover/test/test_lidar_frontend.py \
  ros2/tianrover/test/test_map_database.py
```

待确认信息集中为：真实十二关节反馈接口及单位/方向/减速比/时间戳来源；
底盘轮径与转向零偏参数读写接口；雷达到底盘实测外参。缺少反馈时禁止实车标定和应用。

新增录包、提取与应用入口（硬件接口确认后）：

```bash
ros2 run tianrover calibration_bag record --profile profile.yaml --output train_bag --duration 120
ros2 run tianrover calibration_bag extract --profile profile.yaml --bag train_bag --output train
# 另录一次独立验证包，extract 后使用前述 validate 命令。
ros2 run tianrover calibration_parameters apply --candidate candidate/parameters.yaml \
  --report validation/report.json --profile profile.yaml --dataset independent --backup backup.json
# 默认仅预览。实际应用需指定真实适配器，并显式加 --execute。
ros2 run tianrover calibration_parameters rollback --backup backup.json
```

录包 profile 还要求 namespace、feedback_node、laser_odom_frame、base_frame 和
wheel_measurement（velocity_rad_s / continuous_position_rad / wrapped_position_rad）。
录包不发送运动命令；消息中 wheel_joint 与 steering_joint 必须是已转换为模型约定的实测关节。

地图保存与动作入口：

```bash
ros2 launch tianrover map_save_3d.launch.py database:=$HOME/maps/mars3d.db \
  output:=$HOME/maps/mars3d_saved.db
ros2 launch tianrover calibration_motion.launch.py
```

保存入口要求独立激光速度确认停车，核对活动 RTAB-Map 数据库路径，
暂停、调用 backup 服务刷新内存并复制 .back，再校验图节点与 SQLite 完整性。
恢复保存前的暂停状态，不覆盖已有目标地图。上游 backup 会重载内部记忆，
保存期间应保持停车；行为依据 RTAB-Map CoreWrapper.cpp。
定位入口会拒绝损坏、空或非 RTAB-Map 数据库。

标定动作默认只输出 120 秒的直行/倒车/正负曲率计划，不创建 ROS 发布者。
实际执行必须设置 execute:=true、profile:=...、clearance_verified:=true；
profile 还须确认 firmware_command_timeout_s 不大于 0.3 和 planar_twist_verified=true。
执行检查真实关节、激光速度、时间审计、测量时间戳及 cmd_vel 唯一发布者，
发现其他导航/遥控发布者或数据中断即停止；正常退出发送零速度，
进程被强制终止等场景依赖已确认的底盘命令看门狗。
指令由独立单调时钟限制为最多 20 Hz，不依赖 spin_once 的返回频率。
运行前核对 laser_odom_frame/base_frame 与已确认 profile 一致，
雷达时间审计自身的测量时间戳也须在 300 ms 内，重复的关节名称拒绝。
已在隔离 ROS_DOMAIN_ID=188 验证七个运行时场景：过期测量、过期审计、错误坐标系、
竞争控车发布者均在创建控车发布者前拒绝；执行中时间审计无效、反馈中断均退出并发零速度；
有限动作正常完成、指令限频及最终零速度通过。测试计划仅在测试子进程中缩短为 0.5 秒，
产品默认 120 秒计划保持不变。当前尚未执行任何真实动作或验证固件看门狗。
