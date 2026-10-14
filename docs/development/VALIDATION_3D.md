# 三维链路验证记录（2026-10-08）

> 以下为拆包前的历史记录；当前包名、构建和验证命令见[迁移说明](MIGRATION.md)。

以下为各阶段的测试记录。静态 H16 链路的最新结果在末尾；
早期低频测量用于记录修复依据，不代表当前状态。所有合成测试均未控制实车。

NX 源码部署前备份：/tmp/tianrover-before-3d-20261008.tar.gz。
构建日志：/tmp/tianrover-3d-frontends-build.log 和 /tmp/tianrover-3d-package-build.log。

KISS 节点集成测试已在 NX 的 ROS_DOMAIN_ID=184 执行通过：
60 帧合成三维点云，输出 60 个纯激光位姿，位移 1.189895 m，
相对合成轨迹最大 X 误差 0.015207 m。日志 /tmp/tianrover_kiss_runtime_7gur5mit/
及 /tmp/tianrover-3d-kiss-runtime.log；测试未启动底盘。
这是实际 KISS 节点对合成扫描的测试，不是实车精度或真实雷达逐点时间验收。
复查脚本：python3 src/tianrover/test/kiss_runtime_check.py。

RTAB-Map 后端与 Nav2 规划集成测试已在 NX ROS_DOMAIN_ID=185 通过：
合成三维场景与激光前端格式的合成里程计输入，65 个图节点、70 条空间回环连接，
保存数据库并重新加载定位，44844 个二维地图栅格、8804 个三维地图点，
Nav2 ComputePathToPose 返回 38 个路径位姿。未发送 NavigateToPose 或控车。
日志 /tmp/tianrover_rtabmap_runtime_xrzb4aly/ 及 /tmp/tianrover-3d-rtabmap-runtime.log。
该测试独立验证后端与规划器，不是 KISS 与 RTAB 的整条真实雷达移动验收，
也未验证 DWB 实际跟踪或目标到达停车。

现场雷达已单独重启到 XYZIRT 版本；原底盘未重启，未发送运动命令。
8 秒观测中 22 帧通过时间检查：x/y/z/intensity FLOAT32、ring UINT16、
timestamp FLOAT64，point_step=26，timestamp 位于偏移18（非对齐布局）。
有效点约27500，扫描跨度约0.09994..0.10012秒；主机 header 与逐点时间通过一致性检查。
此次在后台集成测试负载下采样，不能作为雷达完整输出频率或时间同步精度结论。
新雷达 launch PID=418914，日志 /tmp/tianrover-3d-live-lidar.log。

config/calibration.example.yaml 从当前 URDF 中立姿态生成六轮几何，保存模型 SHA-256；
反馈、时钟、外参、场地等未确认项故意留空/false，默认拒绝标定。
导航三维入口使用 KISS deskew 点云作为局部体素层与全局障碍层输入，
地面点阈值及体素高度只是首版平地配置，仍需实车检查近地障碍与制动距离。

TF authority guard 使用 Humble C++ rclcpp::MessageInfo，
监测目标 odom→base 发布者 GID，并要求唯一 KISS 前端；
在 ROS_DOMAIN_ID=186 的实际节点测试中拒绝旧 wheel 发布者、
停止后恢复、接受 KISS 发布者。日志 /tmp/tianrover-3d-tf-guard-runtime.log。

真实 H16 → KISS shadow 检查：10 秒内收到11个 odom_lidar、18个有效时间审计，
无时间审计错误；未发布 KISS TF，未替换当前轮式 odom。
日志 /tmp/tianrover-3d-live-kiss-inspection.json。
观察到输入/输出频率不足以及速度适配器时间间隔拒绝，正在检查处理成本；
该结果只证明真实点云能产生激光位姿，不构成可用于导航的更新率验收。
检查后的 shadow KISS 已停止，真实雷达继续运行。

NX 包级 colcon build/test/test-result 验证通过：86 tests、0 errors、0 failures、0 skipped。
包含原二维/模型/EKF 回归、标定/地图单测，以及 KISS 和 RTAB-Map/Nav2 的 launch_testing。
TF guard 的额外 launch_testing 窄测试也通过。整包 CTest 为15组全部通过，
补注册后的 TF guard 组也通过；当前 test-result 汇总为74 tests、0 errors、0 failures。
汇总数包含各测试结果文件，受窄测试重写 CTest 汇总影响，不等于唯一测试用例数。
日志 /tmp/tianrover-3d-package-test.log、/tmp/tianrover-3d-package-test-result.log。

实测定位到 PointCloud2 数据 setter 的逐字节校验成本，改用等价 array('B') 赋值，
保留完整记录及额外字段。两次真实 H16 点云观测中，检查函数平均耗时分别
约145.9ms（旧）和5.6ms（新）；这是现场单次函数测量，不是整链路实时保证。
包测试中的 KISS/后端测试也使用了该修改。真实前端更新率仍需重新检查。

真实点云低频问题已定位到 BEST_EFFORT 接收：同一 SDK 发布者、同一10秒窗口，
可靠订阅100帧（10.0005Hz，最大间隔0.101秒），尽力订阅30帧（2.929Hz，最大间隔1.399秒）。
已将检查端和 KISS 输入改为可配置的可靠 QoS；KISS 补丁保存在
patches/kiss-reliable-cloud.patch，本包 config/kiss.yaml 设 reliable_cloud=true。
检查端会拒绝没有可靠 KISS 输入的部署，避免漏应用补丁后继续丢帧。

修正后20秒预热实测：原始点云200帧，KISS/速度输出205帧（窗口边界存在少量在途帧），
按测量时间戳均约10Hz，最大时间间隔0.101001秒，速度输出年龄P95约0.0511秒，
204条有效时间审计、0条错误。证据 /tmp/tianrover-3d-pipeline-reliable-profile.json。
这证明当前静态实车点云链路更新率；未验证运动中的去畸变精度、长期运行或负载极限。
复查：python3 src/tianrover/test/lidar_pipeline_profile.py --seconds 20 --output /tmp/profile.json。

地图保存已通过实际 map_save_3d CLI 集成测试（未控车）：
停车确认、活动数据库校验、backup 服务、保存副本、恢复暂停状态后，
再从保存地图定位并让 Nav2 规划成功，65节点、70条空间回环、39个路径位姿。
日志 /tmp/tianrover_rtabmap_runtime_u5xsovse/ 和 /tmp/tianrover-3d-rtabmap-save-cli-runtime.log。
依赖、源码导入、补丁和 CMake 构建命令见 DEPENDENCIES_3D.md。

## 最终源码回归

本轮 NX `colcon build --packages-select tianrover --symlink-install --cmake-force-configure`
通过；16 个 CTest 组全部通过，`colcon test-result --verbose` 汇总 91 tests、
0 errors、0 failures、0 skipped。汇总包含测试结果文件，不代表 91 个独立场景。
覆盖新的相对误差拒绝、可执行模拟适配器的参数读回、完整三维 bringup 和
单个组合定位导航入口。日志 `/tmp/tianrover-3d-final-build.log`、
`/tmp/tianrover-3d-final-test.log`、`/tmp/tianrover-3d-final-test-result.log`。

本次后端结果：65 节点、70 条 type=2 空间回环连接、44421 个地图栅格、
8804 个三维点、39 个规划路径位姿；保存调用实际 map_save_3d CLI，
定位导航使用 localize_and_navigate_3d 入口。
结果 `/tmp/tianrover_rtabmap_runtime_ya_0tth8/result.json`。

另发现早期失败的 launch 留下隔离域 182/184 中的展示节点，已核对 PID、
domain、namespace 后清理。KISS 测试现在在父进程已退出时也清理进程组，
防止失败后的子进程残留。实车 domain 的底盘、雷达和独立 KISS 不在清理范围内。

真实运动中的激光精度/去畸变、实测外参、十二主动关节反馈、
硬件参数应用回滚、地图定位精度和 DWB 目标跟踪/停止均未验收。
当前反馈与参数接口缺失，软件默认拒绝实车标定及参数应用。

## 标定动作运行时拦截

NX 最新构建通过；`colcon test --packages-select tianrover --ctest-args
-R 'calibration_motion|test_motion_runtime'` 两组通过。新增 test_motion_runtime.launch.py
执行真实 ROS 消息/节点发现测试，使用 domain 188 和模拟关节、激光速度及时间审计，
没有底盘节点。测试缩短计划仅作用于子进程 fixture，不修改产品默认动作。

七个场景：stale_stamp、stale_audit、wrong_frame、competing_publisher
均 exit=2、没有控车消息；timing_fault、feedback_loss 在动作中触发，
分别观察 6/11 条消息、最终均为零；finite_success exit=0、20 条消息
（包含 10 条结束零速度），非零指令数量符合 20 Hz 上限。
此前未限频实现的相同 0.5 秒测试发出 338 条消息，说明 spin_once 超时不构成限频。

结果 `/tmp/tianrover_motion_runtime_qqj20v35/result.json`；日志
`/tmp/tianrover-motion-build.log`、`/tmp/tianrover-motion-test.log`。
本轮窄测试后的 test-result 汇总为 79 tests、零错误/失败，
CTest 汇总文件被窄测试重写，不能将 79 与此前整包 91 解释为用例减少。
真实设备未执行任何标定动作，主动反馈接口、参数应用与物理外参仍缺失。
