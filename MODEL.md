# MarsRover 模型修正（2026-10-08）

## 来源与边界

尺寸、网格、质量和惯性来自 NX 原 SolidWorks 导出文件
`~/tianbot_ws/src/tianbot/tianbot_description/urdf/tianrover_mars.urdf`，修正前 SHA-256 为
`fbec8d78322f8305bcf52a04418305d23068191ce174726cd410e42bee638861`。
该文件与新包修正前的 `urdf/mars.urdf` 一致，原始导出文件保留不改。

参考本机 `tianrover_gazebo/launch/gazebo_fixed.launch` 实际加载的
`urdf/tianrover_mini_continuous.xacro`，以及 `plugins/TianRoverDriver/src/TianRoverDriver.cpp`。
参考仓库 HEAD 为 `96aa630`，但模型和驱动已有现场修改，本次读取的是实际工作树。
不把月球车尺寸、关节原点或 ROS 1 Gazebo 插件复制到火星车。

参考模型提供车体—主摇臂—副摇臂—转向座—车轮的运动结构和关节类型依据；
两车摇臂承载的前/后轮位置存在差异，火星车以其自身 CAD 几何确定父子关系。
参考 Gazebo 模型的中轮转向当前被零范围约束，本次火星车仍保留六个可转向关节，
不照抄这个约束。URDF 修正不是 ROS 2 Gazebo 仿真插件移植。

## 修正内容

| 内容 | 修正与依据 |
| --- | --- |
| 左中轮转向原点 | 原 `BogieLM_` 与左前轮共用原点，左中轮网格及轮轴却带约 -0.567583607 m 的 X 偏移。将关节移到自身轮轴上方，同时补偿 visual、collision、inertial 和子关节坐标；转向不再绕左前轮公转 |
| 右后轮悬架父子关系 | `BogieRB_` 原直接挂在 base_link，而左后轮挂在主摇臂。接回 RightMainRocket_Link，重新计算相对变换，保留零位轮心和网格位置，右后轮随其主摇臂运动 |
| 中立关节坐标系 | 各活动 link 在零位统一 X 前、Y 左、Z 上；CAD 的旋转移到网格和惯性 origin。转向为 `0 0 1`，车轮为 `0 1 0`，摇臂为 `0 1 0` |
| 正方向 | 六个转向关节正角度均为左转 yaw；六个车轮正角度均按前进滚动方向。原左右轮 CAD 正方向相反，不再把其镜像符号当作 ROS 接口约定 |
| 根坐标系 | base_footprint 变为无几何、无惯性的地面投影框架；CAD 原根的实体顶板移入固定的 body_top_link 并挂在车体下，保持原几何和质量 |
| 零力矩限制 | 两个主摇臂原 effort=0，改为参考模型的 10 N·m 名义模型限制；这不是火星车的实测力矩或安全限值 |

保留原关节名称，包括历史名称 `WheelRF`（没有尾部下划线）；保留质量、惯性张量、
网格文件、尺寸、车体高度和雷达/IMU/ZED 的零位外参。惯性 origin 的旋转和位置同步变换，
不是只旋转 STL 而忽略惯性。传感器实际标定仍属于 TODO，不由模型变换推定正确。

## 验证与使用

`test/test_mars_urdf.py` 检查六轮排列、转向轴与轮轴、正负转向时轮心稳定、
悬架父子关系及摇臂运动、虚拟 footprint 和雷达外参。
本次另对照原始模型计算所有 visual/collision/inertial 的世界变换：
零位最大矩阵误差约 `5e-16`，质量、惯性张量与 mesh 几何不变。
使用实际 STL 绘制零位及左中轮转向前后对比，确认修正前偏心摆动、修正后原位转向。

构建后继续使用已有入口：

```bash
source ~/tianrover_ws/install/setup.bash
ros2 launch tianrover description.launch.py
```

`robot_state_publisher` 会按 namespace 添加 frame 前缀。当前 `joint_state_publisher`
提供默认展示姿态；接入真实反馈时，应把硬件角度、符号转换为本模型约定，
不能直接沿用旧 CAD 或月球车插件的左右符号，也不能把默认姿态当作真实关节反馈。

NX 上构建与 `colcon test` 通过，`colcon test-result --verbose` 为 19 tests、0 errors、
0 failures、0 skipped。独立 ROS_DOMAIN_ID=181 的实际 TF 检查使用安装后的模型，
先通过真实 `description.launch.py` 验证零位，再单独向 robot_state_publisher 发布合成关节状态：
六个转向关节 ±0.6 rad、六个轮轴 0.7 rad、左右主摇臂各 0.2 rad 均与运动学计算一致。
最大 TF 矩阵误差约 `2.3e-16`。未连接底盘或发送运动指令，所有测试进程退出后停止。

可复查：`python3 src/tianrover/test/model_runtime_check.py`。
脚本还覆盖 `description.launch.py use_joint_state_publisher:=false`，验证默认零位发布者关闭后，
模型能响应隔离测试域中的合成关节输入。这不是实车反馈验收。
日志：NX `/tmp/tianrover-model-build.log`、`/tmp/tianrover-model-test.log`、
`/tmp/tianrover-model-tf.log` 与 `/tmp/tianrover_model_tf_8ad4pkm9/`。
坐标约定参考 [REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)。

## 实车反馈接入

2026-10-08 在 NX 检查 `/mars/joint_states`：5 秒 51 帧，16 个关节位置全部为零，
发布者为默认 `joint_state_publisher`。后续 bringup 启动后出现里程计，但未出现电机关节位置话题，
并与独立 description 形成两套同名模型节点。

已检查的 NX `tianbot_core` 中，`tianrover` 分支未实现 `JointState` 发布。
DYN 的四电机完整反馈结构虽然有 `output_position_rad`，其 ROS 消息未包含该字段；
紧凑结构本身也没有位置。这不构成六轮 TianRover 的反馈接口，不能按索引直接套用。
是否已有固件上传六轮及转向角度，仍需核对实际固件协议与接收数据。

接入顺序：

1. 核对固件的实际测量位置、更新频率、电机 ID、单位、减速比、零点和方向。
2. 驱动将测量结果映射到本模型关节，发布同一 namespace 的 `joint_states`，位置单位为 rad。
   六轮轴为 `WheelLF_`、`WheelLM_`、`WheelLB_`、`WheelRF`、`WheelRM_`、`WheelRB_`；
   六转向为 `BogieLF_`、`BogieLM_`、`BogieLB_`、`BogieRF_`、`BogieRM_`、`BogieRB_`。
   用户已确认：左右各两个 rocker-bogie 被动关节没有编码器，保留中立展示姿态，
   不把它们列为电机反馈接入的必需项，也不宣称其姿态为实测值。
3. 停止已有 description，启用唯一一套模型节点并设置 `use_joint_state_publisher:=false`，
   避免零位消息与真实数据交替覆盖。
4. 分别验证关节角与动态 TF，以及独立的 `odom → base_footprint` 位姿 TF。

不得将 `cmd_vel` 积分或控制目标角度冒充实际测量位置。缺失真实反馈时，本参数只能准备接入路径，
不能让实车模型准确反映电机运动。

本次接入准备已在本机和 NX 的隔离域验证：关闭默认发布者后，六转向、六轮轴和左右主摇臂的
合成位置均正确更新 TF，最大误差约 `2.2e-16`。NX 构建通过，包测试 19 tests、0 errors、
0 failures、0 skipped；日志为 `/tmp/tianrover-feedback-build.log`、`/tmp/tianrover-feedback-tf.log`、
`/tmp/tianrover-feedback-test-result.log`。

现场已停止重复的独立 description，并将 RViz 切到 `mars/odom`，未重启底盘驱动。
再次采样 5 秒：1 个关节发布者、45 帧全零位置；260 帧里程计与 258 帧
`mars/odom → mars/base_footprint` TF。保留唯一的展示发布者，等待真实反馈接入后关闭。
主动轮轴和转向的真实显示仍需确认反馈能力、协议及电机到关节的对应关系。
用户确认旧版 Gazebo 会提供关节状态；仿真中的模型运动不能证明实车驱动具有位置上传接口。
目前已检查的实车驱动缺少该接口，因此缺失数据不是 URDF 本身不响应。
将来只发布十二个主动关节时，还需由展示节点单独维持四个被动关节的中立 TF；
不能直接关闭全部展示发布者后遗漏被动关节，也不能让全零展示消息覆盖主动反馈。

## Git 与回退

NX 的 `~/tianrover_ws/src/tianrover` 已初始化为独立 main 分支仓库。
先提交修正前的整个可用包为 `c42034c`，再提交模型修改。未配置或推送任何 GitHub 远端。
NX 未配置 Git 作者，本次提交显式使用本地 `Tianbot Deployment <tianbot@localhost>`，
未修改用户全局身份设置。

如需只回退模型，可在确认当前工作区改动后执行
`git restore --source=c42034c -- urdf/mars.urdf`，再重建；不使用 reset 丢弃后续包改动。
原 `~/tianbot_ws` 工作树及其已有改动保持不变。
