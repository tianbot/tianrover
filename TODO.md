# 后续工作

- [ ] 使用现场真实行驶数据验证二维地图、回环、保存地图、AMCL 定位和目标到达停车。
- [ ] 实测 MarsRover footprint、制动距离、最低可控速度及角速度，修订部署参数。
- [ ] 基于 RoboSense 三维点云增加独立三维 SLAM/定位适配器；保留二维入口。
- [ ] 标定雷达、IMU、ZED 外参并记录依据；本体关节模型已修正，传感器零位外参仍沿用 CAD，见 MODEL.md。
- [ ] 接入真实关节反馈；当前 joint_state_publisher 仅提供默认展示姿态。
- [ ] Tianbot Studio 编辑部署环境文件和 YAML，复用统一配置契约。
- [ ] 在 ROS 2 Jazzy / Ubuntu 24.04 上加入独立 CI，验证兼容性后再迁移 NX。
- [ ] 独立发布 Mars 网格资源，补全底盘/模型/rs_driver 的 .repos 锁定；当前明确复用现场 underlay，雷达兼容补丁已保存。
