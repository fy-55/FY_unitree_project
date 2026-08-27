# 真机分阶段验证

这不是“一键真机运行”说明。每一阶段只在上一阶段通过后进行，任何异常都停止，不跨级。

## 阶段 0：物理条件

- 机器人由现场人员监护，周围无人、无台阶、无玻璃和易倒物。
- 实体急停或厂家遥控停止路径已验证，不能把键盘当急停。
- 使用厂家支持的高层运动模式，不发送关节力矩。
- 首次测试保留默认 `0.10 m/s` 前向和 `0.20 rad/s` 偏航上限。

## 阶段 1：只读输入

目的：证明网络、消息类型和时间域正确，不启动速度桥。

```bash
ros2 topic type /lf/lowstate
ros2 topic type /odommodestate
ros2 topic type /utlidar/cloud_livox_mid360
ros2 launch g1_nav_bringup real_robot_pre_slam.launch.py
```

成功标准：`/joint_states`、`/odom`、`/scan` 持续发布，`odom -> base_footprint` 可查，且机器人不动。

停止条件：DDS 接口错误、时间跳变、TF 重复发布、点云/里程计中断或机器人出现意外动作。

## 阶段 2：Nav2 无运动

目的：验证定位、规划和安全速度链，但不发布 Unitree API。

```bash
ros2 launch g1_nav_control velocity_bridge.launch.py enable_motion:=false
```

发送目标后，`/cmd_vel_g1` 可以显示最终限幅速度，但 `/api/sport/request` 不应由速度桥发布。验证命令：

```bash
ros2 topic echo /cmd_vel_g1
ros2 topic info /api/sport/request --verbose
```

## 阶段 3：短时接口试验

只有阶段 0–2 全部通过，且现场负责人同意后，才显式设置 `enable_motion:=true`。使用空旷场地、单个短目标和物理急停；先测直线启停，再测原地转向，不直接测试完整路线。

成功标准必须包含：

- 超时或遮挡激光时输出归零。
- Ctrl+C 后发送零速度。
- 真实刹停距离小于重新标定的 Collision Monitor 停止区余量。
- 速度和角速度没有超过配置上限。

若无法记录以上证据，只能描述为接口试验，不能描述为完整真机自主导航验证。
