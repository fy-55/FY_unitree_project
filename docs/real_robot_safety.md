# B2 真机安全流程

## 阶段 0：物理条件

- 场地无人员、台阶、玻璃和易倒物，现场监护人与实体急停已就位。
- 首次测试使用低速、单目标、短距离，不直接执行多点路线。
- `scripts/start_b2_navigation.sh` 不带 `--walk`，先证明只读与规划链。

## 阶段 1：只读数据

```bash
./scripts/start_b2_navigation.sh --map-base=/absolute/path/to/map
ros2 topic hz /odom
ros2 topic hz /converted_scan
ros2 run tf2_ros tf2_echo odom base
```

成功标准：输入持续、TF 唯一、Nav2 可生成路径、机器人不动。

## 阶段 2：运动桥 dry-run

单独启动桥，但保持默认关闭：

```bash
ros2 run b2_driver b2_walk --ros-args \
  -p cmd_vel_topic:=/cmd_vel \
  -p enable_motion:=false
```

日志应显示 `NO-MOTION MODE`，并打印限幅后的诊断速度，不调用 `Move()`。

## 阶段 3：显式运动

只有前两阶段通过后，才运行：

```bash
./scripts/start_b2_navigation.sh --map-base=/absolute/path/to/map --walk
```

该选项会同时满足“启动运动进程”和“设置 `enable_motion=true`”两个条件。首次测试前还应按场地进一步降低 `max_linear_speed_x/y` 与 `max_angular_speed`。

立即停止条件：激光或里程计中断、TF 跳变、非预期横移、持续振荡、命令超时未停止、网络异常或监护人失去急停控制。
