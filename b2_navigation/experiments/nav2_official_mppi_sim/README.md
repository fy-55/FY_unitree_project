# Nav2 Humble 官方 MPPI 隔离仿真实验

这个目录只运行 Nav2 自带的 TurtleBot3 Gazebo 世界和官方
`nav2_mppi_controller`。它不读取 B2 参数、不加载 `nav_ws/install`，
也不会启动 `b2_sensor`、`b2_walk` 或 Unitree SDK 节点。

## 1. 一次性安装缺少的 Gazebo 依赖

这一步必须由用户本人输入 sudo 密码：

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3-gazebo
```

安装后先做隔离和依赖检查：

```bash
cd <repository>/experiments/nav2_official_mppi_sim
./scripts/check_isolation.sh
```

看到最后一行 `[READY]` 才进入下一步。

## 2. 启动官方仿真

终端 1：

```bash
cd <repository>/experiments/nav2_official_mppi_sim
./scripts/start_sim.sh
```

脚本会启动 Gazebo 物理仿真、RViz、Nav2 和官方 MPPI 控制器，当前默认同时
显示 Gazebo 与 RViz。Nav2 默认使用独立进程模式，便于在 Humble 中按
`Ctrl+C` 后干净退出。脚本还会阻止同一个 ROS 域里重复启动第二套同名
Nav2 节点，避免 action 和 lifecycle 服务互相串扰。在 RViz 中：

1. 等待 Nav2 节点全部变为 Active。
2. 初始位姿已经按官方出生点 `(-2.0, -0.5, 0.0)` 自动设置，不需要点击
   `2D Pose Estimate`。
3. 点击 `Nav2 Goal`，在地图另一侧设置目标点。
4. 观察机器人运动，以及 MPPI 发布的候选轨迹。

如果只想完全无界面运行：

```bash
NAV2_SIM_HEADLESS=true NAV2_SIM_USE_RVIZ=false ./scripts/start_sim.sh
```

如果 RViz 再次因为两个 OpenGL 窗口同时运行而退出，可以只隐藏 Gazebo
窗口、保留 RViz；Gazebo 物理仿真仍会在后台运行：

```bash
NAV2_SIM_HEADLESS=true ./scripts/start_sim.sh
```

## 3. 记录一组可用于汇报的数据

保持终端 1 运行，再开终端 2：

```bash
cd <repository>/experiments/nav2_official_mppi_sim
./scripts/record_sim.sh
```

然后在 RViz 发一个目标点。机器人到达后按 `Ctrl+C` 停止录包。
数据保存在 `data/mppi_日期_时间/`。

## 4. 它为什么不会碰到真实 B2

- `ROS_DOMAIN_ID=42`：仿真使用独立 DDS 通信域。
- `ROS_LOCALHOST_ONLY=1`：ROS 2 数据只允许在本机回环网卡传输。
- `GAZEBO_MASTER_URI=http://127.0.0.1:11345`：Gazebo 使用独立本地端口。
- 禁用 Gazebo 在线模型库，只读取本机安装的官方 TurtleBot3 模型，避免
  网络失败导致world加载卡住。
- 主动从 ROS、动态库、Python 和可执行文件搜索路径中剔除
  当前仓库的 `install`：即使当前终端以前 source 过 B2 overlay，
  仿真也不会继承它。
- 启动文件只来自 `/opt/ros/humble/share/nav2_bringup`。

这四层隔离的目标是：即使真实 B2 开机并连接在局域网里，仿真发布的
`/cmd_vel` 也不会被真实机器人发现。

## 5. 当前 MPPI 参数关系

- 控制频率：20 Hz，每 0.05 s 重算一次。
- 模型步长：0.05 s，与控制周期相同。
- 每轮候选轨迹：2000 条。
- 每条轨迹：56 步，即预测未来 2.8 s。
- 最大线速度：0.26 m/s，预测最远约 0.73 m。
- 局部代价地图：3 m × 3 m，机器人到边缘有 1.5 m。
- 机器人形状：代价地图使用 `robot_radius: 0.22`，因此
  `CostCritic.consider_footprint: false`，两边都按圆形机器人计算碰撞。

因此当前预测距离没有超出局部代价地图的可见范围。`visualize: true`
只为仿真观察和组会演示；正式测速时应改为 `false`，避免可视化开销影响
MPPI 计算时间。
