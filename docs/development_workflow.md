# 单仓库与真机工作区协作流程

## 为什么保留三个目录

- G1 真机工作区：继续承担 G1 构建、仿真和真机验证。
- B2 真机工作区：继续承担 B2/CUDA 构建、实验和真机验证。
- 本 portfolio：只组合经过确认的发布提交，用于 GitHub 展示。

这样做不会改变原工作区路径、`build/`、`install/`、地图或设备配置，也避免两个项目的同名 ROS 包在一个 colcon overlay 中互相覆盖。

## 后续更新顺序

1. 在对应真机工作区创建同目录备份。
2. 修改源码并完成离线检查、构建、仿真或分阶段真机验证。
3. 在对应真机工作区提交 Git。
4. 在 portfolio 根目录拉入新的已验证提交：

```bash
git subtree pull --prefix=g1_navigation <path-to-g1-workspace> main --squash
```

或：

```bash
git subtree pull --prefix=b2_navigation <path-to-b2-workspace> main --squash
```

5. 重新运行两个子项目的源码检查，然后推送 portfolio。

## 约束

- 不要在 portfolio 根目录执行 `colcon build`。
- 不要同时 source G1 与 B2 的 `install/setup.bash`。
- 真机专用地图、rosbag、网卡配置和 `.bak` 继续留在原工作区，不进入 portfolio。
- 如果直接修改 portfolio 子目录，下一次 subtree 同步可能产生冲突；个人开发建议以原真机工作区为准。
