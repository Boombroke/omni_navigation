# 快速部署与上手指南

> 如需了解系统各模块的详细架构设计，请参阅 [系统架构详解](ARCHITECTURE.md)。

## 0. 一键配置（推荐）
如果你使用全新的 Ubuntu 24.04 系统，可以直接运行一键配置脚本完成所有环境安装和编译：
```bash
bash src/scripts/setup_env.sh
```

该脚本会依次执行以下步骤：
1. **安装 ROS2 Jazzy**: 添加官方 APT 源，安装 `ros-jazzy-desktop` 和 `ros-dev-tools`
2. **安装系统依赖**: Eigen3、OpenMP、PCL、Nav2、SLAM Toolbox、serial-driver 等
3. **编译安装 small_gicp v1.0.0**: 从 GitHub 克隆并编译（需要 C++17）
4. **初始化 rosdep**: 配置 ROS 包依赖管理
5. **创建工作空间并编译**: 在 `~/sentry_ws` 下创建工作空间并执行 `colcon build`
6. **配置 bashrc（可选）**: 询问是否将工作空间环境写入 `~/.bashrc`

如果你希望手动逐步配置，请继续阅读以下章节。

## 1. 环境要求
- Ubuntu 24.04
- ROS2 Jazzy

## 2. Docker 部署（可选）
Docker 是快速体验本项目的一种方式。

### Docker 安装
请参考 Docker 官方文档安装 Docker Engine 和 Docker Compose。

### 允许 Docker 访问显示器
在宿主机执行：
```bash
xhost +local:docker
```

### 运行容器
```bash
docker run -it --rm \
  --name sentry_nav_container \
  --net=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  sentry_nav:latest
```

> 注意：镜像名需根据实际构建的标签更新。

## 3. 源码编译部署

### 3.1 安装依赖
本项目依赖 small_gicp 进行点云配准，请先编译安装：
```bash
sudo apt install -y libeigen3-dev libomp-dev
git clone https://github.com/koide3/small_gicp.git
cd small_gicp && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

### 3.2 安装 ROS 依赖
在仓库根目录执行：
```bash
source /opt/ros/jazzy/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

### 3.3 编译

> **OOM 预防（重要）**：`btcpp_ros2_interfaces` / `rm_interfaces` 等 IDL 包 Python binding 内存占用大。全量并行在 16G 内存机器上易 OOM。推荐加 `--parallel-workers 4`，或内存极紧时用 `--executor sequential`。

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4
source install/setup.bash
```

`--symlink-install` 选项让 YAML/XML 参数文件修改后无需重新编译，直接生效。

单包编译（调试用）：
```bash
colcon build --packages-select sentry_behavior --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

如果编译出错，`src/scripts/` 下有常见问题修复脚本（`fix_nav2_deps.sh`、`fix_libusb_pcl.sh` 等）。

## 4. 实车模式

### 前置条件
- Livox MID360 激光雷达已连接并配置网络（默认 LiDAR IP: `192.168.1.150`，主机 IP: `192.168.1.50`）
- 串口设备已连接（默认 `/dev/ttyACM0`），已授权：`sudo chmod 666 /dev/ttyACM0`
- 先验点云（PCD）和 2D 地图已放置到 `sentry_nav_bringup/pcd/reality/` 和 `map/reality/`

### 4.0 一键启动（推荐）
```bash
ros2 launch sentry_nav_bringup rm_sentry_launch.py
```
该命令同时启动串口驱动、Livox 驱动、Point-LIO、导航栈，以及（可选）状态机决策和录包。

关键参数：
```bash
# 不启动录包
ros2 launch sentry_nav_bringup rm_sentry_launch.py enable_recorder:=False

# 启动状态机决策
ros2 launch sentry_nav_bringup rm_sentry_launch.py enable_behavior:=True strategy:=rmuc_defend

# 指定地图/PCD 名称（world 参数对应 map/reality/<world>.yaml 和 pcd/reality/<world>.pcd）
ros2 launch sentry_nav_bringup rm_sentry_launch.py world:=rmul_2026
```

### 4.1 建图
```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True
```

### 4.2 导航（已有先验地图）
```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py \
  world:=<YOUR_WORLD_NAME> \
  slam:=False \
  use_robot_state_pub:=True
```

> 实车 launch 的 `world` 默认值是 `204`（数字，对应实验室地图命名），需根据实际地图文件名覆盖。

## 5. 仿真模式 (Gazebo Harmonic)

仿真使用 **Gazebo Harmonic (gz-sim8)**，通过 `ros_gz_bridge` 桥接 ROS2 话题。仿真与实车运行**相同的 MPPI Omni 控制链路**（MPPI + fake_vel_transform + chassis_odometry 契约），无需激光雷达或串口硬件。

### 5.0 仿真依赖

```bash
# Gazebo Harmonic（如未随 ROS2 Jazzy 自动安装）
sudo apt install -y gz-harmonic

# ros_gz 桥接包（Jazzy 对应版本）
sudo apt install -y \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-interfaces
```

`setup_env.sh` 中的 `install_sim_deps` 函数会自动安装以上依赖（见 `src/scripts/setup_env.sh`）。

### 5.1 一键启动仿真

```bash
# 无头模式（无需显示器/GPU，推荐调试与 CI）
ros2 launch sentry_nav_bringup rm_simulation_all_launch.py headless:=true

# 带 RViz 图形界面（需显卡驱动与 X11）
ros2 launch sentry_nav_bringup rm_simulation_all_launch.py world:=rmuc_2026

# 开启状态机决策（守点自主导航闭环）
ros2 launch sentry_nav_bringup rm_simulation_all_launch.py headless:=true enable_behavior:=true
```

仿真 launch 参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `headless` | `false` | 无头模式（不启动 Gazebo GUI） |
| `world` | `rmuc_2026` | 仿真世界（`rmuc_2025` / `rmuc_2026` / `rmul_2026`） |
| `slam` | `True` | SLAM 建图（仿真用 slam_toolbox 提供静态地图） |
| `nav_delay` | `15.0` | Gz 物理稳定后再启动 Nav2 的延迟秒数 |
| `enable_behavior` | `false` | 是否启动 sentry_behavior 状态机决策 |

### 5.2 仿真定位机制

仿真中 **Point-LIO 被旁路**，由 `chassis_odom_relay.py`（`rmu_gazebo_simulator` 包）替代：

- 订阅 Gazebo 真值里程计 `chassis_odometry_gt`（精确 1000Hz，无噪声）。
- 以首帧为原点输出 spawn 相对位姿，广播 `odom→base_footprint` TF。
- 复刻 `odom_bridge` 契约，发布 `odometry`、`chassis_odometry`（供 MPPI 速度反馈）、`registered_scan`、`lidar_odometry`（供 terrain 链路）。
- `navigation_simulation_launch.py` 通过 `enable_odom_bridge:=False` 关闭 `odom_bridge`，仅启动 `chassis_odom_relay.py`。实车 launch 默认 `enable_odom_bridge:=True`，行为不变。

`sim_referee_publisher.py` 定时发布 `game_progress=4` 等裁判消息，驱动 `sentry_behavior` 在仿真中运行战术逻辑。

### 5.3 验证仿真导航

启动后向 `/goal_pose` 发一个目标，观察机器人实际移动：

```bash
# 等 Nav2 完全激活（约 20~30s）后发目标
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'

# 读 Gazebo 真值位姿（确认机器人实际移动）
gz model -m red_standard_robot1 -p
```

预期：机器人从初始位置出发，GT 位移应 > 2m，路径平滑，无 `Optimizer fail`。

### 5.4 仿真已知局限（请诚实对待）

| 局限 | 说明 |
|------|------|
| 地形避障不覆盖 | `IntensityVoxelLayer` 的 `min_obstacle_intensity` 设为远超最大值（永不触发），仿真依赖静态地图作为唯一障碍源；实车地形感知不会在仿真中被测试 |
| MPPI 从静止保守 | cmd_vel 间歇（0.09↔0），到近目标（2.5m）约 40s；较远目标（4m+）在观测窗内可能只推进部分 |
| slam_toolbox bond 超时 | 日志中出现但不阻塞导航，静态地图已由 relay 透传 `/map` 兜底 |
| 启动竞态 | `enable_behavior:=true` 时，若 `bt_navigator` 尚未激活就收到 `/goal_pose` 会被拒绝；Nav2 完全激活后行为节点才稳定驱动 |
| 无实时传感器 | 仿真无 Livox 真实点云；LiDAR 数据来自 Gazebo 的 velodyne 仿真，Point-LIO 被旁路 |

## 6. 状态机决策（独立启动）
```bash
ros2 launch sentry_behavior sentry_behavior_launch.py
```

或在实车 launch 时通过 `enable_behavior:=True` 自动延迟 8 秒启动（见 4.0 节）。

## 7. 比赛录制与回放

`sentry_match_recorder` 在比赛进入 `game_progress=4` 上升沿自动启动 `ros2 bag record`，下降沿自动停止，单场所有切片归到一个目录。`rm_sentry_launch.py` 默认带录包，临时关用 `enable_recorder:=False`。

### 7.1 自动录制

```bash
# 默认带录包，比赛开始即自动录到 logs/match-bags/sortie_<TS>/
ros2 launch sentry_nav_bringup rm_sentry_launch.py
# 临时禁用录包
ros2 launch sentry_nav_bringup rm_sentry_launch.py enable_recorder:=False
```

输出目录结构（rosbag2 `--max-bag-duration` 切片）：
```
logs/match-bags/
└── sortie_20260521_143012/
    ├── metadata.yaml
    ├── sortie_20260521_143012_0.mcap   # 0~60s
    ├── sortie_20260521_143012_1.mcap   # 60~120s
    └── ...
```

### 7.2 整场连续回放

```bash
ros2 bag play logs/match-bags/sortie_20260521_143012
```

### 7.3 切片合并到单文件

```bash
ros2 run sentry_match_recorder merge_sortie logs/match-bags/sortie_20260521_143012

# 合并后删除原切片
ros2 run sentry_match_recorder merge_sortie <SORTIE_DIR> --remove-shards

# 仅打印清单，不写文件
ros2 run sentry_match_recorder merge_sortie <SORTIE_DIR> --dry-run
```

## 8. 常见问题

| 现象 | 解决 |
|------|------|
| 编译失败 | 确认 small_gicp (>= v1.0.0) 已安装；检查 `src/scripts/fix_*.sh` |
| 编译 OOM | 加 `--parallel-workers 4` 或 `--executor sequential` |
| 先验点云缺失 | PCD 文件体积较大未入仓，需自行准备并放置到正确路径 |
| 重定位后位置异常 | 确认先验 PCD 与 2D 地图使用相同坐标原点（同一次建图产生）|
| 仿真机器人不动 | 检查 Nav2 是否完全激活（等 nav_delay 时间后才启动），确认无 `Start occupied` 日志 |
| 仿真 slam_toolbox bond 超时 | 预期行为，不影响导航；静态地图已由 chassis_odom_relay 兜底 |
