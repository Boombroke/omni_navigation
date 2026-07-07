# rmu_gazebo_simulator

Gazebo Harmonic (gz-sim8) 仿真环境，为 Sentry26 导航栈提供与实车**同链路**的仿真验证平台。

---

## 1. 功能概述

- **仿真世界**：`rmuc_2025`、`rmuc_2026`、`rmul_2026` 三张比赛场地。
- **GT 定位中继**：`chassis_odom_relay.py` 用 Gazebo 真值里程计替代实车 Point-LIO，提供精确 `odom→base_footprint` TF 及 `chassis_odometry`（供 MPPI 速度反馈），解决仿真 Point-LIO 漂移问题。
- **仿真裁判发布器**：`sim_referee_publisher.py` 定时发布 `rm_interfaces` 裁判消息（`game_progress=4`），驱动 `sentry_behavior` 状态机在仿真中运行。
- **导航链路验证**：已验证 MPPI Omni 在 rmuc_2026 世界可复现自主导航（GT 位移 ~2.4m，零 `Optimizer fail`）；`enable_behavior:=true` 时状态机闭环驱动机器人朝守点 (3.71,-0.61) 移动。

---

## 2. 环境要求

| 依赖 | 版本 |
|------|------|
| Ubuntu | 24.04 LTS |
| ROS2 | Jazzy |
| Gazebo | Harmonic (gz-sim8) |
| ros_gz | jazzy 对应版本 |

### 安装 Gazebo Harmonic + ros_gz

```bash
sudo apt install -y gz-harmonic
sudo apt install -y \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-interfaces
```

或直接运行：

```bash
bash src/scripts/setup_env.sh   # install_sim_deps 函数自动完成以上安装
```

---

## 3. 快速启动

### 3.1 一键仿真（推荐）

通过 `sentry_nav_bringup` 的顶层 launch 直接启动完整仿真栈（Gazebo + Nav2 + 可选状态机）：

```bash
# 无头模式（不启动 Gazebo GUI，省显卡资源）
ros2 launch sentry_nav_bringup rm_simulation_all_launch.py headless:=true

# 带 RViz 图形界面
ros2 launch sentry_nav_bringup rm_simulation_all_launch.py world:=rmuc_2026

# 开启状态机决策（守点自主导航闭环）
ros2 launch sentry_nav_bringup rm_simulation_all_launch.py headless:=true enable_behavior:=true

# 多机器人仿真
ros2 launch sentry_nav_bringup rm_multi_navigation_simulation_launch.py
```

> **提示**：仿真 launch 自动启动 `chassis_odom_relay.py`（关闭 `odom_bridge`）并等待 `nav_delay`（默认 15s）后启动 Nav2，无需手动 unpause Gazebo。

### 3.2 launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `headless` | `false` | 无头模式（不启动 Gazebo GUI） |
| `world` | `rmuc_2026` | 仿真世界（`rmuc_2025` / `rmuc_2026` / `rmul_2026`） |
| `slam` | `True` | 启动 SLAM（`slam_toolbox`，为 static_layer 提供 `/map`） |
| `nav_delay` | `15.0` | 物理稳定后延迟启动 Nav2 的秒数 |
| `enable_behavior` | `false` | 同时启动 `sentry_behavior` 和 `sim_referee_publisher` |

---

## 4. 仿真定位中继（chassis_odom_relay.py）

**位置**：`rmu_gazebo_simulator/scripts/nav/chassis_odom_relay.py`

**解决的问题**：仿真 Point-LIO 位姿抖动 ~8mm/帧，`chassis_odometry` 静止时出现 ±0.1m/s 幽灵速度，导致 MPPI 反馈失真。GT 真值 (`chassis_odometry_gt`) 精确无噪声。

**工作原理**：

1. 订阅 `chassis_odometry_gt`（Gazebo `/gz/mux/model/red_standard_robot1/...` 桥接，1000Hz）。
2. 以第一帧为 init，后续输出 `rel = init⁻¹ · gt`（spawn 相对位姿），消除绝对坐标偏置。
3. 广播 `odom→base_footprint` TF（替代 `odom_bridge`）。
4. 发布 `odometry`（`odom→gimbal_yaw` 系，供 `fake_vel_transform` 读姿态）。
5. 发布 `chassis_odometry`（`odom` 系，`child=base_footprint`，twist 线速度在惯性轴，供 MPPI 速度反馈）。
6. 发布 `registered_scan` + `lidar_odometry`（`terrain_analysis`/`terrain_analysis_ext` 链路必需；无则 slam 无 `/map` → `static_layer` 阻塞）。`registered_scan` 由**原始仿真雷达 `velodyne_points`（sensor 系）经 GT 的 `odom→sensor` TF 真变换到 odom** 得到，与 GT 机器人位姿严格一致、免疫 sim Point-LIO 漂移（旧实现仅把 Point-LIO `cloud_registered` 的 `camera_init` 帧改标为 odom、不变换坐标，机器人运动后点云随 Point-LIO 漂移偏离 GT，导致 RViz 点云与机器人/地图错位）。

**与 odom_bridge 的关系**：

- `navigation_simulation_launch.py` 传递 `enable_odom_bridge:=False` 关闭 `odom_bridge`，启动 `chassis_odom_relay.py`。
- 实车 launch 默认 `enable_odom_bridge:=True`，行为完全不变。
- 两路**不能同时运行**（会产生冲突的 `odom→base_footprint` TF 广播）。

---

## 5. 仿真裁判发布器（sim_referee_publisher.py）

**位置**：`rmu_gazebo_simulator/scripts/nav/sim_referee_publisher.py`

**功能**：定时（1Hz）发布以下 `rm_interfaces` 消息，使 `sentry_behavior` 状态机进入 `IN_MATCH` 并执行战术决策：

| 话题 | 内容 |
|------|------|
| `referee/game_status` | `game_progress=4`，`stage_remain_time=420` |
| `referee/robot_status` | `remain_hp=400`，`ammo_count=200` |
| `referee/all_robot_hp` | 红方全员满血（500/500） |

`rm_simulation_all_launch.py` 在 `enable_behavior:=true` 时同时启动此脚本和 `sentry_behavior_launch.py`。

---

## 6. 仿真世界说明

| 世界名 | 说明 |
|--------|------|
| `rmuc_2025` | RMUC 2025 赛季标准场地 |
| `rmuc_2026` | RMUC 2026 赛季标准场地（**默认，已验证导航**） |
| `rmul_2026` | RMUL 2026 小场地 |

世界文件位于 `rmu_gazebo_simulator/worlds/`。

### 6.1 物理稳定性修复（rmuc_2026）

**问题**：机器人静置约 60s 后 GT z 坐标发散至 `-∞`（-56000+），roll 翻转——纯 Gazebo 物理不稳，与导航无关。根因：场地地面为单块大 STL 三角网（`rmuc_2026.stl`），世界 SDF 原无 `<physics>` 块；细圆柱轮（r≈0.076m）与大 trimesh 接触时，默认 DART LCP 解算发散。

**修复（commit `8813da8`）**：

1. 新增 `<physics name="sim" type="dart">` 块，`max_step_size=0.001`（缩小步长，稳定 LCP 迭代）。
2. 新增解析地平面模型 `flat_ground`（`<plane>` 法向 `(0,0,1)`，尺寸 100×100m），摩擦 `mu=mu2=0.9`——平面-圆柱接触精确稳定，作主接触面；原 STL 保留（提供墙/坡碰撞与感知），不影响 costmap。
3. 机器人 spawn 置于红方基地 `(-11.3, 1.3)`、z=**0.5**。关键:整场 STL 地面 floor z≥0.228(红方基地平台 ≈0.43m),spawn z 必须高于当地地形面,否则底盘陷入 STL 下方(表现为"卡在地下");`flat_ground`(z=0)仅是物理稳定接触面,非落点基准。(注:8813da8 曾试过中场 `(0,8)`+z=0.05,会卡地下,已修正回基地。)

**轮摩擦说明**：车轮 `mu=0.9` 定义在 `chassis_wheel.def.xmacro`（生效文件）。`rmua19_standard_robot/model.sdf` 中的 `mu=0.2` 是 xmacro 自动生成的死文件，**不生效，无需修改**。

**验证**：headless 无头实测,gz 模型位姿在基地 `(-11.3, 1.3)` 稳定落地 z≈**0.503**、RPY≈0(≈基地平台 0.428 + 轮半径 0.076,四轮平稳接触),Nav2 全激活、SmacPlanner2D 从基地成功出路径、无 `Start occupied`。

---

## 7. 已知局限（诚实记录）

| 局限 | 说明 |
|------|------|
| 地形避障不覆盖 | `IntensityVoxelLayer.min_obstacle_intensity: 100.0` 中和地形直标，仿真障碍仅来自静态地图 |
| MPPI 从静止保守 | Gazebo MecanumDrive2 用 `AddWorldWrench`（非标准摩擦），MPPI 从 GT 静止反馈出发保守，近目标（2.5m）约 40s；远目标可能在 patience 内推进不完全 |
| slam_toolbox bond 超时 | Gazebo `/clock` 初始为 0 导致 bond 超时，日志有告警，不阻塞导航 |
| 启动竞态 | `enable_behavior:=true` 时行为节点可能在 `bt_navigator` 激活前发目标被拒绝；Nav2 激活后才稳定 |
| 无实车传感器 | 无 Livox 真实驱动、无串口通信、不测试 Point-LIO 实际表现 |

---

## 8. 进程清理

仿真进程较多，退出后如有残留可用以下命令清理：

```bash
# Gazebo server
pkill -9 ruby

# 其余 ROS 节点（按 comm 名）
pkill -9 component_conta
pkill -9 sync_slam_toolb
pkill -9 terrainAnalysis
pkill -9 robot_state_pub
pkill -9 parameter_bridg
pkill -9 sentry_behavior

# chassis_odom_relay 是 Python 节点，comm = python3，用 -f 模式
pkill -9 -f 'chassis_odom_relay[.]py'
pkill -9 -f 'sim_referee_pub[l]isher[.]py'
```

> **不要用 `pkill -f <含字面串的模式>`**，会命中正在执行的 bash 命令行自身导致自杀。

---

## 维护者

boombroke <2218681402@qq.com>

本包基于 [SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator) 适配 Gazebo Harmonic + ROS2 Jazzy + Sentry26 导航架构。License: Apache-2.0
