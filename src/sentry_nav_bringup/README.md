# sentry_nav_bringup

哨兵机器人导航总入口包。汇集所有 launch 脚本、Nav2 参数、地图、PCD、RViz 配置与 Nav2 行为树。

## 设计

omni 主线为差异点：

- **底盘**：四麦轮 (`MecanumDrive2`)，可任意方向平移
- **base frame**：Nav2 跑在 `gimbal_yaw_fake` 系——`gimbal_yaw_fake` 与真实 `gimbal_yaw` 反向旋转抵消底盘自旋，让 planner 看到一个稳定惯性系
- **执行端**：`fake_vel_transform` 把 Nav2 输出从 fake 系旋回真实系并叠加 `spin_speed`，再发给 Gazebo MecanumDrive 插件 / `rm_serial_driver`
- **Local controller**：`omni_pid_pursuit_controller::OmniPidPursuitController`（仿真）/ `pb_omni_pid_pursuit_controller::OmniPidPursuitController`（实车，迁移中）；不用 RPP / MPPI
- **决策**：`sentry_behavior` 走 BT.CPP 4.9，目标点既可走 Nav2 action 也可直接 publish `/goal_pose`

## Launch 入口

| 脚本 | 用途 | 关键参数 |
|---|---|---|
| `rm_navigation_simulation_launch.py` | 仿真主入口（第二终端，等 Gazebo 就绪后启动） | `world` (`rmuc_2025`/`rmuc_2026`/`rmul_2026`), `slam` (`True`/`False`), `map`, `params_file`, `use_rviz` |
| `rm_navigation_reality_launch.py` | 实车主入口（不含串口） | `slam`, `world`, `use_robot_state_pub` |
| `rm_sentry_launch.py` | 实车一键（导航 + `rm_serial_driver`） | 同上 |
| `rm_multi_navigation_simulation_launch.py` | 多机仿真 | `robots:=[{name,x,y,...}]` 列表 |
| `rm_simulation_all_launch.py` | 仿真全栈（Gazebo + 导航） | 时序敏感，**不推荐**，请用两步法 |
| `bringup_launch.py` | 内部 nav2 一键，被上面的脚本 include |
| `slam_launch.py` | slam_toolbox |
| `localization_launch.py` | small_gicp 重定位 + Point-LIO 桥接 |
| `navigation_launch.py` | Nav2 lifecycle 节点群 |
| `robot_state_publisher_launch.py` | URDF + TF |
| `rviz_launch.py` | 调试可视化 |

### 仿真两步启动（唯一可靠方式）

时序敏感，不支持一键。

```bash
# 终端 1：Gazebo
QT_QPA_PLATFORM=xcb ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
# unpause（Wayland 下 Play 按钮经常失灵）：
gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: false'
# 等 ~10s 让传感器稳定

# 终端 2：导航
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2026 slam:=True
```

首跑用 `slam:=True` 实时建图；建好图后存入 `map/simulation/<world>/` 与 `pcd/simulation/<world>/`，再启 `slam:=False` 走重定位。

### 实车

```bash
# 建图
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True

# 导航（已有图）
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py world:=<NAME> slam:=False use_robot_state_pub:=True

# 一键（导航 + 串口）
ros2 launch sentry_nav_bringup rm_sentry_launch.py
```

## 配置目录

```
config/
├── simulation/nav2_params.yaml      仿真 Nav2 配置 (controller_frequency=20Hz)
└── reality/
    ├── nav2_params.yaml              实车 Nav2 配置 (controller_frequency=30Hz)
    └── mid360_user_config.json       Livox Mid360 驱动 user config

map/
├── simulation/<world>/               仿真 2D 地图 (.pgm + .yaml)
└── reality/<world>/                  实车 2D 地图

pcd/
├── simulation/<world>/               仿真先验点云 (.pcd, 用于 small_gicp 重定位)
└── reality/<world>/                  实车先验点云 (大文件未入仓, 需手动同步)

rviz/nav2_default_view.rviz          调试可视化布局

behavior_trees/                      Nav2 自带 BT (与 sentry_behavior 战术 BT 不同)
├── navigate_to_pose_w_replanning_and_recovery.xml
└── navigate_through_poses_w_replanning_and_recovery.xml
```

## 关键参数（仿真 vs 实车）

| 参数 | 仿真 | 实车 | 备注 |
|---|---|---|---|
| `controller_frequency` | 20 Hz | 30 Hz | **必须** == `smoothing_frequency` |
| `smoothing_frequency` | 20 Hz | 30 Hz | 见上 |
| `robot_base_frame` (controller / costmap) | `gimbal_yaw_fake` | `gimbal_yaw_fake` | omni 主线全栈走 fake 系 |
| `robot_radius` (local costmap) | 0.20 m | 0.30 m | 实车更宽留余量 |
| `controller_plugins.FollowPath.plugin` | `omni_pid_pursuit_controller::OmniPidPursuitController` | `pb_omni_pid_pursuit_controller::OmniPidPursuitController` | 实车端命名空间迁移中 |
| `velocity_smoother.max_velocity` | `[2.5, 2.5, 3.0]` | `[1.5, 1.5, 3.0]` | `[vx, vy, wz]`，omni 必须 vy ≠ 0 |
| `enable_periodic_relocalization` | `true` | `true` | **必须**开，否则 small_gicp 不持续纠偏 |
| `enable_stamped_cmd_vel` | `true` | `true` | controller / behavior / smoother 三处必须一致 |

详细推导见 [`src/docs/TUNING_GUIDE.md`](../docs/TUNING_GUIDE.md)。

## 速度指令链路

```
controller_server (gimbal_yaw_fake 系, TwistStamped)
  → cmd_vel_controller
    → velocity_smoother (TwistStamped)
      → /cmd_vel_nav2_result (≈world 系)
        → fake_vel_transform (旋回 gimbal_yaw 系 + 叠加 spin_speed)
          → /cmd_vel (Twist)
            → Gazebo MecanumDrive2 插件 / rm_serial_driver
```

## 订阅 / 发布话题

订阅：
- `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`) — RViz 重定位输入
- `/goal_pose` (`geometry_msgs/PoseStamped`) — `sentry_behavior::PubGoal` 或 RViz 发的目标点
- 各 nav2 内部 action: `/navigate_to_pose`, `/follow_path` 等

发布：
- `/tf`, `/tf_static`
- `/map` (静态地图，slam=False 时由 map_server 发)
- `/global_costmap/costmap`, `/local_costmap/costmap`
- `/plan` (global path), `/local_plan`

## 关键约束（不可逾越的）

- `controller_frequency` ≡ `smoothing_frequency`，频率不能超过 CPU 实跑能力
- `enable_periodic_relocalization: true` 必须开
- `enable_stamped_cmd_vel: true` 三处一致（controller_server / behavior_server / velocity_smoother）
- 仿真启动顺序：先 Gazebo 并 unpause → 等稳 → 起导航
- PCD + 2D 地图必须**在同一坐标系（odom 系）建图**，从同一出生点
- 仿真渲染：`gui.config` 用 `ogre`，SDF `<render_engine>` 用 `ogre2`（gpu_lidar 只在 ogre2 下工作）
- 仿真世界白名单：`rmuc_2025` / `rmuc_2026` / `rmul_2026`（其他已删除）

## 相关文档

- [系统架构详解](../docs/ARCHITECTURE.md)
- [快速部署](../docs/QUICKSTART.md)
- [运行模式说明](../docs/RUNNING_MODES.md)
- [参数调优](../docs/TUNING_GUIDE.md)
- [远程调试](../docs/REMOTE_DEBUG.md)
