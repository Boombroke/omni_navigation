# sentry_nav_bringup

哨兵机器人导航总入口包。汇集所有 launch 脚本、Nav2 参数、地图、PCD、RViz 配置与 Nav2 内置行为树 XML。

---

## 包功能概述

`sentry_nav_bringup` 承担以下职责：

- 提供仿真与实车的 **launch 入口**（单机/多机）
- 存放 `config/simulation/` 与 `config/reality/` 两套 **Nav2 参数**
- 存放地图（`map/`）、先验点云（`pcd/`）、RViz 布局（`rviz/`）
- 存放 `bt_navigator` 使用的 **Nav2 内置 BT XML**（与 `sentry_behavior` 战术树不同）

---

## 核心设计

**自旋底盘 + 虚拟惯性系**：底盘持续自旋，`gimbal_yaw_fake` 与 `gimbal_yaw` 反向旋转，
使 Nav2 规划器始终在稳定的惯性系中工作。`fake_vel_transform` 在执行端把 Nav2 输出旋回真实系
并叠加自旋速度分量，再发往底盘。

### TF 树

```
map → odom → base_footprint → chassis → gimbal_yaw → gimbal_pitch → front_mid360
                                             ↓
                                       gimbal_yaw_fake   (Nav2 规划用虚拟系)
```

### 速度指令链路

从 `navigation_launch.py` remappings 直接确认的话题名称：

```
controller_server     →  cmd_vel_controller   (remap: cmd_vel → cmd_vel_controller)
                                ↓
velocity_smoother     订阅 cmd_vel_controller  (remap 输入)
                      发布 cmd_vel_nav2_result (remap: cmd_vel_smoothed → cmd_vel_nav2_result)
                                ↓
fake_vel_transform    订阅 cmd_vel_nav2_result (input_cmd_vel_topic)
                      发布 cmd_vel_chassis     (output_cmd_vel_topic)
                                ↓
        仿真: Gazebo MecanumDrive2 (spawn_robots remap: cmd_vel → cmd_vel_chassis)
        实车: rm_serial_driver 订阅 /cmd_vel_chassis
```

> `enable_stamped_cmd_vel: true` 在 `controller_server`、`behavior_server`、`velocity_smoother`
> 三处**必须一致**，否则类型不匹配导致链路断开。

---

## Launch 入口

### 一览表

| 脚本 | 用途 | 关键参数与默认值 |
|---|---|---|
| `rm_navigation_simulation_launch.py` | **仿真主入口**（第二终端，Gazebo 就绪后启动） | 见下表 |
| `rm_navigation_reality_launch.py` | **实车主入口**（不含串口驱动） | 见下表 |
| `rm_sentry_launch.py` | **实车一键**（导航 + 串口 + 录包 + 可选行为树） | 见下表 |
| `rm_simulation_all_launch.py` | 仿真全栈一键（Gazebo + 自动解暂停 + 导航），时序敏感 | `world`(读 gz_world.yaml), `slam`(`True`), `headless`(`false`), `use_rviz`(`True`), `nav_delay`(`15.0`) |
| `rm_multi_navigation_simulation_launch.py` | 多机仿真，每机实例化一套 nav | `world`(`rmuc_2025`), `robots`(字符串), `map`, `params_file`, `use_rviz`(`True`) |
| `bringup_launch.py` | 内部聚合，被以上入口 include | `slam`, `map`, `prior_pcd_file`, `scan_context_db_file`, `use_composition`(`True`), `log_level`(`info`) |
| `slam_launch.py` | SLAM 模式（point_lio + slam_toolbox + pointcloud_to_laserscan） | `namespace`, `params_file`, `use_sim_time`, `autostart`, `use_respawn`, `log_level` |
| `localization_launch.py` | 定位模式（point_lio + map_server + small_gicp_relocalization） | `namespace`, `map`, `prior_pcd_file`, `scan_context_db_file`, `use_composition`(`False`), `container_name`(`nav2_container`) |
| `navigation_launch.py` | Nav2 lifecycle 节点群 + terrain + odom_bridge + fake_vel_transform | `namespace`, `params_file`, `use_composition`(`False`), `container_name`(`nav2_container`) |
| `robot_state_publisher_launch.py` | URDF + TF（仅在导航模块独立运行时使用） | `namespace`, `use_sim_time`, `robot_name`(`sentry_robot`) |
| `rviz_launch.py` | RViz2 可视化（退出 RViz 触发整体 Shutdown） | `namespace`, `rviz_config`(`rviz/nav2_default_view.rviz`) |

### rm_navigation_simulation_launch.py 参数表

| 参数 | 默认值 | 说明 |
|---|---|---|
| `namespace` | `red_standard_robot1` | 机器人命名空间 |
| `slam` | `False` | `True` = SLAM 建图；`False` = 重定位 |
| `world` | `rmuc_2025` | 场地名，同时决定 map/pcd 路径 |
| `map` | `map/simulation/<world>.yaml` | 2D 地图（slam=False 时加载） |
| `prior_pcd_file` | `pcd/simulation/<world>.pcd` | small_gicp 先验点云 |
| `scan_context_db_file` | `pcd/simulation/<world>.scdb` | Scan Context 数据库；不存在时退化为无全局重定位 |
| `use_sim_time` | `True` | 使用 Gazebo 时钟 |
| `params_file` | `config/simulation/nav2_params.yaml` | Nav2 参数文件 |
| `autostart` | `true` | 自动激活 lifecycle 节点 |
| `use_composition` | `True` | 使用 component container |
| `use_respawn` | `False` | 节点崩溃后自动重启（非 composition 模式） |
| `use_rviz` | `True` | 启动 RViz2 |
| `use_foxglove` | `False` | 启动 foxglove_bridge (port 8765) |

### rm_navigation_reality_launch.py 参数表

| 参数 | 默认值 | 说明 |
|---|---|---|
| `namespace` | `""` | 机器人命名空间（空串 = 无前缀） |
| `slam` | `False` | `True` = SLAM 建图；`False` = 重定位 |
| `world` | `204` | 场地名，决定 `map/reality/<world>.yaml` 与 `pcd/reality/<world>.pcd` 路径 |
| `map` | `map/reality/<world>.yaml` | 2D 地图 |
| `prior_pcd_file` | `pcd/reality/<world>.pcd` | small_gicp 先验点云 |
| `scan_context_db_file` | `pcd/reality/<world>.scdb` | Scan Context 数据库 |
| `use_sim_time` | `False` | 使用系统时钟 |
| `params_file` | `config/reality/nav2_params.yaml` | Nav2 参数文件 |
| `autostart` | `true` | 自动激活 lifecycle 节点 |
| `use_composition` | `True` | 使用 component container |
| `use_respawn` | `False` | 节点崩溃后自动重启 |
| `use_robot_state_pub` | `True` | 是否启动 robot_state_publisher（导航独立运行时需要） |
| `use_rviz` | `True` | 启动 RViz2 |
| `use_foxglove` | `False` | 启动 foxglove_bridge (port 8765) |

### rm_sentry_launch.py 参数表

| 参数 | 默认值 | 说明 |
|---|---|---|
| `namespace` | `""` | 机器人命名空间 |
| `slam` | `False` | SLAM 模式开关 |
| `world` | `204` | 场地名 |
| `use_rviz` | `True` | 启动 RViz2 |
| `use_foxglove` | `False` | 启动 foxglove_bridge |
| `enable_recorder` | `True` | 启动 sentry_match_recorder（延迟 5s，game_progress=4 时自动录包） |
| `enable_behavior` | `False` | 启动 sentry_behavior 战术决策（延迟 8s） |
| `target_tree` | `1` | 传给 sentry_behavior_client 的行为树 ID |

---

## 启动方式

### 仿真（两步法，推荐）

`rm_simulation_all_launch.py` 的定时器方案在低性能机器上仍可能竞争，推荐手动两步：

```bash
# 终端 1：启动 Gazebo（Wayland 下加 QT_QPA_PLATFORM=xcb 避免崩溃）
QT_QPA_PLATFORM=xcb ros2 launch rmu_gazebo_simulator bringup_sim.launch.py

# Gazebo UI 的 Play 按钮在 Wayland 下经常失灵，手动解暂停：
gz service -s /world/default/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'pause: false'

# 等 ~10s，让传感器数据和仿真时钟稳定

# 终端 2：首次（无地图）
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=True

# 有地图后
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False
```

支持的 `world` 值：`rmuc_2025`（默认）/ `rmuc_2026` / `rmul_2026`。

SLAM 完成后把 `.pgm` + `.yaml` 存入 `map/simulation/<world>.yaml`，
`.pcd` 存入 `pcd/simulation/<world>.pcd`，再切 `slam:=False` 走重定位。

### 仿真全栈一键（可选）

```bash
# rm_simulation_all_launch.py 从 rmu_gazebo_simulator/config/gz_world.yaml 读取 world
# 自动在 8s 后解暂停，nav_delay（默认 15s）后启动导航
ros2 launch sentry_nav_bringup rm_simulation_all_launch.py \
  slam:=True nav_delay:=20.0
```

### 实车建图

```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py \
  slam:=True use_robot_state_pub:=True
```

### 实车导航（已有地图）

```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py \
  world:=<场地名> slam:=False use_robot_state_pub:=True
```

### 实车一键（导航 + 串口）

```bash
ros2 launch sentry_nav_bringup rm_sentry_launch.py

# 指定场地 + 开启行为树：
ros2 launch sentry_nav_bringup rm_sentry_launch.py \
  world:=rmul_2026 enable_behavior:=True target_tree:=1
```

---

## 配置目录结构

```
config/
├── simulation/
│   └── nav2_params.yaml          仿真 Nav2 参数 (use_sim_time=true, controller_frequency=20Hz)
└── reality/
    ├── nav2_params.yaml           实车 Nav2 参数 (use_sim_time=false, controller_frequency=30Hz)
    └── mid360_user_config.json    Livox Mid360 驱动网络配置

map/                               2D 地图（用户建图后手动存放）
│   # 建议命名：map/simulation/<world>.yaml  /  map/reality/<world>.yaml

pcd/                               先验点云（用户建图后手动存放）
│   # 建议命名：pcd/simulation/<world>.pcd   /  pcd/reality/<world>.pcd
│   #           pcd/simulation/<world>.scdb  /  pcd/reality/<world>.scdb

rviz/
└── nav2_default_view.rviz         Nav2 调试可视化布局

behavior_trees/                    Nav2 bt_navigator 使用的内置 BT（非战术决策树）
├── navigate_to_pose_w_replanning_and_recovery.xml
└── navigate_through_poses_w_replanning_and_recovery.xml
```

> 战术决策树（`RMUC.xml` 等）在 `sentry_behavior/behavior_trees/`，与此目录无关。

---

## 仿真 vs 实车关键参数差异

| 参数 | 仿真 | 实车 | 位置 |
|---|---|---|---|
| `controller_frequency` | 20.0 Hz | 30.0 Hz | `controller_server` |
| `smoothing_frequency` | 20.0 Hz | 30.0 Hz | `velocity_smoother`（必须 == controller_frequency） |
| `FollowPath.plugin` | `omni_pid_pursuit_controller::OmniPidPursuitController` | `pb_omni_pid_pursuit_controller::OmniPidPursuitController` | `controller_server.FollowPath` |
| `robot_radius` (local) | 0.20 m | 0.35 m | `local_costmap` |
| `robot_radius` (global) | 0.20 m | 0.30 m | `global_costmap` |
| `max_velocity` | `[2.5, 2.5, 3.0]` | `[1.5, 1.5, 3.0]` | `velocity_smoother`，格式 `[vx, vy, wz]` |
| `v_linear_max` | 2.5 m/s | 1.5 m/s | `FollowPath` |
| `xy_goal_tolerance` | 0.15 m | 0.40 m | `general_goal_checker` |
| `local_costmap.update_frequency` | 10.0 Hz | 30.0 Hz | `local_costmap` |
| `inflation_radius` | 1.0 m | 0.55 m | `inflation_layer` |
| `cost_scaling_factor` | 1.5 | 5.0 | `inflation_layer` |
| `behavior_server.cycle_frequency` | 10.0 Hz | 20.0 Hz | `behavior_server` |
| `point_lio.common.lid_topic` | `velodyne_points` | `livox/lidar` | `point_lio.common` |
| `enable_global_relocalization` | `true` | `false` | `small_gicp_relocalization` |
| `small_gicp.num_threads` | 4 | 8 | `small_gicp_relocalization` |
| `fake_vel_transform.init_spin_speed` | 0.0 | 0.0 | `fake_vel_transform`（运行时由 `cmd_spin` 动态设置） |

> `robot_base_frame` 在 `controller_server`、`bt_navigator`、`local_costmap`、`global_costmap`、
> `behavior_server` 中均为 `gimbal_yaw_fake`，仿真与实车一致。

---

## 节点与话题

### 节点依赖关系（bringup_launch → 子 launch）

```
bringup_launch.py
  ├── slam=True  → slam_launch.py
  │     ├── point_lio                    (建图里程计，prior_pcd.enable=False)
  │     ├── slam_toolbox                 (sync_slam_toolbox_node，scan_topic=obstacle_scan)
  │     ├── pointcloud_to_laserscan      (in: terrain_map_ext → out: obstacle_scan)
  │     ├── map_saver_server
  │     └── static_transform_publisher   (map → odom，固定零变换)
  │
  ├── slam=False → localization_launch.py
  │     ├── point_lio                    (定位里程计)
  │     ├── map_server                   (加载 2D 地图)
  │     └── small_gicp_relocalization    (发布 map→odom TF)
  │
  └── 始终 → navigation_launch.py
        ├── terrain_analysis
        ├── terrain_analysis_ext
        ├── odom_bridge             (发布 odom→base_footprint TF + odometry 话题)
        ├── fake_vel_transform      (速度坐标旋转 + 自旋叠加)
        ├── controller_server       → cmd_vel_controller
        ├── velocity_smoother       订阅 cmd_vel_controller → 发布 cmd_vel_nav2_result
        ├── planner_server
        ├── behavior_server
        ├── bt_navigator
        ├── waypoint_follower
        └── lifecycle_manager_navigation
```

### 关键话题

| 话题 | 类型 | 方向 | 节点 |
|---|---|---|---|
| `cmd_vel_controller` | `TwistStamped` | 发布 | `controller_server`（remap from `cmd_vel`） |
| `cmd_vel_nav2_result` | `TwistStamped` | 发布 | `velocity_smoother`（remap from `cmd_vel_smoothed`） |
| `cmd_vel_chassis` | `Twist` | 发布 | `fake_vel_transform` |
| `cmd_spin` | `std_msgs/Float32` | 订阅 | `fake_vel_transform`（动态设置自旋速度） |
| `odometry` | `nav_msgs/Odometry` | 发布 | `odom_bridge` |
| `aft_mapped_to_init` | `nav_msgs/Odometry` | 订阅 | `odom_bridge`（来自 point_lio） |
| `cloud_registered` | `sensor_msgs/PointCloud2` | 订阅 | `odom_bridge`（已配准点云） |
| `terrain_map` | `sensor_msgs/PointCloud2` | 发布 | `terrain_analysis` → local_costmap 观测源 |
| `terrain_map_ext` | `sensor_msgs/PointCloud2` | 发布 | `terrain_analysis_ext` → global_costmap + pointcloud_to_laserscan |
| `obstacle_scan` | `sensor_msgs/LaserScan` | 发布 | `pointcloud_to_laserscan`（供 slam_toolbox） |
| `/initialpose` | `PoseWithCovarianceStamped` | 订阅 | `small_gicp_relocalization`（RViz 重定位输入） |
| `/navigate_to_pose` | Action | server | `bt_navigator` |

> 带命名空间前缀的话题（如 `red_standard_robot1/terrain_map`）由 `bringup_launch.py` 中的
> `PushRosNamespace` 自动添加；params yaml 中以 `<robot_namespace>` 占位符标记，启动时替换。

---

## 关键约束

- `controller_frequency` **必须等于** `smoothing_frequency`，两者都不能超过 CPU 实际吞吐上限
- `enable_stamped_cmd_vel: true` 在 `controller_server`、`behavior_server`、`velocity_smoother` 三处必须一致
- `enable_periodic_relocalization: true` 必须开启，否则 small_gicp 不持续周期纠偏
- 仿真启动顺序：先 Gazebo → 解暂停（`gz service`）→ 等稳定 → 起导航栈
- PCD 先验地图与 2D 地图**必须在同一坐标系、同一起点**建图，不可混用不同 session 的产物
- `world` 默认值差异：仿真入口默认 `rmuc_2025`，实车入口（含 `rm_sentry_launch.py`）默认 `204`

详细调优推导见 [`src/docs/TUNING_GUIDE.md`](../docs/TUNING_GUIDE.md)。

---

## 相关文档

- [系统架构详解](../docs/ARCHITECTURE.md)
- [快速部署](../docs/QUICKSTART.md)
- [运行模式说明](../docs/RUNNING_MODES.md)
- [参数调优](../docs/TUNING_GUIDE.md)
- [远程调试（Foxglove）](../docs/REMOTE_DEBUG.md)
- [sentry_behavior 说明](../sentry_behavior/README.md)
