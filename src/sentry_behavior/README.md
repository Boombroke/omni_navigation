# sentry_behavior

哨兵机器人战术决策包。基于 [BehaviorTree.CPP 4.9](https://github.com/BehaviorTree/BehaviorTree.CPP) + [BehaviorTree.ROS2 0.3](https://github.com/BehaviorTree/BehaviorTree.ROS2)（in-tree，不跟随 upstream jazzy）。

`sentry_behavior_server` 继承 `BT::ros2::TreeExecutionServer`，对外暴露一个 `sentry_behavior` action server；`sentry_behavior_client` 启动后发送 `target_tree` 名字（必须匹配 XML 中 `BehaviorTree ID`）即可执行对应战术树。

## 架构

```
裁判系统 (referee/*) ──► sentry_behavior_server
                              │  订阅后写入全局 blackboard:
                              │    @referee_gameStatus   (GameStatus)
                              │    @referee_robotStatus  (RobotStatus)
                              │    @referee_gameRobotHP  (GameRobotHP)
                              │
                              ▼
                    BehaviorTree.CPP 4.9 主循环 (tick_frequency=20 Hz)
                              │
                              ▼
                    战术树 XML (behavior_trees/*.xml)
                              │
                              ▼
                    PubGoal (SyncActionNode)
                    /goal_pose topic publish
                    立刻 SUCCESS (fire-and-forget)
```

服务端开 `Groot2Publisher` 在端口 `1667`（`groot2_port` 参数），可用 [Groot2](https://www.behaviortree.dev/groot/) 或 [btviz](https://github.com/Boombroke/btviz) 实时可视化树状态。

## 战术树

`behavior_trees/` 下四棵树，运行时由 `target_tree` 参数（必须匹配 XML 中 `BehaviorTree ID`）选择：

| BehaviorTree ID | 文件 | 用途 | 主要节点 |
|---|---|---|---|
| `rmuc_defend` | `RMUC.xml` | RMUC 守家策略：守点 ↔ 补给，含血量/弹量滞回控制 | PubGoal + IsStatusOK |
| `a` | `a.xml` | 进攻向：ammo≥1 且 hp≥300 → 攻击点，否则补给 | PubGoal + IsStatusOK |
| `b` | `b.xml` | 防守向：前哨站存活时巡逻/补给，前哨站倒后切点3 | PubGoal + IsStatusOK + IsOutpostStatusOK |
| `test_navigate` | `test_navigate.xml` | 两点往返冒烟测试（仅 INV 测试脚本使用） | NavigateTo |

> **默认 target_tree**：launch 默认 `a`；`sentry_behavior.yaml` 中 `sentry_behavior_client` 也默认 `a`。

### PubGoal 与 NavigateTo 的语义差异

| | PubGoal | NavigateTo |
|---|---|---|
| 类型 | `BT::SyncActionNode` | `BT::ros2::RosActionNode<NavigateToPose>` |
| 调用方式 | publish `geometry_msgs/PoseStamped` 到指定 topic | 调用 Nav2 `navigate_to_pose` action |
| 返回时机 | 立刻 SUCCESS（fire-and-forget） | 等 Nav2 回 SUCCESS / FAILURE / feedback |
| 适用场景 | 持续驻守（配合 KeepRunningUntilFailure） | 路径必须走完才推进的任务 |

当前三棵战术树（`rmuc_defend` / `a` / `b`）均用 `PubGoal`；`NavigateTo` 保留在插件库供自定义树使用，`test_navigate.xml` 展示其用法。

### 多场比赛永循环外壳（三棵战术树共用结构）

```
KeepRunningUntilFailure                   ← 一场赛结束后 reset，等下一场
└─ Sequence
   ├─ RetryUntilSuccessful(num_attempts=-1)
   │  └─ Delay(500ms)                     ← 节流，防 RetryNode spam tick
   │     └─ IsGameStatus(expected=4)       ← progress != 4 → FAILURE → 重试
   └─ ReactiveFallback                    ← 比赛中
      ├─ Inverter(IsGameStatus expected=4) ← progress 翻非 4 → SUCCESS → 一场赛结束
      └─ KeepRunningUntilFailure           ← 比赛中持续运行主决策
         └─ <主决策 WhileDoElse / Fallback>
```

`Delay(500ms)` 包裹 `IsGameStatus` 的目的：若 progress != 4，`RetryUntilSuccessful` 会无延迟反复重 tick，早期版本实测产生 357 MB/min 日志；加 Delay 后降至 2 tick/s。

### RMUC.xml（`rmuc_defend`）守家逻辑

```
ReactiveSequence
├─ IsGameStatus(expected=4)               ← 进入比赛中才执行主决策
└─ KeepRunningUntilFailure
   └─ WhileDoElse
      ├─ 条件: HP > 150 且 ammo > 0       → do_A: 驻守防守点 (3.71, -0.61)
      └─ else: 低血/弹尽时               → WhileDoElse（嵌套）
         ├─ 条件: HP ≥ 400 且 ammo ≥ 50  → do_A: 返回防守点 (3.71, -0.61)
         └─ else: 继续补给               → 驻守补给点 (-0.27, -3.94)
```

一旦 `game_progress` 离开 4，`ReactiveSequence` 根节点的 `IsGameStatus` 立刻返回 FAILURE，停止整块主决策，`ReactiveFallback` 首项（`Inverter(IsGameStatus)`）返回 SUCCESS，触发一场赛结束流程。

## 插件清单

`plugins/` 下编出的 BT 插件 `.so`，运行时由 `BT::SharedLibrary` 动态加载。

### Conditions

| 注册 ID | 源文件 | 行为 |
|---|---|---|
| `IsGameStatus` | `plugins/condition/is_game_status.cpp` | 读 `@referee_gameStatus`，校验 `game_progress == expected_game_progress` 且 `stage_remain_time ∈ [min_remain_time, max_remain_time]` |
| `IsStatusOK` | `plugins/condition/is_status_ok.cpp` | 读 `@referee_robotStatus`，校验 `current_hp >= hp_min` 且 `ammo_min <= projectile_allowance_17mm <= ammo_max` |
| `IsOutpostStatusOK` | `plugins/condition/is_outpost_status_ok.cpp` | 读 `@referee_gameRobotHP`，校验 `ally_outpost_hp >= outpost_hp_min` |

### Actions

| 注册 ID | 源文件 | 类型 | 行为 |
|---|---|---|---|
| `PubGoal` | `plugins/action/pub_goal.cpp` | `BT::SyncActionNode` | publish `PoseStamped` 到指定 topic；进程级共享 publisher 避免 DDS discovery 首包丢失；坐标未变化时去重不重发 |
| `NavigateTo` | `plugins/action/navigate_to.cpp` | `RosActionNode<NavigateToPose>` | 调用 Nav2 `navigate_to_pose` action，等真实 SUCCESS / FAILURE；`error_code` 输出口透传 `rclcpp_action::ResultCode` |

## ROS 接口

### 服务端订阅（写入全局 blackboard）

| 话题 | 消息类型 | blackboard 键 |
|---|---|---|
| `referee/game_status` | `rm_interfaces/msg/GameStatus` | `@referee_gameStatus` |
| `referee/robot_status` | `rm_interfaces/msg/RobotStatus` | `@referee_robotStatus` |
| `referee/all_robot_hp` | `rm_interfaces/msg/GameRobotHP` | `@referee_gameRobotHP` |

### 发布 / Action

| 接口 | 类型 | 说明 |
|---|---|---|
| `/goal_pose` | `geometry_msgs/PoseStamped` | `PubGoal` 发出 |
| `/behavior_tree_log` | BT.CPP 内部 logger | 节点状态切换日志 |
| action `/sentry_behavior` | `btcpp_ros2_interfaces/action/ExecuteTree` | server 对外 action（client 发 target_tree 到此） |
| action `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `NavigateTo` 调用（仅 test_navigate.xml 用） |

## 参数（`params/sentry_behavior.yaml`）

```yaml
sentry_behavior_server:
  ros__parameters:
    use_sim_time: true
    action_name: sentry_behavior        # 对外 action server 名
    tick_frequency: 20                  # BT 主循环频率, Hz
    groot2_port: 1667                   # Groot2 / btviz 远程可视化端口
    ros_plugins_timeout: 5000           # ms; NavigateTo 的 server_timeout + wait_for_server_timeout
    use_cout_logger: false              # 终端打印节点状态切换 (debug 用)
    plugins:
      - sentry_behavior/bt_plugins
    behavior_trees:
      - sentry_behavior/behavior_trees

sentry_behavior_client:
  ros__parameters:
    use_sim_time: true
    target_tree: a                      # a / b / rmuc_defend / test_navigate
```

## 启动

```bash
# 默认 target_tree=a
ros2 launch sentry_behavior sentry_behavior_launch.py

# RMUC 守家树
ros2 launch sentry_behavior sentry_behavior_launch.py target_tree:=rmuc_defend

# 防守向（含前哨站检查）
ros2 launch sentry_behavior sentry_behavior_launch.py target_tree:=b

# 实车配置（use_sim_time=false）
ros2 launch sentry_behavior sentry_behavior_launch.py use_sim_time:=false target_tree:=a
```

实战中由 `sentry_nav_bringup/launch/rm_sentry_launch.py` 统一拉起（`enable_behavior:=True` 时），不单独跑。

## 可视化（Groot2 替代）

[btviz](https://github.com/Boombroke/btviz) 是仓库作者写的 Tauri 桌面应用，连接 `Groot2Publisher` 实时刷新（无 20 节点限制）：

```bash
ros2 launch sentry_behavior sentry_behavior_launch.py target_tree:=a
# 另开终端或另一台机器:
btviz   # Connect → tcp://127.0.0.1:1667
```

## 回归测试

`tests/inv_smoke.sh` 启动 `sentry_behavior_server` 并以 `target_tree=rmuc_defend` 执行守家树，注入裁判数据，从 `/goal_pose` topic 抓 `PubGoal` 发出的坐标序列，验证 7 条行为不变量：

| INV | 描述 |
|---|---|
| INV-1 | `game_progress != 4`（未开赛）→ 等待，不发 `/goal_pose` |
| INV-2 | `progress=4` + `hp>150` + `ammo>0` → 驻守防守点 `(3.71, -0.61)` |
| INV-3 | `progress=4` + `ammo=0` → 切到补给点 `(-0.27, -3.94)` |
| INV-4 | 守点 → 弹尽回补 → 补满恢复返回守点（`(3.71,-0.61)→(-0.27,-3.94)→(3.71,-0.61)` 滞回） |
| INV-5 | 守点 → `hp≤150` → 回补给 `(-0.27, -3.94)` |
| INV-6 | 比赛结束（`progress` 4→5）→ 停止主决策（守点已发、补给不再出现） |
| INV-7 | server 重启后仍能执行决策并发守点 `(3.71, -0.61)` |

```bash
source install/setup.bash

# 跑全套（exit code = FAIL 数）
tests/inv_smoke.sh

# 记录基线（重构前）
tests/inv_smoke.sh --baseline tests/baseline

# 回归对比（重构后）
tests/inv_smoke.sh --regress tests/baseline
```

> **注意**：`rmuc_defend` 用 `PubGoal` 发布到 `/goal_pose`（非 NavigateToPose action），脚本据此从 `/goal_pose` topic 抓坐标。断言坐标取自 `RMUC.xml`（守点 `(3.71, -0.61)`、补给 `(-0.27, -3.94)`）——改动树坐标或树名需同步更新本脚本断言。

## 目录结构

```
sentry_behavior/
├── behavior_trees/     战术树 XML (RMUC.xml / a.xml / b.xml / test_navigate.xml)
├── include/            头文件
├── launch/             sentry_behavior_launch.py
├── params/             sentry_behavior.yaml
├── plugins/            BT 插件 C++ 源
│   ├── action/         pub_goal.cpp / navigate_to.cpp
│   └── condition/      is_game_status.cpp / is_status_ok.cpp / is_outpost_status_ok.cpp
├── src/                sentry_behavior_server.cpp / sentry_behavior_client.cpp
└── CMakeLists.txt
```

## 添加新节点

1. 在 `plugins/<kind>/` 下写 `.cpp`，继承 `BT::SimpleConditionNode` / `BT::ros2::RosActionNode<T>` / `BT::SyncActionNode` 等
2. 文件末尾用 `CreateRosNodePlugin(NS::ClassName, "RegisteredID");` 或 `BT_REGISTER_NODES(factory)` 导出
3. `CMakeLists.txt` 加：

   ```cmake
   ament_auto_add_library(my_node SHARED plugins/<kind>/my_node.cpp)
   list(APPEND plugin_libs my_node)
   ```

4. `target_compile_definitions(... PRIVATE BT_PLUGIN_EXPORT)` 已在 foreach 里统一加
5. 在战术树 XML 里直接 `<MyNode .../>` 引用
