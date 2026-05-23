# sentry_behavior

哨兵机器人战术决策包。基于 [BehaviorTree.CPP 4.9](https://github.com/BehaviorTree/BehaviorTree.CPP) + [BehaviorTree.ROS2 0.3](https://github.com/BehaviorTree/BehaviorTree.ROS2)（in-tree，不跟随 upstream jazzy）。

服务端继承 `BT::ros2::TreeExecutionServer`，对外暴露一个 `sentry_behavior` action server；客户端发 `target_tree` 名字（`a` / `b` / `rmuc_2026_sentry`）即可切换战术树。

## 架构

```
裁判系统 (referee/*) ──► 服务端 (sentry_behavior_server)
                              │  写入全局 blackboard:
                              │    @referee_gameStatus / @referee_robotStatus
                              │    @referee_robotsHP   / @referee_rfidStatus
                              │
                              ▼
                    BehaviorTree.CPP 4.9 主循环 (tick_frequency=2 Hz)
                              │
                              ▼
                    战术树 XML  (behavior_trees/*.xml)
                              │
              ┌───────────────┴────────────────┐
              ▼                                ▼
   PubGoal (fire-and-forget)        NavigateTo (Nav2 action client)
   /goal_pose Topic Pub             /navigate_to_pose action
   立刻 SUCCESS                     等 Nav2 回 SUCCESS/FAILURE
   (单点重发, 适合"持续驻守")     (整条路径必须走完, 适合"必到")
```

服务端开 `Groot2Publisher` 在端口 `1667`（`groot2_port` 参数），可用 [Groot2](https://www.behaviortree.dev/groot/) 或 [btviz](https://github.com/Boombroke/btviz) 实时可视化树状态。

## 战术树

`behavior_trees/` 下三棵活跃树，运行时由 `target_tree` 参数选择：

| ID | 文件 | 用途 | 目标点链路 |
|---|---|---|---|
| `rmuc_2026_sentry` | `RMUC.xml` | RMUC 2026 标准战术（首点压制 → 补给 → 次点驻守循环） | NavigateTo |
| `a` | `a.xml` | 进攻向自定义战术；多场比赛永循环外壳 | PubGoal |
| `b` | `b.xml` | 防守向自定义战术（含前哨站存活检查）；多场比赛永循环外壳 | PubGoal |

`test_navigate.xml` 是单点冒烟测试，仅 INV 用例脚本会用到。

### 多场比赛永循环外壳（a.xml / b.xml）

```
KeepRunningUntilFailure                   ← 一场赛结束 SUCCESS 后 reset, 等下一场
└─ Sequence
   ├─ RetryUntilSuccessful                ← 等比赛开始: progress != 4 时反复重试
   │  └─ Delay(500ms)                     ← 节流, 防 RetryNode spam tick
   │     └─ IsGameStatus expected=4
   └─ ReactiveFallback                    ← 比赛中
      ├─ Inverter(IsGameStatus expected=4) ← progress 翻非 4 时 SUCCESS → 一场赛 SUCCESS
      └─ KeepRunningUntilFailure           ← 比赛中持续 publish PubGoal
         └─ <主决策 WhileDoElse>
```

设计要点：
- **`ReactiveSequence` vs `Sequence`**：根用 ReactiveSequence 是为了让 `IsGameStatus` 每 tick 重判；裸 Sequence 会 latch 在已 SUCCESS 的子节点。RMUC.xml 根同样改成 ReactiveSequence。
- **`Delay(500ms)` 包 `IsGameStatus`**：早期版本 RetryUntilSuccessful 直接重 tick，progress != 4 时 4.5M 行/秒级日志爆炸（实测 357MB/分钟）。
- **PubGoal 替代 NavigateTo**：a.xml/b.xml 用 fire-and-forget topic 发布，立刻返回 SUCCESS，配合 KeepRunningUntilFailure 实现"持续驻守"语义。RMUC.xml 仍保留 NavigateTo 走 Nav2 action（需要真实 SUCCESS 反馈）。

## 节点清单（当前实现）

`plugins/` 下编出的 BT 插件 `.so`，运行时由 `BT::SharedLibrary` 动态加载。**只列实际存在的**：

### Conditions

| ID | 来源 | 行为 |
|---|---|---|
| `IsGameStatus` | `plugins/condition/is_game_status.cpp` | 读 blackboard `@referee_gameStatus`，校验 `expected_game_progress` 与剩余时间窗 `[min_remain_time, max_remain_time]` |
| `IsStatusOK` | `plugins/condition/is_status_ok.cpp` | 读 `@referee_robotStatus`，校验弹丸数 `[ammo_min, ammo_max]` 与血量 `>= hp_min` |
| `IsOutpostStatusOK` | `plugins/condition/is_outpost_status_ok.cpp` | 读 `@referee_robotsHP`，校验己方前哨站 HP > 0 |

### Actions

| ID | 来源 | 类型 | 行为 |
|---|---|---|---|
| `NavigateTo` | `plugins/action/navigate_to.cpp` | `RosActionNode<NavigateToPose>` | 调用 Nav2 `navigate_to_pose` action，等真实 SUCCESS / FAILURE |
| `PubGoal` | `plugins/action/pub_goal.cpp` | `RosTopicPubNode<PoseStamped>` | publish 一个 `/goal_pose`，立刻 SUCCESS（fire-and-forget） |

> **设计变迁**：旧版 README 提到的 `is_rfid_detected` / `is_attacked` / `is_detect_enemy` / `is_hp_add` / `battlefield_information` / `pursuit` / `recovery_node` / `rate_controller` / `tick_after_timeout_node` 已在 omni 主线重构中清理；当前只保留实际投产用到的节点。

## 订阅 / 发布话题

服务端 `tree_execution_server` 内部订阅，把每帧消息塞进全局 blackboard，BT 节点直接读取。

| 话题 | 类型 | 黑板键 |
|---|---|---|
| `referee/game_status` | `rm_interfaces/msg/GameStatus` | `@referee_gameStatus` |
| `referee/robot_status` | `rm_interfaces/msg/RobotStatus` | `@referee_robotStatus` |
| `referee/all_robot_hp` | `rm_interfaces/msg/GameRobotHP` | `@referee_robotsHP` |
| `referee/rfid_status` | `rm_interfaces/msg/RfidStatus` | `@referee_rfidStatus` |

发布：
- `/goal_pose` (`geometry_msgs/PoseStamped`) — 由 `PubGoal` 发出
- `/behavior_tree_log` — 由 BT.CPP `RosTopicLogger` 发出（节点状态切换日志）

调用 Nav2：
- action `/navigate_to_pose` — 由 `NavigateTo` 触发

## 参数（`params/sentry_behavior.yaml`）

```yaml
sentry_behavior_server:
  ros__parameters:
    use_sim_time: true
    action_name: sentry_behavior        # 对外 action server 名
    tick_frequency: 2                   # BT 主循环频率, Hz. 决策树是反应式, 太高没用
    groot2_port: 1667                   # Groot2 / btviz 远程可视化端口
    ros_plugins_timeout: 5000           # ms; RosActionNode 的 server_timeout +
                                        # wait_for_server_timeout, 仿真 nav2
                                        # 起得慢, 调大避 SEND_GOAL_TIMEOUT
    use_cout_logger: false              # 终端打印节点状态切换 (debug 用)
    plugins:
      - sentry_behavior/bt_plugins      # share/sentry_behavior/bt_plugins/*.so
    behavior_trees:
      - sentry_behavior/behavior_trees  # share/sentry_behavior/behavior_trees/*.xml

sentry_behavior_client:
  ros__parameters:
    use_sim_time: true
    target_tree: a                      # a / b / rmuc_2026_sentry
```

## 启动

```bash
# 默认 target_tree=a
ros2 launch sentry_behavior sentry_behavior_launch.py

# 切到 RMUC 标准树
ros2 launch sentry_behavior sentry_behavior_launch.py target_tree:=rmuc_2026_sentry

# 实车配置覆盖 (use_sim_time=false)
ros2 launch sentry_behavior sentry_behavior_launch.py use_sim_time:=false target_tree:=a
```

实战中由 `sentry_nav_bringup/launch/rm_*_launch.py` 一并拉起，不单独跑。

## 可视化（Groot2 替代）

[btviz](https://github.com/Boombroke/btviz) 是仓库作者写的 Tauri 桌面应用，能连 `Groot2Publisher` 实时刷新（无 20 节点限制）：

```bash
# 启动决策服务端
ros2 launch sentry_behavior sentry_behavior_launch.py target_tree:=a
# 在另一台机器或本机:
btviz   # Connect → tcp://127.0.0.1:1667
```

## 测试

`tests/inv_smoke/` 下有 INV-1~7 决策树回归脚本，模拟裁判数据驱动 BT 跑指定窗口、抓 `/goal_pose` 序列校验。

```bash
# 单测
ros2 run sentry_behavior_test inv1 ...    # (具体路径见 tests/inv_smoke/README)
# 全套
bash tests/inv_smoke/run_all.sh
```

## 目录结构

```
sentry_behavior/
├── behavior_trees/     战术树 XML (RMUC.xml / a.xml / b.xml / test_navigate.xml)
├── include/            头文件 (action_client / tree_execution_server 扩展)
├── launch/             sentry_behavior_launch.py
├── params/             sentry_behavior.yaml
├── plugins/            BT 插件 C++ 源
│   ├── action/         pub_goal.cpp / navigate_to.cpp
│   └── condition/      is_game_status.cpp / is_status_ok.cpp / is_outpost_status_ok.cpp
├── src/                action_client 实现
└── CMakeLists.txt      ament_auto_add_library 注册插件 (BT_PLUGIN_EXPORT)
```

## 添加新节点

1. 在 `plugins/<kind>/` 下写 `.cpp`，继承 `BT::ConditionNode` / `BT::ros2::RosActionNode<T>` / `BT::ros2::RosTopicPubNode<T>` 等
2. 文件末尾用 `CreateRosNodePlugin(NS::ClassName, "RegisteredID");` 或 `BT_REGISTER_NODES(...)` 导出
3. `CMakeLists.txt` 加：

   ```cmake
   ament_auto_add_library(my_node SHARED plugins/<kind>/my_node.cpp)
   list(APPEND plugin_libs my_node)
   ```

4. `target_compile_definitions(... PRIVATE BT_PLUGIN_EXPORT)` 已在 foreach 里统一加
5. 在战术树 XML 里直接 `<MyNode .../>` 引用即可
