#!/usr/bin/env bash
# 跟随模式 mock 集成测试 (无硬件/无仿真):
# 起 sentry_behavior_node(enable_follow) + 30Hz 运动目标 mock, 抓 /goal_pose 验跟随效果。
# 调 follow 参数后可直接重跑本脚本看效果 (默认期望 standoff=1.5, 改参需同步 follow_mock.py 阈值)。
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "$HERE/install/setup.bash" 2>/dev/null || true
set -u

cleanup() { pkill -9 sentry_behavior 2>/dev/null || true; }
trap cleanup EXIT

: > /tmp/follow_node.log
setsid bash -c "source /opt/ros/jazzy/setup.bash; source '$HERE/install/setup.bash'; exec ros2 run sentry_behavior sentry_behavior_node --ros-args -p strategy:=rmuc_defend -p enable_follow:=true -p viz_enable:=false" > /tmp/follow_node.log 2>&1 &
sleep 4
python3 "$HERE/tests/follow_mock.py"
