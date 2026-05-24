#!/usr/bin/env bash
# 启动行为树决策, stdout/stderr 同时显示并落 logs/behavior.log
# 用法: bash src/scripts/run_behavior.sh [target_tree:=a/b/RMUC ...]
# 默认 target_tree=a; 通过环境变量 TARGET_TREE 覆盖

set -u

SENTRY_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="$SENTRY_WS/logs"
LOG_FILE="$LOG_DIR/behavior.log"
TARGET_TREE="${TARGET_TREE:-a}"

mkdir -p "$LOG_DIR"
cd "$SENTRY_WS"

# ROS / colcon 的 setup.bash 会引用 AMENT_TRACE_SETUP_FILES 等未定义变量
# 在 set -u 下会炸. 临时关 -u source 完再恢复.
set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source install/setup.bash
set -u

{
  echo
  echo "===== $(date '+%F %T') run_behavior.sh start tree=$TARGET_TREE ====="
} >>"$LOG_FILE"

exec stdbuf -oL -eL ros2 launch sentry_behavior sentry_behavior_launch.py \
  target_tree:="$TARGET_TREE" "$@" \
  2> >(stdbuf -oL -eL tee -a "$LOG_FILE" >&2) \
  > >(stdbuf -oL -eL tee -a "$LOG_FILE")
