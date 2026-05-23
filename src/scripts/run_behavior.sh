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

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source install/setup.bash

{
  echo
  echo "===== $(date '+%F %T') run_behavior.sh start tree=$TARGET_TREE ====="
} >>"$LOG_FILE"

exec stdbuf -oL -eL ros2 launch sentry_behavior sentry_behavior_launch.py \
  target_tree:="$TARGET_TREE" "$@" \
  2> >(stdbuf -oL -eL tee -a "$LOG_FILE" >&2) \
  > >(stdbuf -oL -eL tee -a "$LOG_FILE")
