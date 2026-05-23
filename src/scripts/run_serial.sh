#!/usr/bin/env bash
# 启动裁判系统串口驱动, stdout/stderr 同时显示并落 logs/serial.log
# 用法: bash src/scripts/run_serial.sh [extra ros2 args...]

set -u

SENTRY_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="$SENTRY_WS/logs"
LOG_FILE="$LOG_DIR/serial.log"

mkdir -p "$LOG_DIR"
cd "$SENTRY_WS"

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source install/setup.bash

{
  echo
  echo "===== $(date '+%F %T') run_serial.sh start (cwd=$SENTRY_WS) ====="
} >>"$LOG_FILE"

# stdbuf -oL -eL: 行缓冲, 让日志实时写入而不是退出后才 flush
exec stdbuf -oL -eL ros2 launch serial_driver serial_driver.launch.py "$@" \
  2> >(stdbuf -oL -eL tee -a "$LOG_FILE" >&2) \
  > >(stdbuf -oL -eL tee -a "$LOG_FILE")
