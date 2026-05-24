#!/usr/bin/env bash
# 启动导航栈, stdout/stderr 同时显示并落 logs/nav.log
# 用法: bash src/scripts/run_nav.sh [ros2 launch args, e.g. slam:=False world:=rmuc_2026]
# 环境变量: NAV_LAUNCH 可改成 rm_navigation_simulation_launch.py 跑仿真

set -u

SENTRY_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="$SENTRY_WS/logs"
LOG_FILE="$LOG_DIR/nav.log"
NAV_LAUNCH="${NAV_LAUNCH:-rm_navigation_reality_launch.py}"

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
  echo "===== $(date '+%F %T') run_nav.sh start launch=$NAV_LAUNCH ====="
} >>"$LOG_FILE"

exec stdbuf -oL -eL ros2 launch sentry_nav_bringup "$NAV_LAUNCH" "$@" \
  2> >(stdbuf -oL -eL tee -a "$LOG_FILE" >&2) \
  > >(stdbuf -oL -eL tee -a "$LOG_FILE")
