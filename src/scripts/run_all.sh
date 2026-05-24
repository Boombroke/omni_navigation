#!/usr/bin/env bash
# 一键启动 serial / nav / behavior 三个组件, 各自落 logs/<name>.log
# 终端输出加 [serial] / [nav] / [behavior] 前缀便于区分来源
# Ctrl+C 一并停止 (先 SIGINT 让 ros2 launch 优雅退出, 2s 后兜底 SIGKILL)
#
# 用法:
#   bash src/scripts/run_all.sh                                          # 默认: 实车 nav + tree=a
#   TARGET_TREE=b bash src/scripts/run_all.sh
#   NAV_LAUNCH=rm_navigation_simulation_launch.py \
#     bash src/scripts/run_all.sh world:=rmuc_2026 slam:=False           # 仿真; 透传参数给 nav
#
# 透传规则:
#   - 命令行 "$@" 全部传给 run_nav.sh (常用的 slam:= / world:= 等)
#   - 串口 / 行为树用环境变量配 (TARGET_TREE 等), 见 run_serial.sh / run_behavior.sh

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 注意: 本脚本不 source ROS / install/setup.bash, 子脚本 (run_*.sh) 各自处理.
# AMENT_TRACE_SETUP_FILES unbound 报错的根因在子脚本, 见 commit f7f2c59 已修.

PIDS=()

# bash 旧版本对空数组 "${ARR[@]}" 在 set -u 下会报 unbound; 下面所有遍历用 :- 兜底.

launch_with_prefix() {
  local tag="$1"; shift
  ( "$@" 2>&1 | stdbuf -oL sed -u "s/^/[$tag] /" ) &
  PIDS+=("$!")
}

cleanup() {
  local rc=${1:-0}
  echo
  echo "[run_all] stopping all subprocesses..."
  for pid in "${PIDS[@]:-}"; do
    [ -z "$pid" ] && continue
    kill -INT "$pid" 2>/dev/null || true
  done
  # 给 ros2 launch 一段时间优雅 shutdown lifecycle nodes
  for _ in $(seq 1 20); do
    local alive=0
    for pid in "${PIDS[@]:-}"; do
      [ -z "$pid" ] && continue
      kill -0 "$pid" 2>/dev/null && alive=1
    done
    [ "$alive" -eq 0 ] && break
    sleep 0.2
  done
  for pid in "${PIDS[@]:-}"; do
    [ -z "$pid" ] && continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "[run_all] force kill pid=$pid"
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  echo "[run_all] done"
  exit "$rc"
}

trap 'cleanup 0' INT TERM

launch_with_prefix "serial"   bash "$SCRIPT_DIR/run_serial.sh"
launch_with_prefix "nav"      bash "$SCRIPT_DIR/run_nav.sh" "$@"
launch_with_prefix "behavior" bash "$SCRIPT_DIR/run_behavior.sh"

echo "[run_all] started ${#PIDS[@]} processes"
echo "[run_all] logs: logs/serial.log  logs/nav.log  logs/behavior.log"
echo "[run_all] Ctrl+C to stop all."

wait
