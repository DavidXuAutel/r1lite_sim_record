#!/bin/bash
# Start R1Lite MuJoCo mirror (prefer start_r1lite_sim_record.sh for full stack).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${LOG_DIR:-$ROOT/logs}"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash 2>/dev/null || true
if [[ -x /opt/ros/humble/bin/python3 ]]; then
  PY=/opt/ros/humble/bin/python3
else
  PY=python3
fi
export DISPLAY="${DISPLAY:-:1}"
export MUJOCO_GL="${MUJOCO_GL:-glfw}"
export MUJOCO_MODEL="${MUJOCO_MODEL:-$ROOT/r1lite.mujoco.urdf}"
export MUJOCO_SYNC_HZ="${MUJOCO_SYNC_HZ:-30}"
export R1LITE_TCP_HOST="${R1LITE_TCP_HOST:-10.229.66.95}"
export R1LITE_TCP_PORT="${R1LITE_TCP_PORT:-8765}"

if [[ -f "$LOG_DIR/mujoco_r1lite.pid" ]]; then
  kill "$(cat "$LOG_DIR/mujoco_r1lite.pid")" 2>/dev/null || true
  sleep 1
fi

nohup "$PY" "$ROOT/scripts/mujoco_ros_mirror_r1lite.py" \
  > "$LOG_DIR/mujoco_r1lite.log" 2>&1 &
echo $! > "$LOG_DIR/mujoco_r1lite.pid"
sleep 2
echo "R1Lite MuJoCo: pid=$(cat "$LOG_DIR/mujoco_r1lite.pid") node=/r1lite/mujoco_mirror MODEL=$MUJOCO_MODEL"
tail -20 "$LOG_DIR/mujoco_r1lite.log" || true
