#!/usr/bin/env bash
# Infer session on 125: MuJoCo mirrors real joints + save triple-cam video + action log.
#
# Robot side (separate terminal / tmux on r1lite@10.229.66.95):
#   bash scripts/start_robot_cameras_and_relays.sh   # cams + MJPEG + joint TCP
#   python3 scripts/ahawam_r1lite_policy_bridge.py --mock-policy   # or real server
#
# Usage on 125:
#   bash start_r1lite_infer_session.sh start [--seconds N]
#   bash start_r1lite_infer_session.sh stop
#   bash start_r1lite_infer_session.sh status
set -eo pipefail

ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}" 2>/dev/null || echo "${BASH_SOURCE[0]}")")" && pwd)"
LOG_DIR="${LOG_DIR:-$ROOT/logs}"
SESSION_ROOT="${R1LITE_SESSION_ROOT:-/home/yao/r1lite_lerobot_datasets/sessions}"
DISPLAY="${DISPLAY:-:1}"
export DISPLAY LOG_DIR R1LITE_SESSION_ROOT="$SESSION_ROOT"
mkdir -p "$LOG_DIR" "$SESSION_ROOT"

SECONDS_ARG="${2:-}"
EXTRA_ARGS=()
CMD="${1:-}"
if [[ "$CMD" == "start" ]]; then
  shift || true
  while [[ $# -gt 0 ]]; do
    EXTRA_ARGS+=("$1")
    shift
  done
fi

kill_pf() {
  local pf="$1"
  if [[ -f "$pf" ]]; then
    local pid
    pid="$(cat "$pf" 2>/dev/null || true)"
    if [[ -n "${pid:-}" ]]; then
      kill "$pid" 2>/dev/null || true
      sleep 1
      kill -9 "$pid" 2>/dev/null || true
    fi
    rm -f "$pf"
  fi
}

cmd_stop() {
  echo "[stop] session video recorder..."
  kill_pf "$LOG_DIR/session_record.pid"
  echo "[stop] MuJoCo..."
  kill_pf "$LOG_DIR/mujoco_r1lite.pid"
  pgrep -f "session_sim_video_record.py" | while read -r p; do kill "$p" 2>/dev/null || true; done
  pgrep -f "mujoco_ros_mirror_r1lite.py" | while read -r p; do kill "$p" 2>/dev/null || true; done
  echo "[stop] done"
}

cmd_start() {
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  export MUJOCO_GL="${MUJOCO_GL:-glfw}"
  export MUJOCO_MODEL="${MUJOCO_MODEL:-$ROOT/r1lite.mujoco.urdf}"

  echo "[check] robot ports..."
  nc -z -w 2 10.229.66.95 8766 && echo "[ok] cam :8766" || echo "[warn] cam :8766 down — run robot camera bringup"
  nc -z -w 2 10.229.66.95 8765 && echo "[ok] joints :8765" || echo "[warn] joints :8765 down — sim will idle"

  # MuJoCo mirror
  kill_pf "$LOG_DIR/mujoco_r1lite.pid"
  if [[ -x /opt/ros/humble/bin/python3 ]]; then PY=/opt/ros/humble/bin/python3; else PY=python3; fi
  nohup "$PY" "$ROOT/scripts/mujoco_ros_mirror_r1lite.py" \
    >"$LOG_DIR/mujoco_r1lite.log" 2>&1 &
  echo $! >"$LOG_DIR/mujoco_r1lite.pid"
  echo "[start] MuJoCo pid=$(cat "$LOG_DIR/mujoco_r1lite.pid")"

  # Triple-cam + action session recorder
  kill_pf "$LOG_DIR/session_record.pid"
  nohup python3 "$ROOT/scripts/session_sim_video_record.py" \
    --http-cam "${R1LITE_CAM_HTTP:-http://10.229.66.95:8766}" \
    --tcp-joints "${R1LITE_TCP_HOST:-10.229.66.95}:${R1LITE_TCP_PORT:-8765}" \
    --session-port "${R1LITE_SESSION_PORT:-8777}" \
    --out-dir "$SESSION_ROOT" \
    "${EXTRA_ARGS[@]}" \
    >"$LOG_DIR/session_record.log" 2>&1 &
  echo $! >"$LOG_DIR/session_record.pid"
  sleep 2
  echo "[start] session recorder pid=$(cat "$LOG_DIR/session_record.pid") root=$SESSION_ROOT"
  echo "[hint] on robot: python3 $ROOT/scripts/ahawam_r1lite_policy_bridge.py --mock-policy \\"
  echo "         --session-host 10.229.20.125 --http-cam http://127.0.0.1:8766"
  echo "       real policy: drop --mock-policy, add --server-ip 10.239.121.25 --server-port 10000"
  echo "       real control: add --enable-cmd (only after dry-run OK + e-stop ready)"
  tail -20 "$LOG_DIR/session_record.log" || true
}

cmd_status() {
  echo "=== infer session (125) ==="
  if [[ -f "$LOG_DIR/mujoco_r1lite.pid" ]] && kill -0 "$(cat "$LOG_DIR/mujoco_r1lite.pid")" 2>/dev/null; then
    echo "MuJoCo:   running pid=$(cat "$LOG_DIR/mujoco_r1lite.pid")"
  else
    echo "MuJoCo:   not running"
  fi
  if [[ -f "$LOG_DIR/session_record.pid" ]] && kill -0 "$(cat "$LOG_DIR/session_record.pid")" 2>/dev/null; then
    echo "Recorder: running pid=$(cat "$LOG_DIR/session_record.pid")"
  else
    echo "Recorder: not running"
  fi
  ls -dt "$SESSION_ROOT"/session_* 2>/dev/null | head -3 || echo "(no sessions yet)"
  nc -z -w 2 10.229.66.95 8766 && echo "cam :8766 ok" || echo "cam :8766 DOWN"
  nc -z -w 2 10.229.66.95 8765 && echo "joints :8765 ok" || echo "joints :8765 DOWN"
  nc -z -w 2 10.239.121.25 10000 && echo "policy 10.239.121.25:10000 ok" || echo "policy 10.239.121.25:10000 DOWN (use --mock-policy; SSH -p 32496)"
}

case "${CMD}" in
  start) cmd_start ;;
  stop) cmd_stop ;;
  status) cmd_status ;;
  restart) cmd_stop; sleep 1; CMD=start; cmd_start ;;
  *) echo "Usage: $0 {start|stop|status|restart} [--seconds N]"; exit 1 ;;
esac
