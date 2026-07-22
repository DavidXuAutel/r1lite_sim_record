#!/usr/bin/env bash
# One-shot launcher: R1Lite MuJoCo sim + triple-cam viewer + LeRobot record.
#
# Independent of Franka teleop stack (does not touch port 8765 record API,
# bare /joint_states, or cam_view_dual).
#
# Usage:
#   bash start_r1lite_sim_record.sh start
#   bash start_r1lite_sim_record.sh stop
#   bash start_r1lite_sim_record.sh status
#   bash start_r1lite_sim_record.sh restart
#   bash start_r1lite_sim_record.sh episode-start [--repo NAME] [--task TEXT]
#   bash start_r1lite_sim_record.sh episode-stop
#   bash start_r1lite_sim_record.sh episode-status
#
# Robot prerequisites (r1lite@10.229.66.95):
#   - joint TCP relay on :8765  (tmux awm_r1lite_joint_tcp)
#   - camera MJPEG on :8766     (tmux awm_r1lite_camera_mjpeg)
# Optional helpers:
#   bash scripts/start_joint_relay_on_robot.sh
set -eo pipefail

ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}" 2>/dev/null || echo "${BASH_SOURCE[0]}")")" && pwd)"
LOG_DIR="${LOG_DIR:-$ROOT/logs}"
DISPLAY="${DISPLAY:-:1}"
export DISPLAY LOG_DIR
export LOCAL_DATASET_ROOT="${LOCAL_DATASET_ROOT:-/home/yao/r1lite_lerobot_datasets}"
export R1LITE_RECORD_API="${R1LITE_RECORD_API:-http://127.0.0.1:8775}"
export R1LITE_RECORD_PORT="${R1LITE_RECORD_PORT:-8775}"
export R1LITE_CAM_HTTP="${R1LITE_CAM_HTTP:-http://10.229.66.95:8766}"
export R1LITE_TCP_HOST="${R1LITE_TCP_HOST:-10.229.66.95}"
export R1LITE_TCP_PORT="${R1LITE_TCP_PORT:-8765}"
export R1LITE_REPO="${R1LITE_REPO:-r1lite_teleop}"

mkdir -p "$LOG_DIR"

usage() {
  cat <<EOF
Usage: $(basename "$0") <command>

  start            MuJoCo sim + triple-cam viewer + record daemon
  stop             Stop R1Lite sim/view/record only (Franka untouched)
  status           Process + stream summary
  restart          stop && start

  episode-start    Start one LeRobot episode  [--repo NAME] [--task TEXT]
  episode-stop     Stop current episode
  episode-status   Recorder HTTP status

Environment:
  DISPLAY              default :1
  LOG_DIR              default \$ROOT/logs
  LOCAL_DATASET_ROOT   default /home/yao/r1lite_lerobot_datasets
  R1LITE_RECORD_API    default http://127.0.0.1:8775
  R1LITE_CAM_HTTP      default http://10.229.66.95:8766
  R1LITE_TCP_HOST/PORT joint relay (default 10.229.66.95:8765)
  R1LITE_REPO          default r1lite_teleop
EOF
}

source_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  set -u
}

kill_pidfile() {
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

check_robot_ports() {
  local ok=0
  if nc -z -w 2 "$R1LITE_TCP_HOST" "$R1LITE_TCP_PORT" 2>/dev/null; then
    echo "[ok] joint TCP $R1LITE_TCP_HOST:$R1LITE_TCP_PORT"
  else
    echo "[warn] joint TCP $R1LITE_TCP_HOST:$R1LITE_TCP_PORT not reachable"
    echo "       on robot: tmux session awm_r1lite_joint_tcp / scripts/start_joint_relay_on_robot.sh"
    ok=1
  fi
  local cam_host cam_port
  cam_host="$(echo "$R1LITE_CAM_HTTP" | sed -E 's#https?://([^:/]+).*#\1#')"
  cam_port="$(echo "$R1LITE_CAM_HTTP" | sed -E 's#https?://[^:/]+:([0-9]+).*#\1#')"
  if nc -z -w 2 "$cam_host" "$cam_port" 2>/dev/null; then
    echo "[ok] camera HTTP $R1LITE_CAM_HTTP"
  else
    echo "[warn] camera HTTP $R1LITE_CAM_HTTP not reachable"
    echo "       on robot: tmux awm_r1lite_camera_mjpeg / relay_cameras_mjpeg.py --port 8766"
    ok=1
  fi
  return "$ok"
}

stop_all() {
  echo "[stop] R1Lite record daemon..."
  bash "$ROOT/lerobot_record/r1lite_record_daemon.sh" stop 2>/dev/null || true

  echo "[stop] triple cam viewer..."
  kill_pidfile "$LOG_DIR/cam_view_triple.pid"
  # Avoid matching this launcher script itself
  pgrep -f "python3 .*/cam_view_triple.py" | while read -r pid; do
    kill "$pid" 2>/dev/null || true
  done
  pgrep -f "python3 .*/cam_replay_r1lite.py" | while read -r pid; do
    kill "$pid" 2>/dev/null || true
  done

  echo "[stop] MuJoCo R1Lite mirror..."
  kill_pidfile "$LOG_DIR/mujoco_r1lite.pid"
  pgrep -f "python3 .*/mujoco_ros_mirror_r1lite.py" | while read -r pid; do
    kill "$pid" 2>/dev/null || true
  done
  sleep 1
  pgrep -f "mujoco_ros_mirror_r1lite.py|cam_view_triple.py|r1lite_record_server.py|cam_replay_r1lite.py" \
    | while read -r pid; do kill -9 "$pid" 2>/dev/null || true; done

  echo "[stop] done (Franka teleop/record untouched)"
}

start_mujoco() {
  source_ros
  export MUJOCO_GL="${MUJOCO_GL:-glfw}"
  export MUJOCO_MODEL="${MUJOCO_MODEL:-$ROOT/r1lite.mujoco.urdf}"
  export MUJOCO_SYNC_HZ="${MUJOCO_SYNC_HZ:-30}"
  if [[ -x /opt/ros/humble/bin/python3 ]]; then
    PY=/opt/ros/humble/bin/python3
  else
    PY=python3
  fi
  kill_pidfile "$LOG_DIR/mujoco_r1lite.pid"
  nohup "$PY" "$ROOT/scripts/mujoco_ros_mirror_r1lite.py" \
    > "$LOG_DIR/mujoco_r1lite.log" 2>&1 &
  echo $! > "$LOG_DIR/mujoco_r1lite.pid"
  sleep 2
  echo "[start] MuJoCo pid=$(cat "$LOG_DIR/mujoco_r1lite.pid") node=/r1lite/mujoco_mirror"
}

start_record() {
  bash "$ROOT/lerobot_record/r1lite_record_daemon.sh" start
}

start_viewer() {
  source_ros
  kill_pidfile "$LOG_DIR/cam_view_triple.pid"
  nohup python3 "$ROOT/scripts/cam_view_triple.py" \
    --transport http \
    --http-base "$R1LITE_CAM_HTTP" \
    --api "$R1LITE_RECORD_API" \
    --repo "$R1LITE_REPO" \
    --data-root "$LOCAL_DATASET_ROOT" \
    > "$LOG_DIR/cam_view_triple.log" 2>&1 &
  echo $! > "$LOG_DIR/cam_view_triple.pid"
  sleep 2
  echo "[start] cam view pid=$(cat "$LOG_DIR/cam_view_triple.pid") window='R1Lite Record | head + left_wrist + right_wrist'"
}

start_all() {
  echo "[start] project=$ROOT"
  check_robot_ports || true
  start_mujoco
  start_record
  start_viewer
  echo "[start] complete — use START/STOP on the image window, or episode-start/stop"
  cmd_status
}

cmd_status() {
  echo "=== R1Lite sim + record status ==="
  if [[ -f "$LOG_DIR/mujoco_r1lite.pid" ]] && kill -0 "$(cat "$LOG_DIR/mujoco_r1lite.pid")" 2>/dev/null; then
    echo "MuJoCo:  running pid=$(cat "$LOG_DIR/mujoco_r1lite.pid")"
  else
    echo "MuJoCo:  not running"
  fi
  if [[ -f "$LOG_DIR/cam_view_triple.pid" ]] && kill -0 "$(cat "$LOG_DIR/cam_view_triple.pid")" 2>/dev/null; then
    echo "Viewer:  running pid=$(cat "$LOG_DIR/cam_view_triple.pid")"
  else
    echo "Viewer:  not running"
  fi
  bash "$ROOT/lerobot_record/r1lite_record_daemon.sh" status || true
  check_robot_ports || true
  echo "dataset: $LOCAL_DATASET_ROOT/$R1LITE_REPO"
}

episode_start() {
  local repo="$R1LITE_REPO"
  local task="r1lite teleop"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --repo) repo="$2"; shift 2 ;;
      --task) task="$2"; shift 2 ;;
      *) echo "unknown arg: $1"; exit 1 ;;
    esac
  done
  curl -sS -X POST "$R1LITE_RECORD_API/record/start" \
    -H 'Content-Type: application/json' \
    -d "{\"repo\":\"$repo\",\"task\":\"$task\"}"
  echo
}

episode_stop() {
  curl -sS -X POST "$R1LITE_RECORD_API/record/stop" \
    -H 'Content-Type: application/json' \
    -d '{}'
  echo
}

episode_status() {
  curl -sS "$R1LITE_RECORD_API/record/status"
  echo
}

case "${1:-}" in
  start) start_all ;;
  stop) stop_all ;;
  status) cmd_status ;;
  restart) stop_all; sleep 1; start_all ;;
  episode-start) shift; episode_start "$@" ;;
  episode-stop) episode_stop ;;
  episode-status) episode_status ;;
  *) usage; exit 1 ;;
esac
