#!/usr/bin/env bash
# Start/stop/status for R1Lite LeRobot record HTTP daemon (port 8775).
# Isolated from Franka: does NOT touch record_server.py / port 8765.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$DIR/.." && pwd)"
LOG_DIR="${LOG_DIR:-$ROOT/logs}"
PID_FILE="$LOG_DIR/r1lite_record.pid"
LOG_FILE="$LOG_DIR/r1lite_record.log"
ENV_FILE="${R1LITE_RECORD_ENV:-$DIR/.env}"
PROC_MATCH='python -u r1lite_record_server.py'

mkdir -p "$LOG_DIR"

activate_env() {
  if [[ -f /home/yao/anaconda3/etc/profile.d/conda.sh ]]; then
    # shellcheck disable=SC1091
    source /home/yao/anaconda3/etc/profile.d/conda.sh
    conda activate lerobot 2>/dev/null || true
  fi
  unset PYTHONPATH
  export PYTHONPATH="$DIR${PYTHONPATH:+:$PYTHONPATH}"

  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u

  export LOCAL_DATASET_ROOT="${LOCAL_DATASET_ROOT:-/home/yao/r1lite_lerobot_datasets}"
  export R1LITE_RECORD_PORT="${R1LITE_RECORD_PORT:-8775}"
  export R1LITE_RECORD_HOST="${R1LITE_RECORD_HOST:-127.0.0.1}"
  export R1LITE_CAM_HTTP="${R1LITE_CAM_HTTP:-http://10.229.66.95:8766}"
  export R1LITE_TCP_HOST="${R1LITE_TCP_HOST:-10.229.66.95}"
  export R1LITE_TCP_PORT="${R1LITE_TCP_PORT:-8765}"

  if [[ -f "$ENV_FILE" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$ENV_FILE"
    set +a
  fi
}

cmd_start() {
  if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "already running pid=$(cat "$PID_FILE")"
    exit 0
  fi
  activate_env
  cd "$DIR"
  nohup python -u r1lite_record_server.py >>"$LOG_FILE" 2>&1 &
  echo $! >"$PID_FILE"
  sleep 2
  if kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "started pid=$(cat "$PID_FILE") port=${R1LITE_RECORD_PORT} log=$LOG_FILE"
  else
    echo "failed to start; see $LOG_FILE" >&2
    tail -40 "$LOG_FILE" >&2 || true
    exit 1
  fi
}

cmd_stop() {
  if [[ -f "$PID_FILE" ]]; then
    pid=$(cat "$PID_FILE")
    kill "$pid" 2>/dev/null || true
    sleep 1
    kill -9 "$pid" 2>/dev/null || true
    rm -f "$PID_FILE"
    echo "stopped"
  else
    pkill -f "$PROC_MATCH" 2>/dev/null || true
    echo "stopped (no pid file)"
  fi
}

cmd_status() {
  if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "daemon running pid=$(cat "$PID_FILE")"
  else
    echo "daemon not running"
  fi
  host="${R1LITE_RECORD_HOST:-127.0.0.1}"
  port="${R1LITE_RECORD_PORT:-8775}"
  curl -sS "http://${host}:${port}/record/status" || true
  echo
}

case "${1:-}" in
  start) cmd_start ;;
  stop) cmd_stop ;;
  status) activate_env; cmd_status ;;
  restart) cmd_stop; sleep 1; cmd_start ;;
  *) echo "Usage: $0 {start|stop|status|restart}"; exit 1 ;;
esac
