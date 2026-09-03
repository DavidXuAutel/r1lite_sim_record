#!/bin/bash
# Start R1Lite triple-cam viewer only (record daemon if needed).
# For sim + record together, use: bash ../start_r1lite_sim_record.sh start
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${LOG_DIR:-$ROOT/logs}"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash 2>/dev/null || true
export DISPLAY="${DISPLAY:-:1}"
export R1LITE_CAM_HTTP="${R1LITE_CAM_HTTP:-http://10.229.66.95:8766}"
export LOCAL_DATASET_ROOT="${LOCAL_DATASET_ROOT:-/home/yao/r1lite_lerobot_datasets}"
export R1LITE_RECORD_API="${R1LITE_RECORD_API:-http://127.0.0.1:8775}"
TRANSPORT="${1:-http}"

bash "$ROOT/lerobot_record/r1lite_record_daemon.sh" start || true

if [[ -f "$LOG_DIR/cam_view_triple.pid" ]]; then
  old="$(cat "$LOG_DIR/cam_view_triple.pid" 2>/dev/null || true)"
  if [[ -n "$old" ]]; then
    kill "$old" 2>/dev/null || true
    sleep 1
    kill -9 "$old" 2>/dev/null || true
  fi
fi

nohup python3 "$ROOT/scripts/cam_view_triple.py" \
  --transport "$TRANSPORT" \
  --http-base "$R1LITE_CAM_HTTP" \
  --api "$R1LITE_RECORD_API" \
  --repo "${R1LITE_REPO:-r1lite_teleop}" \
  --data-root "$LOCAL_DATASET_ROOT" \
  > "$LOG_DIR/cam_view_triple.log" 2>&1 &
echo $! > "$LOG_DIR/cam_view_triple.pid"
sleep 2
echo "R1Lite Record view pid=$(cat "$LOG_DIR/cam_view_triple.pid") api=$R1LITE_RECORD_API"
tail -15 "$LOG_DIR/cam_view_triple.log" || true
curl -sS "$R1LITE_RECORD_API/record/status" || true
echo
