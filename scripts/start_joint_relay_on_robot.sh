#!/bin/bash
# Deploy/start joint TCP relay on r1lite robot (from 125 or laptop).
set -euo pipefail
ROBOT_HOST="${ROBOT_HOST:-10.229.66.95}"
ROBOT_USER="${ROBOT_USER:-r1lite}"
REMOTE_DIR="${REMOTE_DIR:-/home/r1lite/r1lite_mujoco_sync}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "Sync scripts -> ${ROBOT_USER}@${ROBOT_HOST}:${REMOTE_DIR}"
sshpass -p '1' rsync -az -e "ssh -o StrictHostKeyChecking=no" \
  "$ROOT/scripts/relay_joint_states_tcp.py" \
  "${ROBOT_USER}@${ROBOT_HOST}:${REMOTE_DIR}/scripts/" 2>/dev/null || \
rsync -az -e "ssh -o StrictHostKeyChecking=no" \
  "$ROOT/scripts/relay_joint_states_tcp.py" \
  "${ROBOT_USER}@${ROBOT_HOST}:${REMOTE_DIR}/scripts/"

echo "Start relay in tmux awm_r1lite_joint_tcp_relay"
ssh -o StrictHostKeyChecking=no "${ROBOT_USER}@${ROBOT_HOST}" bash -s <<'REMOTE'
set -e
mkdir -p /home/r1lite/r1lite_mujoco_sync/scripts /home/r1lite/r1lite_mujoco_sync/logs
tmux kill-session -t awm_r1lite_joint_tcp_relay 2>/dev/null || true
tmux new-session -d -s awm_r1lite_joint_tcp_relay "bash -lc '
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  unset ROS_DISCOVERY_SERVER
  source /opt/ros/humble/setup.bash
  source /home/r1lite/galaxea/install/setup.bash
  export ROS2CLI_DISABLE_DAEMON=1
  exec python3 /home/r1lite/r1lite_mujoco_sync/scripts/relay_joint_states_tcp.py --port 8765
'"
sleep 2
tmux ls | grep awm_r1lite || true
ss -ltnp | grep 8765 || netstat -ltnp 2>/dev/null | grep 8765 || true
REMOTE
