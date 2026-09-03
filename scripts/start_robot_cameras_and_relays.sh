#!/bin/bash
# Bring up R1Lite camera drivers + MJPEG/joint TCP relays on the robot.
# Safe: does not publish actuator commands.
set -euo pipefail

HDAS_DIR="/home/r1lite/galaxea/install/startup_config/share/startup_config/script/boot/modules/hdas"
RELAY_DIR="${RELAY_DIR:-/home/r1lite/r1lite_mujoco_sync/scripts}"
LOG_DIR="${LOG_DIR:-/home/r1lite/r1lite_mujoco_sync/logs}"
mkdir -p "$LOG_DIR" "$RELAY_DIR"

start_tmux() {
  local name="$1"
  local cmd="$2"
  tmux has-session -t "$name" 2>/dev/null && tmux kill-session -t "$name" || true
  tmux new-session -d -s "$name" "bash -lc '$cmd; echo EXIT=\$?; sleep 5'"
  echo "[start] tmux $name"
}

echo 1 | sudo -S chmod a+rw /dev/video* /dev/media* 2>/dev/null || true

# Cameras
start_tmux awm_v23_camera_head \
  "cd '$HDAS_DIR' && bash ./start_r1lite_camera_head_safe.sh"
sleep 2
start_tmux awm_v22_left_wrist_camera \
  "cd '$HDAS_DIR' && bash ./start_realsense_camera_r1lite_left_wrist_safe.sh"
sleep 2
start_tmux awm_v24_right_wrist_camera \
  "cd '$HDAS_DIR' && bash ./start_realsense_camera_r1lite_right_wrist_safe.sh"
sleep 5

# Relays
start_tmux awm_r1lite_joint_tcp \
  "unset FASTRTPS_DEFAULT_PROFILES_FILE; unset ROS_DISCOVERY_SERVER; source /opt/ros/humble/setup.bash; source /home/r1lite/galaxea/install/setup.bash; export ROS2CLI_DISABLE_DAEMON=1; python3 '$RELAY_DIR/relay_joint_states_tcp.py' --port 8765"

start_tmux awm_r1lite_camera_mjpeg \
  "unset FASTRTPS_DEFAULT_PROFILES_FILE; unset ROS_DISCOVERY_SERVER; source /opt/ros/humble/setup.bash; source /home/r1lite/galaxea/install/setup.bash; export ROS2CLI_DISABLE_DAEMON=1; python3 '$RELAY_DIR/relay_cameras_mjpeg.py' --port 8766"

sleep 4
echo "=== tmux ==="
tmux ls | grep -E 'awm_v2|awm_r1lite' || true
echo "=== ports ==="
ss -lntp 2>/dev/null | grep -E '8765|8766' || netstat -lntp 2>/dev/null | grep -E '8765|8766' || true
echo "=== camera topics (sample) ==="
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_DISCOVERY_SERVER
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/r1lite/galaxea/install/setup.bash
export ROS2CLI_DISABLE_DAEMON=1
timeout 8 ros2 topic list 2>/dev/null | grep -E 'camera_head|camera_wrist|joint_states' | head -30 || true
