# R1Lite MuJoCo Sync (125)

Observation-only MuJoCo mirror for Galaxea **R1Lite**, prepared the same way as Franka on `yao@10.229.20.125`.

## Naming rules (do not collide with Franka)

| Kind | R1Lite | Franka (unchanged) |
|------|--------|---------------------|
| Mirror node | `/r1lite/mujoco_mirror` | `/mujoco_franka_mirror` |
| Joint ROS topic | `/r1lite/joint_states` | `/joint_states` / `/franka_robot_state_broadcaster/...` |
| Relay node (robot) | `/r1lite/joint_tcp_relay` | — |
| TCP fallback | `10.229.66.95:8765` | FCI `read_franka_q` |

R1 stack **never publishes or subscribes bare `/joint_states`** on the 125 graph. Franka subscriptions stay as-is.

| Franka | R1Lite |
|---------|--------|
| `~/franka_mujoco_sync` | `~/r1lite_mujoco_sync` |
| `mujoco_ros_mirror.py --source franka` | `scripts/mujoco_ros_mirror_r1lite.py` |
| `start_mujoco_gpu.sh` | `scripts/start_mujoco_r1lite.sh` |
| FCI `read_franka_q` fallback | TCP joint relay from robot |

**Does not command the robot.** Viewer only.

## Layout

```
~/r1lite_mujoco_sync/
  r1lite.mujoco.urdf          # mesh paths rewritten for MuJoCo
  r1lite.raw.urdf             # original package:// URDF
  meshes/                     # STL assets from robot
  scripts/
    mujoco_ros_mirror_r1lite.py
    relay_joint_states_tcp.py # run ON r1lite@10.229.66.95
    start_mujoco_r1lite.sh    # run ON 125
    start_joint_relay_on_robot.sh
  logs/
```

## Why TCP relay?

Robot `10.229.66.95` and host `10.229.20.125` are different L3 subnets. DDS multicast from the robot usually does not reach 125. The TCP relay (port **8765**) is the cross-subnet fallback, analogous to Franka’s FCI `read_franka_q`.

## Bringup

### 1. On robot (`r1lite@10.229.66.95`) — joint relay

Robot stack must already be publishing `/joint_states` (HDAS / jointTracker).

```bash
# copy once from 125, or use start script below
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source /home/r1lite/galaxea/install/setup.bash
python3 /home/r1lite/r1lite_mujoco_sync/scripts/relay_joint_states_tcp.py --port 8765
```

### 2. On 125 (`yao@10.229.20.125`) — MuJoCo viewer

```bash
cd ~/r1lite_mujoco_sync
bash scripts/start_mujoco_r1lite.sh
```

Viewer uses `DISPLAY=:1` and `MUJOCO_GL=glfw` (same as Franka).

### 3. Smoke checks

```bash
# 125: model loads
python3 -c "import mujoco; m=mujoco.MjModel.from_xml_path('r1lite.mujoco.urdf'); print('nq', m.nq, 'njnt', m.njnt)"

# 125: TCP open
nc -vz 10.229.66.95 8765

# robot: vendor joints + namespaced republish
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
ros2 topic hz /joint_states
ros2 topic hz /r1lite/joint_states
ros2 node list | grep r1lite
```

## Topics mirrored

`/r1lite/joint_states` (and TCP) names: chassis steer/wheel ×3, torso×3, left arm×6 + fingers×2, right arm×6 + fingers×2.

## Related

- Franka twin bringup: `~/franka_mujoco_digital_twin` / `docs/bringup_remote.md`
- Franka simple mirror: `~/gello_desk/mujoco_ros_mirror.py`
