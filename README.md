# R1Lite Sim + Record

独立项目：Galaxea **R1Lite** MuJoCo 仿真镜像 + 三路相机预览 + LeRobot 采集。  
与 Franka 遥操/采集栈完全隔离（不碰 Desk、不碰 `:8765` Franka record、不订 `/joint_states`）。

## 一键启动（125）

```bash
cd ~/r1lite_sim_record
bash start_r1lite_sim_record.sh start    # 仿真 + 图像窗 + 录制服务
bash start_r1lite_sim_record.sh status
bash start_r1lite_sim_record.sh stop
```

图像窗 `R1Lite Record | head + left_wrist + right_wrist` 底栏：**START / STOP / REPLAY / ABORT**  
或：

```bash
bash start_r1lite_sim_record.sh episode-start --repo r1lite_teleop --task "demo"
bash start_r1lite_sim_record.sh episode-stop
```

## 命名隔离

| 项 | R1Lite | Franka |
|----|--------|--------|
| 项目目录 | `~/r1lite_sim_record` | `~/gello_desk/lerobot_record` 等 |
| MuJoCo 节点 | `/r1lite/mujoco_mirror` | `/mujoco_franka_mirror` |
| 预览节点 | `/r1lite/cam_view` | `lerobot_dual_cam_view` |
| 录制桥 | `/r1lite/record_bridge` | （Franka record_server 内） |
| 录制 API | `http://127.0.0.1:8775` | `:8765` |
| 数据目录 | `/home/yao/r1lite_lerobot_datasets/` | `/home/yao/lerobot_datasets/` |
| 默认 repo | `r1lite_teleop` | `fr3_gello_teleop` |
| 关节 | TCP `10.229.66.95:8765` 或 `/r1lite/joint_states` | `/joint_states` 等 |

## 机器人侧（先于 125 启动）

在 `r1lite@10.229.66.95`：

1. **关节 TCP**（端口 8765）— tmux `awm_r1lite_joint_tcp`
2. **相机 MJPEG**（端口 8766）— tmux `awm_r1lite_camera_mjpeg`

也可从本机/125：`bash scripts/start_joint_relay_on_robot.sh`

## 目录

```
r1lite_sim_record/
  start_r1lite_sim_record.sh   # 仿真+采集一键脚本
  r1lite.mujoco.urdf
  meshes/
  scripts/
    mujoco_ros_mirror_r1lite.py
    cam_view_triple.py
    relay_joint_states_tcp.py      # 跑在机器人
    relay_cameras_mjpeg.py         # 跑在机器人
    start_mujoco_r1lite.sh
    start_cam_view_triple.sh
    start_joint_relay_on_robot.sh
  lerobot_record/
    r1lite_record_server.py        # :8775
    r1lite_record_daemon.sh
    ros_bridge_r1lite.py
    cam_replay_r1lite.py
```

## 说明

- **只观测，不发控制指令。**
- 当前 episode 的 `action` 暂与关节 `state` 相同（尚无独立遥操动作话题）。
- `ABORT` 只停本项目的仿真/预览/录制，不影响 Franka。

## AHA-WAM 真机策略接口

见 [`deploy/AHAWAM_REAL_ROBOT_INTERFACE.md`](deploy/AHAWAM_REAL_ROBOT_INTERFACE.md)。

```bash
# 125：仿真镜像真机动作 + 保存三路相机视频
bash start_r1lite_infer_session.sh start

# 机器人：推理桥（默认 dry-run；策略未起时加 --mock-policy）
python3 scripts/ahawam_r1lite_policy_bridge.py --mock-policy --session-host 10.229.20.125
```
