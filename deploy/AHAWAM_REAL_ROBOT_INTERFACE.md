# AHA-WAM 真机策略接口（对接 R1Lite）

本文整理策略服务器协议，并标注与 **Galaxea R1Lite** 观测/控制的差异与待对齐项。

> pickle + TCP 仅限可信内网；服务须 `--host 0.0.0.0`。

---

## 1. 策略服务器

| 项 | 值 |
|----|-----|
| SSH 入口 | `ssh -p 32496 a26413@10.239.121.25` |
| 网关 | `10.239.121.25`（仅映射 SSH `32496`，**不**映射 `10000`） |
| Pod 内网 | `10.42.x.x`（会变；进程在容器内 `0.0.0.0:10000`） |
| 策略端口 | **`10000`**（`--host 0.0.0.0 --port 10000`） |
| 推荐 conda | `ahawam`（勿用降级到 torch 2.4.1 的 `ahawam-robotwin`） |

登录策略机：

```bash
ssh -p 32496 a26413@10.239.121.25
```

启动（服务器上）：

```bash
cd /home/a26413/AHA-WAM
source /home/a26413/miniconda3/etc/profile.d/conda.sh
conda activate ahawam
export DIFFSYNTH_MODEL_BASE_PATH=/home/a26413/model-cache
export DIFFSYNTH_SKIP_DOWNLOAD=true
python -m deploy.server.wam_policy_server \
  --policy-path deploy/deploy.yml \
  --policy-module deploy.server.ahawam_policy \
  --policy-class AHAWAMPolicy \
  --instruction "pick up the red block" \
  --action-dim 14 \
  --host 0.0.0.0 \
  --port 10000
```

`deploy/deploy.yml` 指向：

- `checkpoint_path: checkpoints/AHA-WAM-RoboTwin2.0/robotwin_ahawam.pt`
- `dataset_stats_path: checkpoints/AHA-WAM-RoboTwin2.0/dataset_stats.json`
- `project_root: /home/a26413/AHA-WAM`

**网络（推荐：R1 本机链路，不经 125）：** 网关只放行 SSH `32496`，直连 `10.239.121.25:10000` 会 refused。  
在 **r1lite@10.229.66.95** 上建隧道，推理客户端连本机：

```bash
# 机器人上常驻隧道
python3 scripts/ahawam_policy_ssh_tunnel.py --listen-host 127.0.0.1
# tmux: awm_r1lite_policy_tunnel

# 本机观测 + 本机隧道 → 策略（dry-run）
python3 scripts/ahawam_r1lite_dryrun_client.py \
  --server-ip 127.0.0.1 --server-port 10000 \
  --http-cam http://127.0.0.1:8766 \
  --tcp-joints 127.0.0.1:8765 \
  --once

# 或 policy_bridge（同样在机器人上）
python3 scripts/ahawam_r1lite_policy_bridge.py \
  --server-ip 127.0.0.1 --server-port 10000 \
  --http-cam http://127.0.0.1:8766
```

---

## 2. 传输协议（TCP + pickle）

帧格式：`4 字节大端长度 + pickle payload`（见上游 `deploy/common/tcp_protocol.py`）。

### 请求 `infer`

```python
{
    "type": "infer",
    "instruction": str,
    "state": np.ndarray,                 # float32 [14]
    "images": {"front": np.ndarray},     # uint8 [H,W,3] RGB
}
```

- 服务端将图像缩放到 **384×320**，归一化到 `[-1, 1]`
- **真机只读 `images["front"]` 单路**（仿真三路；部署单路）
- `state` 按 `dataset_stats.json` 归一化

### 响应 `action_chunk`

```python
{
    "ok": True,
    "type": "action_chunk",
    "action_chunk": np.ndarray,   # float32 [T,14], T=16
    "model_latency_ms": float,
    "server_inference_step": int,
}
```

- `action_chunk` 已反归一化；冒烟约 ~857 ms/chunk

### 14 维约定（RoboTwin / Piper）

```text
[左臂6关节, 左夹爪, 右臂6关节, 右夹爪]
```

顺序必须与 `dataset_stats.json` 一致。

---

## 3. 上游默认 ROS 客户端（Piper）

`deploy/client/wam_remote_client_node.py` 默认话题面向 **双 Piper**，**不能直接套在 R1Lite 上**：

| 方向 | 默认话题 |
|------|----------|
| 前视图 | `/camera_f/color/image_raw` |
| 左/右状态 | `/puppet/joint_left` · `/puppet/joint_right` |
| 左/右指令 | `/master/joint_left` · `/master/joint_right` |

安全参数：`--max-joint-delta 0.05`、`--dry-run` 等。务必先 dry-run。

---

## 4. R1Lite 映射（本仓库）

### 4.1 建议的 14 维 state/action 抽取

从 `/r1lite/joint_states` 或 TCP `10.229.66.95:8765`：

| 维 | R1Lite 关节 |
|----|-------------|
| 0..5 | `left_arm_joint1` … `left_arm_joint6` |
| 6 | 左夹爪标量（见下） |
| 7..12 | `right_arm_joint1` … `right_arm_joint6` |
| 13 | 右夹爪标量 |

**夹爪（待标定）：** 真机为双指 `*_gripper_finger_joint1/2`。部署前需确认与训练统计一致的标量定义，例如：

- `g = 0.5 * (finger1 + finger2)`，或
- 仅用 `finger1`，或
- 经线性映射到 Piper 的 `[gripper_min, gripper_max]`（上游默认约 `0.0 .. 0.07`）

未标定前，dry-run 客户端用 `0.5*(f1+f2)` 并打印，**禁止实控**。

### 4.2 前视相机 → `images["front"]`

| 用途 | R1Lite 源 |
|------|-----------|
| 推荐 | 头左：`/hdas/camera_head/left_raw/image_raw_color/compressed` |
| HTTP 中继 | `http://10.229.66.95:8766/stream/head` → RGB |

腕部相机不送策略服务器（协议仅 `front`）。

### 4.3 控制输出（未接实控）

R1Lite **没有** `/master/joint_*`。实控需走 Galaxea HDAS / mobiman 安全接口，本仓库当前：

- 只做 **观测 + 推理 dry-run**（见 `scripts/ahawam_r1lite_dryrun_client.py`）
- **不发布** 关节指令，直到限位、夹爪映射、急停流程确认

### 4.4 接真机前检查清单

1. **14 维顺序 + 夹爪标量** 与 `dataset_stats.json` 对齐  
2. **前视视角** 尽量接近训练分布（头相机高度/朝向）  
3. **网络** `nc -vz 10.239.121.25 10000`（策略）；SSH 运维用 `-p 32496`  
4. **先 dry-run**：检查 `action_chunk` 数值范围与跳变，再考虑实控  
5. 准备急停；勿把策略服务暴露公网  

---

## 5. 本仓库相关脚本

| 脚本 | 作用 |
|------|------|
| `scripts/ahawam_r1lite_policy_bridge.py` | **真机桥**：观测→推理→(可选)发指令 + 会话动作流 |
| `scripts/session_sim_video_record.py` | **125**：三路 MP4 + joints/actions 日志 |
| `start_r1lite_infer_session.sh` | 125 一键：MuJoCo 镜像真机 + 视频录制 |
| `scripts/mock_ahawam_policy_server.py` | 本地 mock 策略（服务未起时联调） |
| `scripts/ahawam_r1lite_dryrun_client.py` | 轻量单次推理探测 |
| `scripts/start_robot_cameras_and_relays.sh` | 机器人侧相机 + MJPEG/关节中继 |

### 推荐流程

**A. 125（仿真抓取真机 + 存视频）**

```bash
bash ~/r1lite_sim_record/start_r1lite_infer_session.sh start
# 停止： bash ~/r1lite_sim_record/start_r1lite_infer_session.sh stop
# 产物： /home/yao/r1lite_lerobot_datasets/sessions/session_YYYYMMDD_HHMMSS/
```

**B. 机器人（推理桥，默认不发控制）**

```bash
# 相机+关节中继
bash ~/r1lite_mujoco_sync/scripts/start_robot_cameras_and_relays.sh

# mock（策略机 10.239.121.25:10000 未起时）
python3 ~/r1lite_sim_record/scripts/ahawam_r1lite_policy_bridge.py \
  --mock-policy \
  --http-cam http://127.0.0.1:8766 \
  --session-host 10.229.20.125

# 真策略（服务已 --host 0.0.0.0 --port 10000）
python3 ~/r1lite_sim_record/scripts/ahawam_r1lite_policy_bridge.py \
  --server-ip 10.239.121.25 --server-port 10000 \
  --http-cam http://127.0.0.1:8766 \
  --session-host 10.229.20.125 \
  --instruction "pick up the red block"

# 确认 dry-run 合理后，才加 --enable-cmd（会发 motion_target / 夹爪）
```

指令话题（`--enable-cmd`）：

- `/motion_target/target_joint_state_arm_{left,right}` (`sensor_msgs/JointState`)
- `/motion_control/position_control_gripper_{left,right}` (`std_msgs/Float32`)
- 可选 `--cmd-mode hdas` → `/motion_control/control_arm_*` (`hdas_msg/MotorControl`)

> 当前 jointTracker 若将臂指令 remap 到 `/disabled/...`，`--enable-cmd` 的 target 可能不会驱动电机；实控前需恢复安全可控的 tracker/HDAS 配置并准备急停。
