#!/usr/bin/env python3
"""Shared 14-DoF pack/unpack for AHA-WAM ↔ Galaxea R1Lite."""

from __future__ import annotations

from typing import Iterable

import numpy as np

LEFT_ARM = [f"left_arm_joint{i}" for i in range(1, 7)]
RIGHT_ARM = [f"right_arm_joint{i}" for i in range(1, 7)]
LEFT_FINGERS = ("left_gripper_finger_joint1", "left_gripper_finger_joint2")
RIGHT_FINGERS = ("right_gripper_finger_joint1", "right_gripper_finger_joint2")

# Names used when publishing sensor_msgs/JointState arm targets
LEFT_ARM_CMD_NAMES = list(LEFT_ARM)
RIGHT_ARM_CMD_NAMES = list(RIGHT_ARM)

STATE14_NAMES = (
    LEFT_ARM
    + ["left_gripper"]
    + RIGHT_ARM
    + ["right_gripper"]
)


def gripper_from_fingers(f1: float, f2: float) -> float:
    """Map dual-finger joints to a single gripper scalar (training-compatible TBD)."""
    return float(0.5 * (f1 + f2))


def fingers_from_gripper(g: float) -> tuple[float, float]:
    """Inverse of gripper_from_fingers (symmetric open)."""
    g = float(g)
    return g, g


def pack_state14(q: dict[str, float]) -> np.ndarray | None:
    need = LEFT_ARM + list(LEFT_FINGERS) + RIGHT_ARM + list(RIGHT_FINGERS)
    if not all(n in q for n in need):
        return None
    lg = gripper_from_fingers(q[LEFT_FINGERS[0]], q[LEFT_FINGERS[1]])
    rg = gripper_from_fingers(q[RIGHT_FINGERS[0]], q[RIGHT_FINGERS[1]])
    return np.array(
        [q[n] for n in LEFT_ARM] + [lg] + [q[n] for n in RIGHT_ARM] + [rg],
        dtype=np.float32,
    )


def unpack_action14(a: Iterable[float]) -> dict[str, float]:
    """14-vector → joint name map (arms + both fingers per side)."""
    v = np.asarray(list(a), dtype=np.float32).reshape(-1)
    if v.shape[0] != 14:
        raise ValueError(f"expected 14-DoF action, got shape {v.shape}")
    out: dict[str, float] = {}
    for i, n in enumerate(LEFT_ARM):
        out[n] = float(v[i])
    lf1, lf2 = fingers_from_gripper(float(v[6]))
    out[LEFT_FINGERS[0]] = lf1
    out[LEFT_FINGERS[1]] = lf2
    for i, n in enumerate(RIGHT_ARM):
        out[n] = float(v[7 + i])
    rf1, rf2 = fingers_from_gripper(float(v[13]))
    out[RIGHT_FINGERS[0]] = rf1
    out[RIGHT_FINGERS[1]] = rf2
    out["left_gripper"] = float(v[6])
    out["right_gripper"] = float(v[13])
    return out


def clip_delta(curr: np.ndarray, target: np.ndarray, max_joint: float, max_grip: float) -> np.ndarray:
    """Limit per-step change (indices 6 and 13 are grippers)."""
    curr = np.asarray(curr, dtype=np.float32)
    target = np.asarray(target, dtype=np.float32)
    delta = target - curr
    out = curr.copy()
    for i in range(14):
        lim = max_grip if i in (6, 13) else max_joint
        d = float(np.clip(delta[i], -lim, lim))
        out[i] = curr[i] + d
    return out.astype(np.float32)
