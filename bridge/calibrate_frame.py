"""Derive ``mapping.r_map`` (XR frame -> robot base frame) from three gestures.

    python bridge/calibrate_frame.py [--source.type webxr|oculus_reader]

The operator stands where they will teleoperate, squeezes the grip, moves the
right controller ~30 cm in robot +X ("forward" along the robot's base X axis),
releases; repeats for robot +Y, then +Z.  The three motion directions are
orthonormalised into a rotation matrix and written to config.yaml.
"""
from __future__ import annotations

import os
import sys
import time

import numpy as np
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
from bridge.teleop_bridge import load_config, make_source  # noqa: E402

AXES = ["+X (robot base X)", "+Y (robot base Y)", "+Z (up)"]


def capture_motion(source, hand: str, label: str) -> np.ndarray:
    print(f"\n==> squeeze GRIP, move the {hand} controller ~30 cm along robot {label}, release.")
    start = None
    last = None
    while True:
        s = source.latest()
        c = (s.right if hand == "right" else s.left) if s else None
        if c is None:
            time.sleep(0.02)
            continue
        if c.grip > 0.5:
            if start is None:
                start = c.T[:3, 3].copy()
                print("    recording ...")
            last = c.T[:3, 3].copy()
        elif start is not None:
            v = last - start
            n = np.linalg.norm(v)
            if n < 0.08:
                print(f"    too short ({n*100:.0f} cm), try again")
                start = None
                continue
            print(f"    captured {n*100:.0f} cm")
            return v / n
        time.sleep(0.005)


def main() -> int:
    cfg, args = load_config()
    hand = cfg.get("mapping", {}).get("hand", "right")
    source = make_source(cfg.get("source", {}))
    source.start()
    print("waiting for controller ...")
    while source.latest() is None:
        time.sleep(0.1)
    vs = [capture_motion(source, hand, a) for a in AXES]
    source.stop()
    # columns of M are the XR directions of robot X, Y, Z;  R_map = M^T (orthonormalised)
    M = np.stack(vs, axis=1)
    U, _, Vt = np.linalg.svd(M)
    Rm = (U @ Vt).T
    if np.linalg.det(Rm) < 0:
        print("warning: left-handed result, flipping last axis")
        U[:, -1] *= -1
        Rm = (U @ Vt).T
    print("\nR_map =\n", np.round(Rm, 3))
    print("residual angle per axis (deg):",
          [round(float(np.degrees(np.arccos(np.clip(Rm[i] @ vs[i], -1, 1)))), 1) for i in range(3)])
    with open(args.config, encoding="utf-8") as f:
        full = yaml.safe_load(f)
    full.setdefault("mapping", {})["r_map"] = np.round(Rm, 6).tolist()
    full["mapping"]["operator_yaw_deg"] = 0
    with open(args.config, "w", encoding="utf-8") as f:
        yaml.safe_dump(full, f, sort_keys=False, allow_unicode=True)
    print(f"written to {args.config}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
