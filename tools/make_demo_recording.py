"""Write a synthetic controller recording (JSONL) for ``--source.type replay``.

Gesture (right hand, XR frame): grip, trace a 10 cm square in the horizontal
plane (forward / left / back / right) over 8 s, release, 1 s pause, then
grip and lift 8 cm, release.  Useful for exercising the robot layer
(dummy or URSim) without a headset.

    python tools/make_demo_recording.py runs/demo.jsonl
"""
import json
import math
import os
import sys

import numpy as np

HZ = 72


def seg(t, t0, t1, a, b):
    """linear blend a->b with smoothstep over [t0, t1]."""
    if t <= t0:
        return np.array(a, float)
    if t >= t1:
        return np.array(b, float)
    s = (t - t0) / (t1 - t0)
    s = s * s * (3 - 2 * s)
    return np.array(a, float) + s * (np.array(b, float) - np.array(a, float))


def pose_at(t):
    base = np.array([0.0, 1.0, 0.0])
    if t < 0.5:
        return base, 0.0
    if t < 2.5:
        return base + seg(t, 0.5, 2.5, [0, 0, 0], [0, 0, -0.10]), 1.0
    if t < 4.5:
        return base + seg(t, 2.5, 4.5, [0, 0, -0.10], [-0.10, 0, -0.10]), 1.0
    if t < 6.5:
        return base + seg(t, 4.5, 6.5, [-0.10, 0, -0.10], [-0.10, 0, 0]), 1.0
    if t < 8.5:
        return base + seg(t, 6.5, 8.5, [-0.10, 0, 0], [0, 0, 0]), 1.0
    if t < 9.5:
        return base, 0.0
    if t < 11.5:
        return base + seg(t, 9.5, 11.5, [0, 0, 0], [0, 0.08, 0]), 1.0
    return base + [0, 0.08, 0], 0.0


def main(path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    n = int(12.5 * HZ)
    with open(path, "w", encoding="utf-8") as f:
        for i in range(n):
            t = i / HZ
            p, grip = pose_at(t)
            # small wrist yaw oscillation while gripping to exercise rotation mapping
            yaw = 0.15 * math.sin(2 * math.pi * 0.25 * t) if grip > 0.5 else 0.0
            q = [0.0, math.sin(yaw / 2), 0.0, math.cos(yaw / 2)]
            frame = {"t": t, "right": {"p": p.round(5).tolist(), "q": q, "grip": grip, "trigger": 0.3 if 4 < t < 6 else 0.0,
                                      "a": False, "b": False, "axes": [0, 0], "tracked": True}, "left": None}
            f.write(json.dumps(frame) + "\n")
    print(f"wrote {n} frames to {path}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "runs/demo.jsonl")
