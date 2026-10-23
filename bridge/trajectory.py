"""Recording and playback of TCP trajectories.

``Recorder`` appends one JSON line per bridge cycle in which a new controller
sample arrived: the controller sample (same format as ``--record`` /
``ReplaySource``) plus ``tcp`` (robot TCP after the cycle), ``engaged`` and
``gripper`` (0..1 trigger command).  A recording can therefore be replayed two
ways:

* ``--source.type replay`` feeds the *controller* samples through the mapper
  again (tests the mapping);
* ``TrajectoryPlayer`` replays the recorded *TCP* poses directly, after a
  rate-limited approach from wherever the robot currently is (reproduces the
  motion exactly).
"""
from __future__ import annotations

import json
import os
import time

import numpy as np
from scipy.spatial.transform import Rotation as R


class Recorder:
    def __init__(self, path: str, meta: dict | None = None) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.path = path
        self._f = open(path, "a", encoding="utf-8")
        self.frames = 0
        self.t_start = time.monotonic()
        if meta:
            self._f.write(json.dumps({"meta": meta}) + "\n")

    def write(self, sample_json: dict, tcp, engaged: bool, gripper: float) -> None:
        d = dict(sample_json)
        d["tcp"] = [round(float(v), 5) for v in tcp]
        d["engaged"] = bool(engaged)
        d["gripper"] = round(float(gripper), 3)
        self._f.write(json.dumps(d) + "\n")
        self.frames += 1

    @property
    def seconds(self) -> float:
        return time.monotonic() - self.t_start

    def close(self) -> None:
        self._f.close()


def load_tcp_frames(path: str) -> list[dict]:
    """Return the frames of a recording that carry a ``tcp`` field."""
    out = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            d = json.loads(line)
            if "tcp" in d:
                out.append(d)
    return out


def list_recordings(directory: str, limit: int = 5) -> list[str]:
    if not os.path.isdir(directory):
        return []
    files = [os.path.join(directory, n) for n in os.listdir(directory) if n.endswith(".jsonl")]
    files.sort(key=os.path.getmtime, reverse=True)
    return files[:limit]


def _step_toward(cur: np.ndarray, goal: np.ndarray, max_lin: float, max_ang: float, dt: float) -> tuple[np.ndarray, bool]:
    """One rate-limited step from pose ``cur`` toward ``goal``. Returns (pose, reached)."""
    dp = goal[:3] - cur[:3]
    n = float(np.linalg.norm(dp))
    lin_cap = max_lin * dt
    p = goal[:3] if n <= lin_cap else cur[:3] + dp * (lin_cap / n)
    r0, r1 = R.from_rotvec(cur[3:]), R.from_rotvec(goal[3:])
    rel = (r0.inv() * r1).as_rotvec()
    ang = float(np.linalg.norm(rel))
    ang_cap = max_ang * dt
    r = r1 if ang <= ang_cap else r0 * R.from_rotvec(rel * (ang_cap / ang))
    reached = n <= lin_cap and ang <= ang_cap
    return np.concatenate([p, r.as_rotvec()]), reached


class TrajectoryPlayer:
    """Replays recorded TCP poses.  Phase 1 approaches the first pose at the
    speed limits; phase 2 follows the recording in real time (``speed`` x)."""

    def __init__(self, frames: list[dict], max_lin_speed: float = 0.25, max_ang_speed: float = 1.0,
                 speed: float = 1.0) -> None:
        if not frames:
            raise ValueError("empty recording")
        self.frames = frames
        self.max_lin, self.max_ang, self.speed = max_lin_speed, max_ang_speed, speed
        self.phase = "approach"
        self._t0_rec = frames[0]["t"]
        self._t0_wall: float | None = None
        self._i = 0
        self._last: np.ndarray | None = None
        self.done = False

    def gripper(self) -> float | None:
        if self.phase != "play":
            return None
        return self.frames[min(self._i, len(self.frames) - 1)].get("gripper")

    def progress(self) -> float:
        return 0.0 if self.phase == "approach" else self._i / max(1, len(self.frames) - 1)

    def step(self, current_tcp: np.ndarray, dt: float, now: float | None = None) -> np.ndarray | None:
        """Return the pose to servo this cycle (None when finished)."""
        if self.done:
            return None
        now = time.monotonic() if now is None else now
        cur = np.asarray(current_tcp, dtype=float) if self._last is None else self._last
        if self.phase == "approach":
            goal = np.asarray(self.frames[0]["tcp"], dtype=float)
            pose, reached = _step_toward(cur, goal, self.max_lin, self.max_ang, dt)
            if reached:
                self.phase = "play"
                self._t0_wall = now
            self._last = pose
            return pose
        # play: advance index to the recorded time, interpolate linearly between frames
        el = (now - self._t0_wall) * self.speed
        while self._i + 1 < len(self.frames) and self.frames[self._i + 1]["t"] - self._t0_rec <= el:
            self._i += 1
        if self._i + 1 >= len(self.frames):
            self.done = True
            goal = np.asarray(self.frames[-1]["tcp"], dtype=float)
        else:
            a, b = self.frames[self._i], self.frames[self._i + 1]
            ta, tb = a["t"] - self._t0_rec, b["t"] - self._t0_rec
            u = 0.0 if tb <= ta else float(np.clip((el - ta) / (tb - ta), 0.0, 1.0))
            pa, pb = np.asarray(a["tcp"], dtype=float), np.asarray(b["tcp"], dtype=float)
            ra, rb = R.from_rotvec(pa[3:]), R.from_rotvec(pb[3:])
            rel = (ra.inv() * rb).as_rotvec()
            goal = np.concatenate([pa[:3] + u * (pb[:3] - pa[:3]), (ra * R.from_rotvec(rel * u)).as_rotvec()])
        pose, _ = _step_toward(cur, goal, self.max_lin, self.max_ang, dt)   # never exceed the limits
        self._last = pose
        return pose
