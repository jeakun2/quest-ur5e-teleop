"""Kinematic stand-in for a UR5e: integrates servo targets with a first-order
lag so the bridge, mapping and safety logic can be exercised without a robot.
"""
from __future__ import annotations

import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from .base import Robot

HOME_TCP = np.array([0.40, -0.10, 0.35, 2.22, -2.22, 0.0])  # tool pointing down, in front of base


class DummyRobot(Robot):
    name = "dummy"

    def __init__(self, lag: float = 0.6, log_hz: float = 4.0) -> None:
        self._pose = HOME_TCP.copy()
        self._lag = lag
        self._last_log = 0.0
        self._log_period = 1.0 / log_hz
        self.servo_calls = 0

    def connect(self) -> None:
        print(f"[dummy] connected, TCP={np.round(self._pose, 3)}")

    def tcp_pose(self) -> np.ndarray:
        return self._pose.copy()

    def servo(self, pose: np.ndarray, dt: float) -> None:
        self.servo_calls += 1
        p = self._pose[:3] + self._lag * (pose[:3] - self._pose[:3])
        r0, r1 = R.from_rotvec(self._pose[3:]), R.from_rotvec(pose[3:])
        rel = (r0.inv() * r1).as_rotvec()
        r = r0 * R.from_rotvec(rel * self._lag)
        self._pose = np.concatenate([p, r.as_rotvec()])
        now = time.monotonic()
        if now - self._last_log > self._log_period:
            self._last_log = now
            print(f"[dummy] servo -> {np.round(self._pose[:3], 3)} rot={np.round(self._pose[3:], 2)}")

    def hold(self) -> None:
        pass
