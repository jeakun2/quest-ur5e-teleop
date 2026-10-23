"""Gripper back-ends driven by the index trigger (0..1).

* ``RobotiqUrcapGripper``: Robotiq 2F-85/140 through the URCap socket server
  on the robot (port 63352).  Position 0 (open) .. 255 (closed).
* ``IsaacGripper``: the simulated Robotiq 2F-85 of isaac/ur5e_sim.py (auto-selected with robot.type isaac).
* ``NullGripper``: does nothing (default).
"""
from __future__ import annotations

import socket
import time


class NullGripper:
    name = "none"
    def connect(self): ...
    def disconnect(self): ...
    def command(self, closed: float) -> None: ...


class RobotiqUrcapGripper:
    name = "robotiq_urcap"

    def __init__(self, ip: str, port: int = 63352, speed: int = 255, force: int = 100,
                 min_change: float = 0.02, max_hz: float = 20.0) -> None:
        self.ip, self.port, self.speed, self.force = ip, port, speed, force
        self.min_change, self.min_period = min_change, 1.0 / max_hz
        self._sock: socket.socket | None = None
        self._last_val = -1.0
        self._last_t = 0.0

    def _cmd(self, s: str) -> str:
        self._sock.sendall((s + "\n").encode())
        return self._sock.recv(256).decode(errors="ignore").strip()

    def connect(self) -> None:
        self._sock = socket.create_connection((self.ip, self.port), timeout=2.0)
        if "0" in self._cmd("GET STA")[-1:]:  # not activated
            self._cmd("SET ACT 1")
            for _ in range(40):
                time.sleep(0.25)
                if self._cmd("GET STA").endswith("3"):
                    break
        self._cmd(f"SET SPE {self.speed}")
        self._cmd(f"SET FOR {self.force}")
        self._cmd("SET GTO 1")
        print(f"[gripper] robotiq urcap at {self.ip}:{self.port} ready")

    def disconnect(self) -> None:
        if self._sock:
            self._sock.close()

    def command(self, closed: float) -> None:
        now = time.monotonic()
        if abs(closed - self._last_val) < self.min_change or now - self._last_t < self.min_period:
            return
        self._last_val, self._last_t = closed, now
        pos = int(max(0.0, min(1.0, closed)) * 255)
        self._cmd(f"SET POS {pos}")


class IsaacGripper:
    """Robotiq 2F-85 inside Isaac Sim: the command rides on the robot's socket (IsaacSimRobot.set_gripper)."""
    name = "isaac"

    def __init__(self, robot) -> None:
        self._robot = robot

    def connect(self): ...
    def disconnect(self): ...

    def command(self, closed: float) -> None:
        self._robot.set_gripper(closed)


def make_gripper(cfg: dict, robot=None):
    kind = (cfg or {}).get("type", "none")
    if kind == "robotiq_urcap":
        return RobotiqUrcapGripper(cfg["ip"], cfg.get("port", 63352), cfg.get("speed", 255), cfg.get("force", 100))
    if kind == "isaac" or (kind == "none" and robot is not None and getattr(robot, "name", "") == "isaac"):
        if robot is None or not hasattr(robot, "set_gripper"):
            raise SystemExit("gripper.type isaac needs robot.type isaac")
        return IsaacGripper(robot)
    return NullGripper()
