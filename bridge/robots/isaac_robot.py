"""UR5e in Isaac Sim (isaac/ur5e_sim.py) as a bridge robot back-end.

The sim is a small TCP/JSON-lines server; every bridge cycle does one request/reply
(localhost round trip ~0.2 ms).  Poses are UR "Base"-frame [x,y,z,rx,ry,rz] exactly
like ur_rtde, so mapping/calibration settings carry over to a real UR5e.
The gripper command (0..1, from the trigger) rides along on the same messages.
"""
from __future__ import annotations

import json
import socket
import time

import numpy as np

from .base import Robot


class IsaacSimRobot(Robot):
    name = "isaac"

    def __init__(self, host: str = "127.0.0.1", port: int = 5560, timeout: float = 1.0,
                 connect_wait: float = 120.0, reconnect_every: float = 2.0) -> None:
        self.host, self.port, self.timeout, self.connect_wait = host, port, timeout, connect_wait
        self.reconnect_every = reconnect_every     # after a link error: retry this often (s), never block
        self._next_reconnect = 0.0
        self.reconnects = 0
        self._sock: socket.socket | None = None
        self._rf = None
        self._tcp = np.zeros(6)
        self._q: np.ndarray | None = None
        self._last: dict = {}
        self._err = ""
        self._servoing = False
        self._grip = 0.0
        self._grip_sent = -1.0
        self._pending: dict = {}      # cam / view requests riding on the next message

    # ------------------------------------------------------------ transport
    def connect(self) -> None:
        deadline = time.monotonic() + self.connect_wait
        while True:
            try:
                self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
                break
            except OSError as e:
                if time.monotonic() > deadline:
                    raise SystemExit(f"[isaac] cannot connect to the sim at {self.host}:{self.port} ({e}). "
                                     f"Start it first:  python tools/isaac_sim.py --headless")
                time.sleep(1.0)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.settimeout(self.timeout)
        self._rf = self._sock.makefile("rb")
        self._err = ""
        rep = self._rpc({"get": 1})
        print(f"[isaac] connected to {self.host}:{self.port}, sim {rep.get('hz', 0)} Hz, "
              f"TCP={np.round(self._tcp, 3)}")

    def disconnect(self) -> None:
        try:
            if self._sock:
                self.hold()
                self._sock.close()
        except Exception:
            pass
        self._sock = None

    def _try_reconnect(self) -> bool:
        """Called while the link is broken (sim frozen/restarted): drop the dead socket and try a fresh
        connection at most every `reconnect_every` s, with a short timeout so the bridge loop never stalls.
        On success the error clears, the gripper state is re-sent and the robot reports ok() again."""
        now = time.monotonic()
        if now < self._next_reconnect:
            return False
        self._next_reconnect = now + self.reconnect_every
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass
        self._sock, self._rf = None, None
        try:
            sock = socket.create_connection((self.host, self.port), timeout=min(0.3, self.timeout))
        except OSError:
            return False
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(self.timeout)
        self._sock, self._rf = sock, sock.makefile("rb")
        self._err = ""
        self._grip_sent = -1.0
        self._servoing = False
        try:
            self._rpc({"get": 1})
        except Exception:
            return False
        self.reconnects += 1
        print(f"[isaac] reconnected to the sim at {self.host}:{self.port} (#{self.reconnects}), "
              f"TCP={np.round(self._tcp, 3)}")
        return True

    def _rpc(self, msg: dict) -> dict:
        if self._sock is None:
            raise RuntimeError("not connected")
        if self._grip != self._grip_sent:
            msg = dict(msg, gripper=round(self._grip, 3))
        if self._pending:
            msg = dict(msg, **self._pending)
            self._pending = {}
        try:
            self._sock.sendall((json.dumps(msg) + "\n").encode())
            line = self._rf.readline()
            if not line:
                raise ConnectionError("sim closed the connection")
            rep = json.loads(line)
        except Exception as e:
            self._err = f"isaac sim link: {e}"
            raise
        if "gripper" in msg:
            self._grip_sent = self._grip
        self._last = rep
        if "tcp" in rep:
            self._tcp = np.asarray(rep["tcp"], dtype=float)
        if "q" in rep:
            self._q = np.asarray(rep["q"], dtype=float)
        return rep

    # ------------------------------------------------------------ Robot API
    def tcp_pose(self) -> np.ndarray:
        if self._err and not self._try_reconnect():
            return self._tcp.copy()
        try:
            self._rpc({"get": 1})
        except Exception:
            pass
        return self._tcp.copy()

    def joints(self) -> np.ndarray | None:
        return None if self._q is None else self._q.copy()

    def servo(self, pose: np.ndarray, dt: float) -> None:
        if self._err and not self._try_reconnect():
            return
        try:
            self._rpc({"servo": [float(v) for v in pose]})
            self._servoing = True
        except Exception:
            pass

    def hold(self) -> None:
        if self._sock is None or self._err:
            return
        try:
            self._rpc({"hold": 1})
        except Exception:
            pass
        self._servoing = False

    def ok(self) -> tuple[bool, str]:
        if self._err:
            return False, self._err + " (reconnecting)"
        if self._sock is None:
            return False, "isaac sim not connected"
        return True, ""

    # ------------------------------------------------------------ extras
    def set_gripper(self, closed: float) -> None:
        """0 = open .. 1 = closed; sent with the next request (IsaacGripper calls this)."""
        self._grip = min(1.0, max(0.0, float(closed)))

    def command(self, d: dict) -> None:
        if d.get("reset"):
            self._pending["reset"] = 1
        if "cam" in d:
            self._pending.setdefault("cam", {}).update(d["cam"])
        if "view" in d:
            self._pending["view"] = d["view"]

    def extra(self) -> dict:
        return {k: self._last.get(k) for k in ("cams", "view", "tcp_offset", "resets") if k in self._last}

    def screenshot(self, path: str) -> None:
        """Ask the sim (GUI mode) to save a viewport screenshot."""
        self._rpc({"shot": path})
