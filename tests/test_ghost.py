"""AR hologram placement (UISession.ghost_snap) and the Isaac gripper plumbing."""
import json
import os
import socket
import sys
import threading

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bridge.mapping import MapperConfig, TeleopMapper  # noqa: E402
from bridge.session import UISession  # noqa: E402
from bridge.gripper import make_gripper, IsaacGripper  # noqa: E402
from bridge.robots.isaac_robot import IsaacSimRobot  # noqa: E402


def make_session(tmp_path, yaw=0.0):
    m = TeleopMapper(MapperConfig(operator_yaw_deg=yaw))
    return UISession({"mode": "hmd"}, m, rec_dir=str(tmp_path), config_path=None, hand="right"), m


def test_ghost_snap_puts_tcp_at_hand(tmp_path):
    s, m = make_session(tmp_path, yaw=30.0)
    hand = np.eye(4); hand[:3, 3] = [0.2, 1.1, -0.5]
    tcp = np.array([-0.49, -0.13, 0.49, 2.2, 2.2, 0.0])
    assert s.ghost_snap_pending
    s.ghost_snap(hand, tcp)
    assert not s.ghost_snap_pending
    g = s.ui()["ghost"]
    assert g is not None and len(g["p"]) == 3 and len(g["q"]) == 4
    # base pose maps the robot-frame TCP back onto the hand position
    T = np.eye(4); T[:3, :3] = R.from_quat(g["q"]).as_matrix(); T[:3, 3] = g["p"]
    p = T @ np.array([*tcp[:3], 1.0])
    assert np.allclose(p[:3], hand[:3, 3], atol=1e-3)
    # robot +Z (up) must be XR +Y (up) regardless of yaw
    up = T[:3, :3] @ np.array([0, 0, 1.0])
    assert np.allclose(up, [0, 1, 0], atol=1e-6)


def test_ghost_menu_and_commands(tmp_path):
    s, m = make_session(tmp_path)
    keys = [k for k, _, _ in s._items()]
    assert "ghost" in keys and "path_clear" in keys
    e0 = s.ui()["path_epoch"]
    s.command({"cmd": "path_clear"})
    assert s.ui()["path_epoch"] == e0 + 1
    s.ghost_snap(np.eye(4), np.zeros(6))
    assert s.ui()["ghost"] is not None
    s.command({"cmd": "ghost", "hide": True})
    assert s.ui()["ghost"] is None
    s.command({"cmd": "ghost"})
    assert s.ghost_snap_pending


class FakeSim:
    def __init__(self):
        self.srv = socket.socket(); self.srv.bind(("127.0.0.1", 0)); self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.grip = 0.0
        self.received = []
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        conn, _ = self.srv.accept()
        for line in conn.makefile("rb"):
            d = json.loads(line); self.received.append(d)
            if "gripper" in d:
                self.grip = d["gripper"]
            conn.sendall((json.dumps({"tcp": [0] * 6, "q": [0.1] * 6, "grip": self.grip, "t": 0, "hz": 120,
                                      "tracking": False, "ok": True}) + "\n").encode())


def test_isaac_gripper_rides_on_robot_socket():
    sim = FakeSim()
    r = IsaacSimRobot("127.0.0.1", sim.port, connect_wait=5)
    r.connect()
    g = make_gripper({"type": "none"}, robot=r)          # auto-selects the Isaac gripper
    assert isinstance(g, IsaacGripper)
    g.command(0.7)
    r.tcp_pose()                                          # next request carries the gripper value
    assert any(abs(m.get("gripper", -1) - 0.7) < 1e-6 for m in sim.received)
    n = len(sim.received)
    r.tcp_pose()                                          # unchanged value is not re-sent
    assert "gripper" not in sim.received[n]
    assert r.joints() is not None and np.allclose(r.joints(), 0.1)
    r.disconnect()
