"""IsaacSimRobot client against a fake ur5e_sim server (same JSON-lines protocol)."""
import json
import os
import socket
import sys
import threading

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bridge.robots.isaac_robot import IsaacSimRobot  # noqa: E402


class FakeSim:
    def __init__(self):
        self.srv = socket.socket(); self.srv.bind(("127.0.0.1", 0)); self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.tcp = [0.1, 0.2, 0.3, 0.0, 3.14, 0.0]
        self.received = []
        self.hold = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        conn, _ = self.srv.accept()
        f = conn.makefile("rb")
        for line in f:
            d = json.loads(line)
            self.received.append(d)
            if "servo" in d:
                self.tcp = list(d["servo"]); self.hold = False
            if "hold" in d:
                self.hold = True
            conn.sendall((json.dumps({"tcp": self.tcp, "q": [0] * 6, "t": 1.0, "hz": 120.0,
                                      "tracking": not self.hold, "ok": True}) + "\n").encode())
        conn.close()


def test_isaac_client_roundtrip():
    sim = FakeSim()
    r = IsaacSimRobot("127.0.0.1", sim.port, connect_wait=5)
    r.connect()
    assert r.ok() == (True, "")
    assert np.allclose(r.tcp_pose(), [0.1, 0.2, 0.3, 0.0, 3.14, 0.0])
    r.servo(np.array([0.2, 0.2, 0.3, 0.0, 3.14, 0.0]), 0.008)
    assert np.allclose(r.tcp_pose()[:3], [0.2, 0.2, 0.3])
    r.hold()
    assert sim.received[-1] == {"hold": 1}
    r.disconnect()
    assert r.ok()[0] is False


def test_isaac_client_detects_dead_sim():
    sim = FakeSim()
    r = IsaacSimRobot("127.0.0.1", sim.port, connect_wait=5, timeout=0.5)
    r.connect()
    sim.srv.close()
    # kill the server side: the handler thread exits when the client closes, so close from the sim side instead
    r._sock.shutdown(socket.SHUT_RDWR)
    r.tcp_pose()
    ok, why = r.ok()
    assert not ok and "isaac sim link" in why


class RestartableSim(FakeSim):
    """FakeSim that keeps accepting connections (a restarted sim listens on the same port)."""
    @staticmethod
    def _lines(f):
        try:
            yield from f
        except OSError:
            return

    def _run(self):
        while True:
            try:
                conn, _ = self.srv.accept()
            except OSError:
                return
            self.conn = conn
            f = conn.makefile("rb")
            for line in self._lines(f):           # Windows raises WinError 10058 on a shut-down socket
                d = json.loads(line)
                self.received.append(d)
                if "servo" in d:
                    self.tcp = list(d["servo"]); self.hold = False
                if "hold" in d:
                    self.hold = True
                try:
                    conn.sendall((json.dumps({"tcp": self.tcp, "q": [0] * 6, "t": 1.0, "hz": 120.0,
                                              "tracking": not self.hold, "ok": True}) + "\n").encode())
                except OSError:
                    break
            conn.close()


def test_isaac_client_reconnects_after_sim_restart():
    sim = RestartableSim()
    r = IsaacSimRobot("127.0.0.1", sim.port, connect_wait=5, timeout=0.5, reconnect_every=0.0)
    r.connect()
    r.set_gripper(1.0)
    r.servo(np.array([0.2, 0.2, 0.3, 0.0, 3.14, 0.0]), 0.008)
    assert sim.received[-1].get("gripper") == 1.0
    # the sim dies mid-session: the link breaks, the bridge sees UNSAFE but the loop keeps running
    sim.conn.shutdown(socket.SHUT_RDWR)
    r.tcp_pose()
    ok, why = r.ok()
    assert not ok and "reconnecting" in why
    # ... and recovers on the next call once the (restarted) sim accepts again, re-sending the gripper state
    sim.tcp = [0.5, 0.0, 0.3, 0.0, 3.14, 0.0]
    n0 = len(sim.received)
    for _ in range(20):
        r.tcp_pose()
        if r.ok()[0]:
            break
    assert r.ok() == (True, "")
    assert r.reconnects == 1
    assert np.allclose(r.tcp_pose()[:3], [0.5, 0.0, 0.3])
    r.servo(np.array([0.6, 0.0, 0.3, 0.0, 3.14, 0.0]), 0.008)
    assert any(d.get("gripper") == 1.0 for d in sim.received[n0:])   # re-sent on the new link
    r.disconnect()
