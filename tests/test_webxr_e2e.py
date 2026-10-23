"""End-to-end: fake Quest client -> WebXRSource (ws) -> bridge loop -> DummyRobot.

Runs the real ``run()`` loop for a few seconds in a thread while a websocket
client streams a scripted grip-and-move gesture, then checks the dummy TCP
moved +X by the expected amount and that status messages came back.
"""
import asyncio
import json
import os
import socket
import sys
import threading
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bridge import teleop_bridge as tb  # noqa: E402
from bridge.robots.dummy_robot import DummyRobot, HOME_TCP  # noqa: E402


def free_port():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close(); return p


def test_webxr_to_dummy_robot():
    port = free_port()
    cfg = {"source": {"type": "webxr", "port": port, "tls": False},
           "robot": {"type": "dummy", "frequency": 125},
           "mapping": {"pos_filter_alpha": 1.0, "max_lin_speed": 1.0}}
    created = {}
    orig = tb.make_robot

    def spy(c):
        created["robot"] = orig(c); return created["robot"]
    tb.make_robot = spy
    args = tb.argparse.Namespace(record="", duration=4.0, quiet=True)
    th = threading.Thread(target=tb.run, args=(cfg, args), daemon=True)
    th.start()
    try:
        statuses = asyncio.run(_client(port))
    finally:
        th.join(timeout=10)
        tb.make_robot = orig
    robot: DummyRobot = created["robot"]
    assert robot.servo_calls > 50
    moved = robot.tcp_pose()[:3] - HOME_TCP[:3]
    assert moved[0] == pytest.approx(0.10, abs=0.01), moved
    assert abs(moved[1]) < 0.01 and abs(moved[2]) < 0.01
    assert any(s.get("state", "").startswith("ENGAGED") for s in statuses), statuses[-3:]
    # the lock command sent over the same socket reached the session and shows up in the status
    assert statuses[-1]["ui"]["locks"]["z"] is True and statuses[-1]["ui"]["mode"] == "headless"


async def _client(port):
    import websockets
    for _ in range(50):  # wait for server
        try:
            ws = await websockets.connect(f"ws://127.0.0.1:{port}/ws", open_timeout=1)
            break
        except Exception:                 # refused, or accepted-then-closed while the server is starting
            await asyncio.sleep(0.1)
    else:
        raise RuntimeError("server did not start")
    statuses = []

    async def reader():
        try:
            async for m in ws:
                d = json.loads(m)
                if d.get("type") == "status":
                    statuses.append(d)
        except Exception:
            pass
    rt = asyncio.ensure_future(reader())
    await ws.send(json.dumps({"type": "cmd", "cmd": "lock", "axis": "z", "on": True}))
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        el = time.monotonic() - t0
        grip = 1.0 if 0.3 < el < 2.3 else 0.0
        z = -min(max((el - 0.5) / 1.0, 0.0), 1.0) * 0.10   # move 10 cm forward (-Z) within 1 s
        if el >= 2.3:
            z = 0.0                                            # after release: controller returns, robot must not
        frame = {"t": el, "right": {"p": [0.0, 1.0, z], "q": [0, 0, 0, 1], "grip": grip, "trigger": 0.0,
                                     "a": False, "b": False, "axes": [0, 0], "tracked": True}}
        await ws.send(json.dumps(frame))
        await asyncio.sleep(1 / 72)
    await ws.close()
    rt.cancel()
    return statuses


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q", "-s"]))
