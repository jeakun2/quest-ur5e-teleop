"""E2E: fake Quest client with a moving head -> hmd mode -> thumbstick click
opens the menu -> stick down + A toggles the Z lock -> B closes the menu."""
import asyncio
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bridge import teleop_bridge as tb  # noqa: E402
from test_webxr_e2e import free_port  # noqa: E402


def test_menu_over_websocket():
    port = free_port()
    cfg = {"source": {"type": "webxr", "port": port, "tls": False}, "robot": {"type": "dummy", "frequency": 125},
           "ui": {"head_still_s": 1.0}}
    args = tb.argparse.Namespace(record="", duration=8.0, quiet=True, config=None)
    th = threading.Thread(target=tb.run, args=(cfg, args), daemon=True)
    th.start()
    try:
        seen = asyncio.run(_client(port))
    finally:
        th.join(timeout=15)
    assert seen["mode"] == "hmd", seen
    assert seen["menu_open"] is True, seen
    assert seen["lock_z"] is True, seen
    assert seen["menu_closed"] is True, seen


async def _client(port):
    import websockets
    for _ in range(100):
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
    asyncio.ensure_future(reader())
    t0 = time.monotonic()

    def frame(stick=False, a=False, b=False, axes=(0, 0)):
        el = time.monotonic() - t0
        return {"t": el, "right": {"p": [0, 1, 0], "q": [0, 0, 0, 1], "grip": 0, "trigger": 0, "a": a, "b": b,
                                   "stick": stick, "axes": list(axes), "tracked": True},
                "head": {"p": [0.05 * math.sin(el * 3), 1.6, 0.05 * math.cos(el * 2)], "q": [0, 0, 0, 1]}}

    async def stream(sec, **kw):
        # deadline-based pacing: on Windows asyncio.sleep(1/72) returns almost immediately
        # (15.6 ms clock), which would flood the server with thousands of frames per second
        t = time.monotonic(); i = 0
        while time.monotonic() - t < sec:
            await ws.send(json.dumps(frame(**kw))); i += 1
            await asyncio.sleep(max(0.0, t + i / 72 - time.monotonic()))

    def ui():
        return statuses[-1]["ui"] if statuses else {}

    async def until(pred, timeout=3.0, **kw):
        t = time.monotonic()
        while time.monotonic() - t < timeout:
            await stream(0.05, **kw)
            if pred(ui()):
                return True
        return False
    seen = {}
    await until(lambda u: u.get("mode") == "hmd")
    seen["mode"] = ui().get("mode")
    await stream(0.15, stick=True)
    seen["menu_open"] = await until(lambda u: u.get("menu") is not None)
    if seen["menu_open"]:
        keys = [i["key"] for i in ui()["menu"]["items"]]
        for n in range(keys.index("lock_z")):
            await stream(0.1, axes=(0, 1))
            await until(lambda u: u.get("menu") and [i["key"] for i in u["menu"]["items"] if i["sel"]] == [keys[n + 1]])
        seen["cursor"] = [i["key"] for i in ui()["menu"]["items"] if i["sel"]]
        await stream(0.15, a=True)
        seen["lock_z"] = await until(lambda u: u.get("locks", {}).get("z") is True)
        await stream(0.15, b=True)
        seen["menu_closed"] = await until(lambda u: u.get("menu") is None)
    await ws.close()
    return seen
