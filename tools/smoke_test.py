"""Smoke test: drive a running bridge over its WebSocket like a Quest + monitor would.

    python bridge/teleop_bridge.py --source.port 8777 --source.tls false [--robot.type ur_rtde ...]
    python tools/smoke_test.py 8777                 # full sequence
    python tools/smoke_test.py 8777 runs/x.jsonl    # only replay that recording

Sequence: head moving -> hmd mode -> stick click opens the menu -> stick down to
"Z축 고정" -> A toggles it -> B closes -> cmd: record -> grip + 10 cm forward gesture
(the robot moves +X 10 cm) -> stop recording -> cmd: replay -> the robot returns to the
start pose and replays the motion.  Everything is printed; the final TCP should equal
the TCP after the gesture.
"""
import asyncio, json, sys, time
sys.stdout.reconfigure(encoding="utf-8")
import websockets

PORT = int(sys.argv[1]); REPLAY = sys.argv[2] if len(sys.argv) > 2 else None
statuses = []


async def main():
    try:
        ws = await websockets.connect(f"ws://127.0.0.1:{PORT}/ws", open_timeout=5)
    except Exception:
        import ssl                                   # bridge started with TLS (default) -> wss
        ctx = ssl.create_default_context(); ctx.check_hostname = False; ctx.verify_mode = ssl.CERT_NONE
        ws = await websockets.connect(f"wss://127.0.0.1:{PORT}/ws", open_timeout=5, ssl=ctx)

    async def reader():
        async for m in ws:
            d = json.loads(m)
            if d.get("type") == "status":
                statuses.append(d)
    asyncio.ensure_future(reader())

    def frame(el, p=(0, 1, 0), grip=0, stick=False, a=False, b=False, axes=(0, 0), head_moving=True):
        import math
        hp = [0.05 * math.sin(el * 2), 1.6, 0.05 * math.cos(el)] if head_moving else [0, 1.6, 0]
        return {"t": el, "right": {"p": list(p), "q": [0, 0, 0, 1], "grip": grip, "trigger": 0, "a": a, "b": b,
                                   "stick": stick, "axes": list(axes), "tracked": True},
                "head": {"p": hp, "q": [0, 0, 0, 1]}}

    sent = [0]

    async def stream(seconds, **kw):
        t0 = time.monotonic(); i = 0
        while time.monotonic() - t0 < seconds:
            await ws.send(json.dumps(frame(time.monotonic(), **kw))); sent[0] += 1; i += 1
            await asyncio.sleep(max(0.0, t0 + i / 72 - time.monotonic()))

    async def until(pred, timeout=2.0, **kw):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            await stream(0.05, **kw)
            if pred(last()):
                return True
        return False

    def last(): return statuses[-1] if statuses else {}

    if REPLAY is None:
        await stream(0.8)
        print("mode after head motion:", last()["ui"]["mode"])
        await stream(0.15, stick=True)
        ok = await until(lambda st: st.get("ui", {}).get("menu") is not None)
        print("menu open:", ok, "| state:", last()["state"], "| sent", sent[0])
        items = [i["key"] for i in last()["ui"]["menu"]["items"]]
        for _ in range(items.index("lock_z")):
            await stream(0.08, axes=(0, 1)); await stream(0.08)
        await stream(0.15, a=True)
        await until(lambda st: st.get("ui", {}).get("locks", {}).get("z"))
        print("cursor on:", [i["label"] for i in last()["ui"]["menu"]["items"] if i["sel"]], "locks:", last()["ui"]["locks"])
        await stream(0.15, b=True)
        print("menu closed:", await until(lambda st: st.get("ui", {}).get("menu") is None))
        await ws.send(json.dumps({"type": "cmd", "cmd": "lock", "axis": "z", "on": False}))
        await ws.send(json.dumps({"type": "cmd", "cmd": "record"}))
        tcp0 = last()["tcp"]
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:                       # grip + move forward 10 cm, release
            el = time.monotonic() - t0
            z = -min(el / 1.5, 1.0) * 0.10
            await ws.send(json.dumps(frame(time.monotonic(), p=(0, 1, z), grip=1 if el < 1.7 else 0)))
            await asyncio.sleep(1 / 72)
        await until(lambda st: st.get("ui", {}).get("recording"))
        print("TCP after gesture:", tcp0[:3], "->", last()["tcp"][:3], "rec:", last()["ui"]["recording"])
        await ws.send(json.dumps({"type": "cmd", "cmd": "record", "on": False}))
        await until(lambda st: st.get("ui", {}).get("recording") is None)
        print("msg:", last()["ui"]["msg"], "| sent", sent[0])
    await ws.send(json.dumps({"type": "cmd", "cmd": "replay", **({"path": REPLAY} if REPLAY else {})}))
    t0 = time.monotonic(); seen = set()
    while time.monotonic() - t0 < 40:
        await ws.send(json.dumps(frame(time.monotonic(), head_moving=False)))
        await asyncio.sleep(1 / 72)
        st = last().get("state", "")
        if st not in seen:
            seen.add(st); print(f"  {time.monotonic()-t0:5.1f}s {st}  tcp={last().get('tcp', [])[:3]}")
        if last().get("ui", {}).get("playing") is None and len(seen) > 1 and "PLAYING" in " ".join(seen):
            break
    print("final TCP:", last()["tcp"][:3], "| msg:", last()["ui"]["msg"])
    await ws.close()

asyncio.run(main())
