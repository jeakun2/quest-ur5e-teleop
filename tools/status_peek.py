"""Print a few status messages from a running bridge (what the headset/monitor pages receive).

    python tools/status_peek.py [ws://127.0.0.1:8765/ws|wss://...] [n]
"""
import asyncio
import json
import ssl
import sys

import websockets

URL = sys.argv[1] if len(sys.argv) > 1 else "wss://127.0.0.1:8765/ws"
N = int(sys.argv[2]) if len(sys.argv) > 2 else 2


async def main():
    kw = {}
    if URL.startswith("wss"):
        ctx = ssl.create_default_context(); ctx.check_hostname = False; ctx.verify_mode = ssl.CERT_NONE
        kw["ssl"] = ctx
    async with websockets.connect(URL, open_timeout=5, **kw) as ws:
        got = 0
        while got < N:
            d = json.loads(await asyncio.wait_for(ws.recv(), 5))
            if d.get("type") != "status":
                continue
            got += 1
            ui = d.get("ui", {})
            print(json.dumps({"state": d.get("state"), "robot": d.get("robot"), "tcp": d.get("tcp"), "q": d.get("q"),
                              "gripper": d.get("gripper"), "engaged": d.get("engaged"),
                              "ghost": ui.get("ghost"), "path_epoch": ui.get("path_epoch"),
                              "menu_keys": [i["key"] for i in (ui.get("menu") or {}).get("items", [])]},
                             ensure_ascii=False))

asyncio.run(main())
