"""Tiny VNC driver for URSim's PolyScope (port 5900): click / type / capture.

    python tools/ursim_vnc.py capture out.png
    python tools/ursim_vnc.py click out.png x y [x y ...]   # clicks in order, then captures
    python tools/ursim_vnc.py type out.png "text"
"""
import sys
import time

from vncdotool import api

HOST = "127.0.0.1::5900"


def main():
    cmd, out = sys.argv[1], sys.argv[2]
    c = api.connect(HOST, password=None, timeout=20)
    try:
        if cmd == "click":
            pts = [int(v) for v in sys.argv[3:]]
            for i in range(0, len(pts), 2):
                c.mouseMove(pts[i], pts[i + 1])
                time.sleep(0.2)
                c.mousePress(1)
                time.sleep(1.5)
        elif cmd == "type":
            c.type(sys.argv[3])
            time.sleep(0.5)
        c.refreshScreen(incremental=False)
        time.sleep(0.5)
        c.captureScreen(out)
    finally:
        api.shutdown()


if __name__ == "__main__":
    main()
