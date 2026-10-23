"""Put a freshly started URSim (docker, PolyScope 5.x, 1280x800 VNC on :5900) into
Remote Control mode by clicking through the GUI, then verify via the Dashboard.

    python tools/ursim_enable_remote.py

Sequence (coordinates for the stock 1280x800 PolyScope layout):
  Confirm Safety Configuration -> dismiss Getting Started -> hamburger -> Settings
  -> System -> Remote Control -> Enable -> Exit -> header Local switch -> "Remote Control".
Each click is best-effort; the script stops as soon as the Dashboard reports remote mode.
"""
import sys
import time

from dashboard_client import DashboardClient
from vncdotool import api

STEPS = [
    ("confirm safety config", (639, 706)),
    ("dismiss getting started", (640, 440)),
    ("open menu", (1247, 31)),
    ("settings", (1160, 178)),
    ("system", (122, 348)),
    ("remote control page", (150, 406)),
    ("enable", (617, 466)),
    ("exit settings", (145, 661)),
    ("header local switch", (1116, 31)),
    ("choose Remote Control", (1055, 89)),
]


def is_remote(ip="127.0.0.1") -> bool:
    d = DashboardClient(ip)
    try:
        d.connect()
        return bool(d.isInRemoteControl())
    finally:
        d.disconnect()


def main() -> int:
    if is_remote():
        print("already in remote control")
        return 0
    c = api.connect("127.0.0.1::5900", password=None, timeout=20)
    try:
        for name, (x, y) in STEPS:
            c.mouseMove(x, y)
            time.sleep(0.2)
            c.mousePress(1)
            print(f"  clicked {name}")
            time.sleep(1.5)
            if name in ("exit settings", "choose Remote Control") and is_remote():
                break
    finally:
        api.shutdown()
    ok = is_remote()
    print("remote control:", ok)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
