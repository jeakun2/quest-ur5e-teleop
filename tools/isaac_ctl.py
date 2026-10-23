"""Talk to a running isaac/ur5e_sim.py (any number of clients may be connected).

    python tools/isaac_ctl.py status                 # TCP, joints, loop Hz
    python tools/isaac_ctl.py shot <out.png>         # viewport screenshot (sim started without --headless)
    python tools/isaac_ctl.py quit                   # close the simulator
    python tools/isaac_ctl.py reset                  # re-initialise the scene (physics reset, home pose)
    python tools/isaac_ctl.py servo x y z rx ry rz [--gripper 0..1]   # one-off target (UR Base frame); holds again after --stale s
    python tools/isaac_ctl.py status --gripper 1     # just close/open the gripper
Options: --host 127.0.0.1 --port 5560
"""
import argparse
import json
import os
import socket
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["status", "shot", "quit", "servo", "reset"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5560)
    ap.add_argument("--gripper", type=float, default=None, help="0 open .. 1 closed (with servo/status)")
    a = ap.parse_args()
    if a.cmd == "status":
        msg = {"get": 1}
    elif a.cmd == "shot":
        msg = {"shot": os.path.abspath(a.args[0])}
    elif a.cmd == "quit":
        msg = {"quit": 1}
    elif a.cmd == "reset":
        msg = {"reset": 1}
    else:
        msg = {"servo": [float(v) for v in a.args[:6]]}
    if a.gripper is not None:
        msg["gripper"] = a.gripper
    s = socket.create_connection((a.host, a.port), timeout=3.0)
    s.sendall((json.dumps(msg) + "\n").encode())
    rep = s.makefile("rb").readline()
    print(json.loads(rep) if rep else "(no reply)")
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
