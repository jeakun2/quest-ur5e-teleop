"""Show the QR code the Quest scans to open the teleop page.

    python tools/show_qr.py                 # prints URL + ASCII QR, saves qr/quest_url.png
    python tools/show_qr.py --open          # ... and opens the PNG in the default image viewer
    python tools/show_qr.py --port 8765 --tls true --ip 192.168.0.52

Point the Quest 3 at the QR (it recognises QR codes and offers "Open link"), or type the URL
in the Quest browser.  The bridge prints the same QR when it starts a WebXR source.
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from bridge.netinfo import lan_ips, print_quest_qr, qr_png  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--tls", default="true")
    ap.add_argument("--ip", default="", help="use this address instead of auto-detect")
    ap.add_argument("--out", default=os.path.join(ROOT, "qr", "quest_url.png"))
    ap.add_argument("--open", action="store_true")
    a = ap.parse_args()
    tls = str(a.tls).lower() not in ("0", "false", "no")
    if a.ip:
        url = f"{'https' if tls else 'http'}://{a.ip}:{a.port}"
        from bridge.netinfo import qr_ascii
        print(f"[qr] Quest browser URL: {url}")
        print(qr_ascii(url) or "")
        qr_png(url, a.out)
        print(f"[qr] saved {a.out}")
    else:
        url = print_quest_qr(a.port, tls, a.out)
    if a.open and os.path.exists(a.out):
        os.startfile(a.out)  # Windows default image viewer
    return 0 if url else 1


if __name__ == "__main__":
    sys.exit(main())
