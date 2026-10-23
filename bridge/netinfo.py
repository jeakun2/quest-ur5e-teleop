"""LAN address discovery + QR code for the Quest browser URL."""
from __future__ import annotations

import os
import re
import socket
import subprocess
import sys


def lan_ips() -> list[str]:
    """IPv4 addresses a headset on the LAN could reach, best first (Wi-Fi/DHCP, then others).
    Link-local (169.254.*), loopback and virtual/WSL adapters are dropped."""
    found: list[tuple[int, str]] = []
    if sys.platform == "win32":
        try:
            out = subprocess.check_output(
                ["powershell", "-NoProfile", "-Command",
                 "Get-NetIPAddress -AddressFamily IPv4 | ForEach-Object { $_.InterfaceAlias + '|' + $_.IPAddress + '|' + $_.PrefixOrigin }"],
                text=True, errors="replace", timeout=15)
            for line in out.splitlines():
                parts = line.strip().split("|")
                if len(parts) != 3:
                    continue
                alias, ip, origin = parts
                if ip.startswith(("127.", "169.254.")):
                    continue
                if re.search(r"vEthernet|WSL|Hyper-V|VirtualBox|VMware|Loopback|OpenVPN|Bluetooth", alias, re.I):
                    continue
                score = 0
                if re.search(r"wi-?fi|wlan|무선", alias, re.I):
                    score += 10
                if origin.lower() == "dhcp":
                    score += 5
                if ip.startswith("192.168."):
                    score += 2
                found.append((score, ip))
        except Exception:
            pass
    if not found:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("10.255.255.255", 1))
            found.append((0, s.getsockname()[0]))
            s.close()
        except Exception:
            pass
    found.sort(key=lambda t: -t[0])
    seen, ips = set(), []
    for _, ip in found:
        if ip not in seen:
            seen.add(ip); ips.append(ip)
    return ips


def qr_ascii(url: str, invert: bool = True) -> str | None:
    """Return the QR as text (two rows per character line) or None if qrcode is not installed."""
    try:
        import qrcode
    except ImportError:
        return None
    q = qrcode.QRCode(border=2, error_correction=qrcode.constants.ERROR_CORRECT_M)
    q.add_data(url)
    q.make(fit=True)
    m = q.get_matrix()
    # dark modules on a light background: in a dark terminal, "dark" = block characters when invert=False.
    # Most terminals here are dark, so draw light background as full blocks (invert=True).
    rows = []
    for y in range(0, len(m), 2):
        top = m[y]
        bot = m[y + 1] if y + 1 < len(m) else [False] * len(top)
        line = []
        for t, b in zip(top, bot):
            dark_t, dark_b = (not t, not b) if invert else (t, b)
            line.append({(True, True): "█", (True, False): "▀", (False, True): "▄", (False, False): " "}[(dark_t, dark_b)])
        rows.append("".join(line))
    return "\n".join(rows)


def qr_png(url: str, path: str, caption: str | None = None) -> str | None:
    try:
        import qrcode
        from PIL import Image, ImageDraw
    except ImportError:
        return None
    img = qrcode.make(url, border=2, box_size=12).convert("RGB")
    w, h = img.size
    pad = 70
    canvas = Image.new("RGB", (w, h + pad), "white")
    canvas.paste(img, (0, 0))
    d = ImageDraw.Draw(canvas)
    text = caption or url
    try:
        from PIL import ImageFont
        font = ImageFont.truetype("arial.ttf", 28)
    except Exception:
        font = None
    tw = d.textlength(text, font=font) if font else len(text) * 7
    d.text(((w - tw) / 2, h + 15), text, fill="black", font=font)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    canvas.save(path)
    return path


def print_quest_qr(port: int, tls: bool, png_path: str | None = None) -> str | None:
    """Print the URL(s) + an ASCII QR for the best LAN address; optionally save a PNG. Returns the URL."""
    ips = lan_ips()
    scheme = "https" if tls else "http"
    if not ips:
        print(f"[qr] no LAN address found; open {scheme}://<PC-IP>:{port} on the headset")
        return None
    url = f"{scheme}://{ips[0]}:{port}"
    print(f"[qr] Quest browser URL: {url}" + (f"   (other adapters: {', '.join(ips[1:])})" if len(ips) > 1 else ""))
    art = qr_ascii(url)
    if art:
        print(art)
    else:
        print("[qr] pip install qrcode[pil]  to print a QR code here")
    if png_path:
        try:
            p = qr_png(url, png_path)
            if p:
                print(f"[qr] saved {p}")
        except Exception as e:                     # e.g. the PNG is open in a viewer (Windows file lock)
            print(f"[qr] png not saved ({e})")
    return url
