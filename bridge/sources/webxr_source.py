"""Input source for the WebXR client in ``web/`` (no developer mode needed).

One TLS server does two jobs on the same port:
  * HTTPS GET  -> serves the static files in ``web/`` (index.html, app.js)
  * WSS        -> receives JSON controller frames from the Quest browser
                  and sends back small status messages.

WebXR requires a secure context, so a self-signed certificate is
generated on first run (``certs/``).  The Quest browser shows a warning
once; choose "Advanced -> Proceed".

Wire format (one JSON object per frame, see docs/PROTOCOL.md)::

    {"t": 12.345, "right": {"p": [x,y,z], "q": [x,y,z,w], "grip": 0.0,
     "trigger": 0.0, "a": false, "b": false, "axes": [0,0], "tracked": true},
     "left": {...}, "head": {"p": [...], "q": [...]}}
"""
from __future__ import annotations

import asyncio
import json
import mimetypes
import os
import ssl
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from .base import Controller, Sample, Source

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
WEB_DIR = os.path.join(_ROOT, "web")
CERT_DIR = os.path.join(_ROOT, "certs")


def _pose_matrix(p, q) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R.from_quat(q).as_matrix()  # scipy: (x, y, z, w)
    T[:3, 3] = p
    return T


def _controller(d: dict) -> Controller:
    return Controller(
        T=_pose_matrix(d["p"], d["q"]),
        grip=float(d.get("grip", 0.0)),
        trigger=float(d.get("trigger", 0.0)),
        primary=bool(d.get("a", False)),
        secondary=bool(d.get("b", False)),
        thumbstick=tuple(d.get("axes", (0.0, 0.0))),
        thumbstick_click=bool(d.get("stick", False)),
        tracked=bool(d.get("tracked", True)),
    )


def ensure_self_signed_cert(cert_dir: str = CERT_DIR, hostnames: list[str] | None = None):
    """Create ``cert.pem``/``key.pem`` once (valid 10 years)."""
    os.makedirs(cert_dir, exist_ok=True)
    cert = os.path.join(cert_dir, "cert.pem")
    key = os.path.join(cert_dir, "key.pem")
    if os.path.exists(cert) and os.path.exists(key):
        return cert, key
    import datetime
    import ipaddress
    import socket

    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.x509.oid import NameOID

    k = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "quest-ur5e-teleop")])
    sans = [x509.DNSName("localhost")]
    for h in hostnames or []:
        try:
            sans.append(x509.IPAddress(ipaddress.ip_address(h)))
        except ValueError:
            sans.append(x509.DNSName(h))
    try:
        for ip in {i[4][0] for i in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)}:
            sans.append(x509.IPAddress(ipaddress.ip_address(ip)))
    except Exception:
        pass
    now = datetime.datetime.now(datetime.timezone.utc)
    crt = (x509.CertificateBuilder().subject_name(name).issuer_name(name)
           .public_key(k.public_key()).serial_number(x509.random_serial_number())
           .not_valid_before(now - datetime.timedelta(days=1))
           .not_valid_after(now + datetime.timedelta(days=3650))
           .add_extension(x509.SubjectAlternativeName(sans), critical=False)
           .sign(k, hashes.SHA256()))
    with open(key, "wb") as f:
        f.write(k.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL,
                                serialization.NoEncryption()))
    with open(cert, "wb") as f:
        f.write(crt.public_bytes(serialization.Encoding.PEM))
    return cert, key


class WebXRSource(Source):
    name = "webxr"

    def __init__(self, host: str = "0.0.0.0", port: int = 8765, tls: bool = True,
                 web_dir: str = WEB_DIR, max_rate: float = 250.0, cam_proxy: str = "http://127.0.0.1:5561") -> None:
        super().__init__()
        self.host, self.port, self.tls, self.web_dir = host, port, tls, web_dir
        self.cam_proxy = cam_proxy
        self._min_period = 1.0 / max_rate      # frames arriving faster than this are not published (buttons kept)
        self._held: dict = {}
        self._last_pub = 0.0
        self.dropped = 0
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._clients: set = set()
        self._status_msg: dict = {}
        self._stop = None

    # ---- lifecycle -------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True, name="webxr-server")
        self._thread.start()

    def stop(self) -> None:
        if self._loop and self._stop:
            self._loop.call_soon_threadsafe(self._stop.set)
        if self._thread:
            self._thread.join(timeout=2.0)

    def push_status(self, msg: dict) -> None:
        """Send a status dict to every connected headset (rate-limited by caller)."""
        self._status_msg = msg
        if self._loop:
            self._loop.call_soon_threadsafe(self._broadcast, json.dumps(msg))

    def _broadcast(self, text: str) -> None:
        async def _send(ws):
            try:
                await ws.send(text)
            except Exception:       # client went away between cycles
                self._clients.discard(ws)
        for ws in list(self._clients):
            asyncio.ensure_future(_send(ws))

    # ---- server ----------------------------------------------------------
    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    async def _serve(self) -> None:
        import websockets
        from websockets.asyncio.server import serve

        ssl_ctx = None
        if self.tls:
            cert, key = ensure_self_signed_cert()
            ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_ctx.load_cert_chain(cert, key)
        self._stop = asyncio.Event()
        async with serve(self._handler, self.host, self.port, ssl=ssl_ctx,
                         process_request=self._process_request, max_size=2 ** 16,
                         ping_interval=5, ping_timeout=5):
            await self._stop.wait()

    def _process_request(self, connection, request):
        """Serve static files for plain HTTP(S) GETs; let WebSocket upgrades through."""
        if "upgrade" in request.headers.get("Connection", "").lower():
            return None
        path = request.path.split("?", 1)[0]
        if path in ("", "/"):
            path = "/index.html"
        if path == "/favicon.ico":
            return connection.respond(204, "")
        if path.startswith("/cam/") and path.endswith(".jpg") and self.cam_proxy:
            # overlay camera frames come from the Isaac sim's little HTTP server (same PC)
            import urllib.request
            from websockets.datastructures import Headers
            from websockets.http11 import Response
            try:
                with urllib.request.urlopen(self.cam_proxy + path, timeout=0.5) as r:
                    body = r.read()
                return Response(200, "OK", Headers([("Content-Type", "image/jpeg"), ("Content-Length", str(len(body))),
                                                    ("Cache-Control", "no-store"), ("Connection", "close")]), body)
            except Exception:
                return connection.respond(404, "no frame\n")
        fs_path = os.path.normpath(os.path.join(self.web_dir, path.lstrip("/")))
        if not fs_path.startswith(os.path.normpath(self.web_dir)) or not os.path.isfile(fs_path):
            return connection.respond(404, "not found\n")
        ctype = mimetypes.guess_type(fs_path)[0] or "application/octet-stream"
        if ctype.startswith("text/") or ctype in ("application/javascript", "application/json"):
            ctype += "; charset=utf-8"
        with open(fs_path, "rb") as f:
            body = f.read()
        from websockets.datastructures import Headers
        from websockets.http11 import Response
        return Response(200, "OK", Headers([("Content-Type", ctype), ("Content-Length", str(len(body))),
                                            ("Cache-Control", "no-store"), ("Connection", "close")]), body)

    async def _handler(self, ws) -> None:
        self._clients.add(ws)
        try:
            if self._status_msg:
                await ws.send(json.dumps(self._status_msg))
            async for msg in ws:
                try:
                    d = json.loads(msg)
                except json.JSONDecodeError:
                    continue
                mtype = d.get("type")
                if mtype == "ping":
                    await ws.send(json.dumps({"type": "pong", "t": d.get("t")}))
                    continue
                if mtype == "cmd":
                    self.push_command(d)
                    continue
                if mtype == "log":
                    print(f"[headset] {d.get('msg')}  ({d.get('ua', '')})")
                    continue
                if mtype == "visibility":
                    print(f"[webxr] headset visibility: {d.get('state')}")
                    continue
                if mtype is not None:
                    continue
                now = time.monotonic()
                if now - self._last_pub < self._min_period:   # protect the control loop from a flooding client
                    # frames arrive in bursts (TCP coalescing); keep button presses of dropped frames so that
                    # a short click is never lost, only the pose is skipped
                    self.dropped += 1
                    for hand in ("right", "left"):
                        c = d.get(hand)
                        if c:
                            held = self._held.setdefault(hand, {})
                            for k in ("a", "b", "stick"):
                                held[k] = held.get(k, False) or bool(c.get(k))
                            if c.get("trigger", 0) > 0.8:
                                held["trigger"] = True
                    continue
                self._last_pub = now
                s = Sample(t=now)
                for hand in ("right", "left"):
                    c = d.get(hand)
                    if c:
                        held = self._held.pop(hand, None)
                        if held:
                            c = dict(c)
                            for k in ("a", "b", "stick"):
                                c[k] = bool(c.get(k)) or held.get(k, False)
                            if held.get("trigger"):
                                c["trigger"] = max(float(c.get("trigger", 0)), 1.0)
                        setattr(s, hand, _controller(c))
                if d.get("head"):
                    s.head = _pose_matrix(d["head"]["p"], d["head"]["q"])
                self._publish(s)
        finally:
            self._clients.discard(ws)

    def status(self) -> str:
        scheme = "https" if self.tls else "http"
        drop = f" dropped={self.dropped}" if self.dropped else ""
        return f"webxr {scheme}://<PC-IP>:{self.port} clients={len(self._clients)} samples={self.count}{drop}"
