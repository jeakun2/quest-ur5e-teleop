"""Replay a JSONL recording (made with ``teleop_bridge.py --record``) as a source.

Useful for testing the mapping and robot layer without a headset.
"""
from __future__ import annotations

import json
import threading
import time

from .base import Sample, Source
from .webxr_source import _controller, _pose_matrix


class ReplaySource(Source):
    name = "replay"

    def __init__(self, path: str, loop: bool = True, speed: float = 1.0) -> None:
        super().__init__()
        self.path, self.loop_playback, self.speed = path, loop, speed
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True, name="replay")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        with open(self.path, encoding="utf-8") as f:
            frames = [json.loads(line) for line in f if line.strip()]
        if not frames:
            return
        while self._running:
            t0_rec, t0_wall = frames[0]["t"], time.monotonic()
            for d in frames:
                if not self._running:
                    return
                due = t0_wall + (d["t"] - t0_rec) / self.speed
                while time.monotonic() < due:
                    time.sleep(0.001)
                s = Sample(t=time.monotonic())
                if d.get("right"):
                    s.right = _controller(d["right"])
                if d.get("left"):
                    s.left = _controller(d["left"])
                if d.get("head"):
                    s.head = _pose_matrix(d["head"]["p"], d["head"]["q"])
                self._publish(s)
            if not self.loop_playback:
                break

    def status(self) -> str:
        return f"replay {self.path} samples={self.count}"
