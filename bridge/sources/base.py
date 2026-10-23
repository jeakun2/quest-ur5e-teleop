"""Common controller sample type shared by every input source.

Frame convention of every sample ("XR frame"): right-handed, metres,
+Y up, -Z forward (the direction the operator faces when the headset
was last recentred), +X right.  OpenXR / WebXR / oculus_reader all use
this frame natively; the Unity client converts before sending.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Controller:
    """State of one controller."""

    T: np.ndarray                 # 4x4 pose in XR frame (metres)
    grip: float = 0.0             # 0..1 analog grip (side) trigger
    trigger: float = 0.0          # 0..1 analog index trigger
    primary: bool = False         # A (right) / X (left)
    secondary: bool = False       # B (right) / Y (left)
    thumbstick: tuple = (0.0, 0.0)
    thumbstick_click: bool = False
    tracked: bool = True


@dataclass
class Sample:
    """One time-stamped reading of both controllers."""

    t: float = field(default_factory=time.monotonic)
    right: Controller | None = None
    left: Controller | None = None
    head: np.ndarray | None = None   # optional 4x4 head pose


class Source:
    """Base class. Sub-classes push samples with ``_publish`` from any thread."""

    name = "base"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: Sample | None = None
        self._count = 0
        self._cmds: list[dict] = []

    # UI commands (menu / locks / recording ...) sent by a client, see docs/PROTOCOL.md
    def push_command(self, cmd: dict) -> None:
        with self._lock:
            self._cmds.append(cmd)

    def pop_commands(self) -> list[dict]:
        with self._lock:
            out, self._cmds = self._cmds, []
        return out

    def start(self) -> None:  # pragma: no cover - overridden
        pass

    def stop(self) -> None:  # pragma: no cover - overridden
        pass

    def _publish(self, sample: Sample) -> None:
        with self._lock:
            self._latest = sample
            self._count += 1

    def latest(self) -> Sample | None:
        with self._lock:
            return self._latest

    @property
    def count(self) -> int:
        return self._count

    def status(self) -> str:
        return ""
