"""Input source backed by rail-berkeley / jborbik ``oculus_reader``.

The headset runs the ``com.rail.oculus.teleop`` APK (installed
automatically over adb on first use) which prints controller poses and
buttons to logcat; ``OculusReader`` tails logcat through adb.

Requirements: developer mode enabled on the Quest, adb on PATH or
``adb_path`` given, headset connected over USB (or ``ip_address`` for
Wi-Fi adb after ``adb tcpip 5555``).
"""
from __future__ import annotations

import os
import sys
import threading
import time

import numpy as np

from .base import Controller, Sample, Source

_HERE = os.path.dirname(os.path.abspath(__file__))
_THIRD_PARTY = os.path.normpath(os.path.join(_HERE, "..", "..", "third_party", "oculus_reader"))


def _import_reader():
    if _THIRD_PARTY not in sys.path:
        sys.path.insert(0, _THIRD_PARTY)
    from oculus_reader.reader import OculusReader  # type: ignore

    return OculusReader


def _stick(js) -> tuple:
    """Oculus SDK joystick: +y = forward.  Bridge convention (WebXR gamepad): -y = forward."""
    js = tuple(js) if js is not None else (0.0, 0.0)
    return (float(js[0]), -float(js[1])) if len(js) >= 2 else (0.0, 0.0)


class OculusReaderSource(Source):
    name = "oculus_reader"

    def __init__(self, ip_address: str | None = None, adb_path: str | None = None,
                 poll_hz: float = 200.0, keep_awake: bool = True) -> None:
        super().__init__()
        self.ip_address = ip_address or None
        self.adb_path = adb_path
        self.poll_hz = poll_hz
        self.keep_awake = keep_awake   # pretend the proximity sensor is covered (headset-off use)
        self._reader = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._last_raw_id = None

    def start(self) -> None:
        if self.adb_path:
            os.environ["PATH"] = os.path.dirname(self.adb_path) + os.pathsep + os.environ.get("PATH", "")
        OculusReader = _import_reader()
        # OculusReader talks to the adb server on 127.0.0.1:5037; make sure it runs.
        os.system((self.adb_path or "adb") + " start-server >nul 2>&1" if os.name == "nt"
                  else (self.adb_path or "adb") + " start-server >/dev/null 2>&1")
        if self.keep_awake:
            # same broadcast as tools/quest_keep_awake.py: tracking keeps running with the headset off
            os.system(f'"{self.adb_path or "adb"}" shell am broadcast -a com.oculus.vrpowermanager.prox_close')
        self._reader = OculusReader(ip_address=self.ip_address)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="oculus-poll")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._reader is not None:
            try:
                self._reader.stop()
            except Exception:
                pass
        if self.keep_awake:
            os.system(f'"{self.adb_path or "adb"}" shell am broadcast -a com.oculus.vrpowermanager.automation_disable')

    def _loop(self) -> None:
        period = 1.0 / self.poll_hz
        while self._running:
            transforms, buttons = self._reader.get_transformations_and_buttons()
            if transforms:
                raw_id = id(transforms)
                if raw_id != self._last_raw_id:  # new logcat line arrived
                    self._last_raw_id = raw_id
                    self._publish(self._convert(transforms, buttons))
            time.sleep(period)

    @staticmethod
    def _convert(transforms: dict, buttons: dict) -> Sample:
        s = Sample()
        if "r" in transforms:
            s.right = Controller(
                T=np.array(transforms["r"], dtype=float),
                grip=float(buttons.get("rightGrip", (0.0,))[0]),
                trigger=float(buttons.get("rightTrig", (0.0,))[0]),
                primary=bool(buttons.get("A", False)),
                secondary=bool(buttons.get("B", False)),
                thumbstick=_stick(buttons.get("rightJS", (0.0, 0.0))),
                thumbstick_click=bool(buttons.get("RJ", False)),
            )
        if "l" in transforms:
            s.left = Controller(
                T=np.array(transforms["l"], dtype=float),
                grip=float(buttons.get("leftGrip", (0.0,))[0]),
                trigger=float(buttons.get("leftTrig", (0.0,))[0]),
                primary=bool(buttons.get("X", False)),
                secondary=bool(buttons.get("Y", False)),
                thumbstick=_stick(buttons.get("leftJS", (0.0, 0.0))),
                thumbstick_click=bool(buttons.get("LJ", False)),
            )
        return s

    def status(self) -> str:
        return f"oculus_reader samples={self.count}"
