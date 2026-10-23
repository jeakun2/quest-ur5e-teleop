"""UR5e back-end using ur_rtde (RTDEControlInterface.servoL).

Requirements on the robot / URSim:
  * PolyScope: Settings -> System -> Remote Control enabled, and the
    robot switched to *Remote* (top-right of PolyScope).
  * Robot powered on with brakes released (``--auto-power`` does this
    through the Dashboard server).
  * The PC and the robot on the same network; port 30004 reachable.

servoL runs at ``frequency`` Hz (UR e-series: up to 500 Hz).  ``lookahead``
(0.03..0.2 s) smooths, ``gain`` (100..2000) stiffens the tracking.
"""
from __future__ import annotations

import time

import numpy as np

from .base import Robot


class URRtdeRobot(Robot):
    name = "ur_rtde"

    def __init__(self, ip: str, frequency: float = 125.0, lookahead: float = 0.1,
                 gain: float = 300.0, auto_power: bool = False, speed_slider: float = 1.0) -> None:
        self.ip = ip
        self.frequency = frequency
        self.lookahead = lookahead
        self.gain = gain
        self.auto_power = auto_power
        self.speed_slider = speed_slider
        self._ctrl = None
        self._recv = None
        self._dash = None
        self._servoing = False

    def connect(self) -> None:
        from rtde_control import RTDEControlInterface
        from rtde_receive import RTDEReceiveInterface
        from dashboard_client import DashboardClient

        if self.auto_power:
            self._dash = DashboardClient(self.ip)
            self._dash.connect()
            mode = self._dash.robotmode()
            print(f"[ur] dashboard robotmode: {mode}")
            if "POWER_OFF" in mode or "IDLE" in mode or "BOOTING" in mode:
                print("[ur] powering on + releasing brakes ...")
                self._dash.powerOn()
                self._dash.brakeRelease()
                for _ in range(60):
                    time.sleep(0.5)
                    if "RUNNING" in self._dash.robotmode():
                        break
            print(f"[ur] dashboard robotmode: {self._dash.robotmode()}")

        self._recv = RTDEReceiveInterface(self.ip, self.frequency)
        self._ctrl = RTDEControlInterface(self.ip, self.frequency)
        if self.speed_slider != 1.0:
            self._ctrl.setSpeedSlider(self.speed_slider)
        print(f"[ur] connected to {self.ip} @ {self.frequency} Hz, TCP={np.round(self.tcp_pose(), 3)}")

    def disconnect(self) -> None:
        self.hold()
        try:
            if self._ctrl:
                self._ctrl.stopScript()
                self._ctrl.disconnect()
            if self._recv:
                self._recv.disconnect()
            if self._dash:
                self._dash.disconnect()
        except Exception:
            pass

    def tcp_pose(self) -> np.ndarray:
        return np.asarray(self._recv.getActualTCPPose(), dtype=float)

    def joints(self):
        try:
            return np.asarray(self._recv.getActualQ(), dtype=float)
        except Exception:
            return None

    def begin_cycle(self) -> float:
        return self._ctrl.initPeriod()

    def end_cycle(self, t0, dt: float) -> None:
        self._ctrl.waitPeriod(t0)

    def servo(self, pose: np.ndarray, dt: float) -> None:
        # velocity/acceleration args are ignored by servoL but must be given
        self._ctrl.servoL(list(map(float, pose)), 0.5, 0.5, dt, self.lookahead, self.gain)
        self._servoing = True

    def hold(self) -> None:
        if self._ctrl and self._servoing:
            try:
                self._ctrl.servoStop()
            except Exception:
                pass
            self._servoing = False

    def ok(self) -> tuple[bool, str]:
        if not self._ctrl or not self._ctrl.isConnected():
            return False, "rtde control disconnected"
        if self._recv.isProtectiveStopped():
            return False, "PROTECTIVE STOP"
        if self._recv.isEmergencyStopped():
            return False, "EMERGENCY STOP"
        st = self._recv.getRobotStatus()
        if not (st & 0b1):  # bit0 = power on
            return False, "robot powered off"
        return True, ""
