"""Controller pose -> robot TCP target mapping with clutch, scaling, filtering
and workspace limits.  Pure numpy; no robot or headset dependency, so it is
unit-tested in ``tests/test_mapping.py``.

Frames
------
XR frame   : right-handed, +Y up, -Z forward, metres (controller poses).
Robot frame: UR base frame, +Z up (TCP poses as ``[x, y, z, rx, ry, rz]``,
             rotation vector, as used by ur_rtde).

``R_map`` (3x3) rotates XR-frame vectors into robot-frame vectors.  The
default assumes the operator stands behind the robot base looking along
robot +X:  forward (-Z_xr) -> +X, left (-X_xr) -> +Y, up (+Y_xr) -> +Z.
``operator_yaw_deg`` rotates that about robot Z for other standing spots;
``calibrate_frame.py`` can derive R_map from three gestures.

Clutch
------
The robot only follows while the clutch (grip button by default) is held.
On engage the controller pose and the *current robot TCP* are latched, and
afterwards the target is::

    p_target = p_tcp0 + scale * R_map @ (p_ctrl - p_ctrl0)
    R_target = R_map @ (R_ctrl @ R_ctrl0^T) @ R_map^T @ R_tcp0

so releasing and re-gripping lets the operator "ratchet" through a large
workspace with small arm motions, exactly like a mouse lifted off the desk.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation as R

DEFAULT_R_MAP = np.array([[0.0, 0.0, -1.0],
                          [-1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0]])


def rotvec_to_matrix(rv) -> np.ndarray:
    return R.from_rotvec(np.asarray(rv, dtype=float)).as_matrix()


def matrix_to_rotvec(m) -> np.ndarray:
    return R.from_matrix(np.asarray(m, dtype=float)).as_rotvec()


def pose_to_T(pose6) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rotvec_to_matrix(pose6[3:6])
    T[:3, 3] = pose6[0:3]
    return T


def T_to_pose(T) -> np.ndarray:
    return np.concatenate([T[:3, 3], matrix_to_rotvec(T[:3, :3])])


@dataclass
class MapperConfig:
    r_map: np.ndarray = field(default_factory=lambda: DEFAULT_R_MAP.copy())
    operator_yaw_deg: float = 0.0
    pos_scale: float = 1.0            # 1.0 = 1:1 motion; <1 finer, >1 coarser
    rot_enabled: bool = True
    clutch: str = "grip"              # "grip" | "trigger" | "always"
    clutch_threshold: float = 0.5
    clutch_toggle: bool = False       # press-to-toggle instead of hold
    pos_filter_alpha: float = 0.35    # EMA on target position (1 = no filtering)
    rot_filter_alpha: float = 0.35    # slerp fraction toward target rotation
    max_lin_speed: float = 0.25       # m/s   per-cycle clamp
    max_ang_speed: float = 1.0        # rad/s per-cycle clamp
    workspace_min: np.ndarray = field(default_factory=lambda: np.array([-0.80, -0.80, 0.02]))
    workspace_max: np.ndarray = field(default_factory=lambda: np.array([0.80, 0.80, 0.80]))
    max_reach: float = 0.85           # sphere around base (UR5e reach 0.85 m)
    min_reach: float = 0.15           # keep clear of the base column
    stale_timeout: float = 0.3        # s without a fresh sample -> disengage
    # axis locks: a locked axis keeps the TCP value latched at clutch engage
    lock_x: bool = False
    lock_y: bool = False
    lock_z: bool = False
    lock_rot: bool = False
    # "relative" = ratchet (default).  "absolute" = controller pose relative to a
    # calibrated neutral pose (center_T) maps to the TCP relative to tcp_center,
    # so the same hand position always gives the same robot pose.
    mode: str = "relative"
    center_T: np.ndarray | None = None   # 4x4 XR pose of the operator's neutral hand position
    tcp_center: np.ndarray | None = None  # robot TCP paired with center_T

    def effective_r_map(self) -> np.ndarray:
        yaw = R.from_euler("z", self.operator_yaw_deg, degrees=True).as_matrix()
        return yaw @ self.r_map

    def locks(self) -> tuple[bool, bool, bool, bool]:
        return self.lock_x, self.lock_y, self.lock_z, self.lock_rot

    def absolute_ready(self) -> bool:
        return self.mode == "absolute" and self.center_T is not None and self.tcp_center is not None


@dataclass
class MapperOutput:
    engaged: bool
    target: np.ndarray | None        # [x,y,z,rx,ry,rz] or None when holding
    clamped: bool = False
    rate_limited: bool = False


class TeleopMapper:
    def __init__(self, cfg: MapperConfig | None = None) -> None:
        self.cfg = cfg or MapperConfig()
        self.engaged = False
        self._toggle_prev = False
        self._ctrl0: np.ndarray | None = None
        self._tcp0: np.ndarray | None = None
        self._filt_p: np.ndarray | None = None
        self._filt_R: np.ndarray | None = None
        self._last_cmd: np.ndarray | None = None

    # ------------------------------------------------------------------
    def _clutch_value(self, grip: float, trigger: float) -> bool:
        c = self.cfg.clutch
        if c == "always":
            return True
        v = grip if c == "grip" else trigger
        pressed = v >= self.cfg.clutch_threshold
        if not self.cfg.clutch_toggle:
            return pressed
        if pressed and not self._toggle_prev:
            self.engaged = not self.engaged
        self._toggle_prev = pressed
        return self.engaged

    def disengage(self) -> None:
        self.engaged = False
        self._ctrl0 = self._tcp0 = None
        self._filt_p = self._filt_R = None
        self._last_cmd = None

    # ------------------------------------------------------------------
    def update(self, ctrl_T: np.ndarray, grip: float, trigger: float,
               robot_tcp: np.ndarray, dt: float, tracked: bool = True) -> MapperOutput:
        """Advance one control cycle.

        ctrl_T    4x4 controller pose (XR frame)
        robot_tcp current robot TCP pose [x,y,z,rx,ry,rz] (robot frame)
        dt        cycle period in seconds
        """
        want = self._clutch_value(grip, trigger) and tracked
        if not want:
            if self.engaged:
                self.disengage()
            return MapperOutput(False, None)

        if not self.engaged or self._ctrl0 is None:
            # latch
            self.engaged = True
            self._ctrl0 = ctrl_T.copy()
            self._tcp0 = pose_to_T(robot_tcp)
            self._filt_p = self._tcp0[:3, 3].copy()
            self._filt_R = self._tcp0[:3, :3].copy()
            self._last_cmd = np.asarray(robot_tcp, dtype=float).copy()

        Rm = self.cfg.effective_r_map()
        if self.cfg.absolute_ready():
            ref_ctrl, ref_tcp = self.cfg.center_T, pose_to_T(self.cfg.tcp_center)
        else:
            ref_ctrl, ref_tcp = self._ctrl0, self._tcp0
        dp_xr = ctrl_T[:3, 3] - ref_ctrl[:3, 3]
        p_raw = ref_tcp[:3, 3] + self.cfg.pos_scale * (Rm @ dp_xr)

        if self.cfg.rot_enabled and not self.cfg.lock_rot:
            dR_xr = ctrl_T[:3, :3] @ ref_ctrl[:3, :3].T
            R_raw = Rm @ dR_xr @ Rm.T @ ref_tcp[:3, :3]
        else:
            R_raw = self._tcp0[:3, :3]

        # --- axis locks hold the value latched at engage
        for i, locked in enumerate((self.cfg.lock_x, self.cfg.lock_y, self.cfg.lock_z)):
            if locked:
                p_raw[i] = self._tcp0[i, 3]

        # --- workspace clamp (before filtering so filter never fights it)
        clamped = False
        p_cl = np.clip(p_raw, self.cfg.workspace_min, self.cfg.workspace_max)
        r = np.linalg.norm(p_cl)
        if r > self.cfg.max_reach:
            p_cl = p_cl * (self.cfg.max_reach / r)
        rxy = np.linalg.norm(p_cl[:2])
        if rxy < self.cfg.min_reach and rxy > 1e-9:
            p_cl[:2] = p_cl[:2] * (self.cfg.min_reach / rxy)
        clamped = not np.allclose(p_cl, p_raw, atol=1e-9)

        # --- low-pass
        a = self.cfg.pos_filter_alpha
        self._filt_p = self._filt_p + a * (p_cl - self._filt_p)
        if self.cfg.rot_enabled:
            key = R.from_matrix(np.stack([self._filt_R, R_raw]))
            # slerp fraction toward target
            rel = key[0].inv() * key[1]
            self._filt_R = (key[0] * R.from_rotvec(rel.as_rotvec() * self.cfg.rot_filter_alpha)).as_matrix()
        else:
            self._filt_R = R_raw

        # --- per-cycle rate limit against the last *commanded* pose
        rate_limited = False
        prev = self._last_cmd
        p_cmd = self._filt_p.copy()
        step = p_cmd - prev[:3]
        max_step = self.cfg.max_lin_speed * dt
        n = np.linalg.norm(step)
        if n > max_step:
            p_cmd = prev[:3] + step * (max_step / n)
            rate_limited = True
        R_prev = rotvec_to_matrix(prev[3:6])
        rel = R.from_matrix(R_prev.T @ self._filt_R)
        ang = np.linalg.norm(rel.as_rotvec())
        max_ang = self.cfg.max_ang_speed * dt
        if ang > max_ang:
            rel = R.from_rotvec(rel.as_rotvec() * (max_ang / ang))
            rate_limited = True
        R_cmd = R_prev @ rel.as_matrix()

        cmd = np.concatenate([p_cmd, matrix_to_rotvec(R_cmd)])
        self._last_cmd = cmd
        return MapperOutput(True, cmd, clamped, rate_limited)
