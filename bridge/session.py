"""Operator session logic on top of the mapper: headset-on/off mode detection,
the in-headset menu, calibration and tutorial flows, axis locks, recording
and trajectory playback.

Two operating modes
-------------------
headless  Headset lying on the desk (or worn with the display off).  Only the
          basic control is active: hold the GRIP (middle finger) and move ->
          the TCP follows in position and orientation; trigger = gripper;
          B = stop.  The menu button is ignored so a blind press cannot open
          the menu.  Everything else can still be driven from the PC monitor
          page (web/monitor.html) through ``cmd`` messages.
hmd       Headset worn: thumbstick click opens a menu (stick up/down, A select,
          B back) with tutorial, calibration, axis locks, recording, playback,
          scale and mapping mode.

With ``ui.mode: auto`` the mode is derived from head motion: a head pose that
has not moved for ``head_still_s`` seconds means the headset is not worn.
Sources without a head pose (oculus_reader) are always headless unless
``ui.mode: hmd`` is forced.

All of this is pure Python with no I/O besides the recorder, so it is unit
tested in ``tests/test_session.py``.
"""
from __future__ import annotations

import collections
import math
import os
import time
from dataclasses import dataclass

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from .mapping import TeleopMapper, pose_to_T
from .trajectory import Recorder, TrajectoryPlayer, list_recordings, load_tcp_frames


# ---------------------------------------------------------------- helpers
class Edges:
    """Rising-edge detector keyed by name."""

    def __init__(self) -> None:
        self._prev: dict[str, bool] = {}

    def rising(self, key: str, value: bool) -> bool:
        prev = self._prev.get(key, False)
        self._prev[key] = bool(value)
        return bool(value) and not prev


class HeadMotion:
    """Classifies the head pose as 'still' / 'moving' over a sliding window.
    Samples are decimated to ``min_dt`` and the angle test is vectorised, so a
    3 s window costs a few hundred numpy flops per update."""

    def __init__(self, window_s: float = 3.0, pos_m: float = 0.02, ang_deg: float = 3.0, min_dt: float = 0.05) -> None:
        self.window, self.pos_m, self.ang, self.min_dt = window_s, pos_m, math.radians(ang_deg), min_dt
        self._t: collections.deque = collections.deque()
        self._p: collections.deque = collections.deque()
        self._R: collections.deque = collections.deque()
        self._last: str | None = None

    def update(self, t: float, T: np.ndarray) -> str | None:
        if self._t and t - self._t[-1] < self.min_dt:
            return self._last
        self._t.append(t); self._p.append(T[:3, 3].copy()); self._R.append(T[:3, :3].copy())
        while self._t and t - self._t[0] > self.window:
            self._t.popleft(); self._p.popleft(); self._R.popleft()
        if len(self._t) < 2:
            self._last = None
            return None
        p0, R0 = self._p[0], self._R[0]
        dp = float(np.max(np.linalg.norm(np.asarray(self._p) - p0, axis=1)))
        rel = np.einsum("ji,njk->nik", R0, np.asarray(self._R))            # R0^T @ Ri for every i
        cos = np.clip((np.trace(rel, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0)
        da = float(np.max(np.arccos(cos)))
        if dp >= self.pos_m or da >= self.ang:
            self._last = "moving"                 # motion is recognised immediately
        elif t - self._t[0] < 0.9 * self.window:
            self._last = None                     # ... but "still" needs the whole window
        else:
            self._last = "still"
        return self._last


class GestureCapture:
    """Direction of a grip-held controller motion (release ends the gesture)."""

    def __init__(self, min_len: float = 0.08) -> None:
        self.min_len = min_len
        self._start: np.ndarray | None = None
        self._last: np.ndarray | None = None

    @property
    def recording(self) -> bool:
        return self._start is not None

    def feed(self, ctrl) -> tuple[str, np.ndarray | None, float]:
        """Returns (status, unit_vector, length).  status: idle|recording|done|short."""
        p = ctrl.T[:3, 3]
        if ctrl.grip >= 0.5:
            if self._start is None:
                self._start = p.copy()
            self._last = p.copy()
            return "recording", None, float(np.linalg.norm(self._last - self._start))
        if self._start is None:
            return "idle", None, 0.0
        v = self._last - self._start
        self._start = self._last = None
        n = float(np.linalg.norm(v))
        if n < self.min_len:
            return "short", None, n
        return "done", v / n, n


def yaw_from_forward_gesture(v_xr: np.ndarray, r_map: np.ndarray) -> float:
    """operator_yaw_deg such that the horizontal XR direction ``v_xr`` maps to robot +X."""
    v = np.array([v_xr[0], 0.0, v_xr[2]], dtype=float)       # drop the vertical part (XR +Y is up)
    n = np.linalg.norm(v)
    if n < 1e-6:
        raise ValueError("gesture was vertical")
    v_rob = r_map @ (v / n)
    return float(-math.degrees(math.atan2(v_rob[1], v_rob[0])))


def r_map_from_gestures(vs: list[np.ndarray]) -> np.ndarray:
    """R_map from the XR directions of robot +X, +Y, +Z (orthonormalised, right-handed)."""
    M = np.stack(vs, axis=1)
    U, _, Vt = np.linalg.svd(M)
    Rm = (U @ Vt).T
    if np.linalg.det(Rm) < 0:
        U[:, -1] *= -1
        Rm = (U @ Vt).T
    return Rm


@dataclass
class Action:
    """What the bridge loop should do this cycle."""

    state: str | None = None          # status text override (None = normal teleop state)
    allow_teleop: bool = True
    servo_pose: np.ndarray | None = None  # pose to servo instead of teleop (playback)
    gripper: float | None = None


@dataclass
class Ctx:
    ctrl: object | None
    tcp: np.ndarray
    now: float
    dt: float
    a: bool
    b: bool
    trig: bool
    mapper: TeleopMapper


# ---------------------------------------------------------------- flows
class Flow:
    name = "flow"
    title = ""

    def __init__(self, session: "UISession") -> None:
        self.s = session
        self.step = 0
        self.done = False
        self.allow_teleop = False
        self.result = ""

    def lines(self) -> list[str]:  # pragma: no cover - overridden
        return []

    def update(self, c: Ctx) -> None:  # pragma: no cover - overridden
        pass

    def after_cycle(self, tcp: np.ndarray, engaged: bool) -> None:
        pass

    def n_steps(self) -> int:
        return 1


class TutorialFlow(Flow):
    name = "tutorial"
    title = "튜토리얼"
    STEPS = [
        "① 오른손 그립(중지 버튼)을 쥔 채 손을 10 cm 이상 움직여 보세요. 로봇 TCP가 따라옵니다.",
        "② 그립을 놓으면 로봇이 그 자리에 멈춥니다. 손을 편한 위치로 옮긴 뒤 다시 쥐어 보세요(래칫).",
        "③ 트리거(검지)를 끝까지 당기면 그리퍼가 닫히고, 놓으면 열립니다.",
        "④ A 버튼: 자세(회전) 추종 끄기/켜기. 두 번 눌러 원래대로 돌려놓으세요.",
        "⑤ B 버튼: 누르는 동안 즉시 정지. 한 번 눌러 보세요.",
        "⑥ 썸스틱을 누르면 메뉴가 열립니다(헤드셋 착용 시). 축 고정·녹화·보정이 거기 있습니다. 스틱을 눌러 끝내세요.",
    ]

    def __init__(self, session) -> None:
        super().__init__(session)
        self.allow_teleop = True
        self._engage_tcp: np.ndarray | None = None
        self._was_engaged = False
        self._released_once = False
        self._a_count = 0

    def n_steps(self) -> int:
        return len(self.STEPS)

    def lines(self) -> list[str]:
        return [f"{self.step + 1}/{len(self.STEPS)}", self.STEPS[self.step], "스틱 클릭 = 종료" if self.step < 5 else ""]

    def update(self, c: Ctx) -> None:
        if self.s.stick_pressed_this_cycle:
            self.done = True
            self.result = "튜토리얼 완료" if self.step == 5 else f"튜토리얼 종료 ({self.step + 1}/{len(self.STEPS)})"
            return
        if c.ctrl is None:
            return
        if self.step == 2 and c.ctrl.trigger > 0.8:
            self.step = 3
        elif self.step == 3:
            if c.a:
                self._a_count += 1
            if self._a_count >= 2:
                self.step = 4
        elif self.step == 4 and c.b:
            self.step = 5

    def after_cycle(self, tcp: np.ndarray, engaged: bool) -> None:
        if self.step == 0:
            if engaged and not self._was_engaged:
                self._engage_tcp = tcp.copy()
            if engaged and self._engage_tcp is not None and np.linalg.norm(tcp[:3] - self._engage_tcp[:3]) > 0.05:
                self.step = 1
        elif self.step == 1:
            if self._was_engaged and not engaged:
                self._released_once = True
            if self._released_once and engaged and not self._was_engaged:
                self.step = 2
        self._was_engaged = engaged


class CalibFlow(Flow):
    """Robot direction (yaw) + operator neutral position + verification."""

    name = "calib"
    title = "보정: 로봇 방향·조작 위치"

    def __init__(self, session) -> None:
        super().__init__(session)
        self.gesture = GestureCapture()
        self.yaw: float | None = None
        self.center_T: np.ndarray | None = None
        self.tcp_center: np.ndarray | None = None
        self.verify_delta: np.ndarray | None = None
        self.verify_err_deg: float | None = None
        self._engage_tcp: np.ndarray | None = None
        self._was_engaged = False
        self._old_yaw = session.mapper.cfg.operator_yaw_deg

    def n_steps(self) -> int:
        return 4

    def lines(self) -> list[str]:
        if self.step == 0:
            st = "기록 중..." if self.gesture.recording else ""
            return ["1/4 로봇 방향", "로봇을 마주 보고 서서, 그립을 쥔 채 컨트롤러를 로봇 쪽(로봇 +X)으로 20 cm 이상 밀고 놓으세요.", st]
        if self.step == 1:
            return ["2/4 조작 위치", f"방향 설정됨: yaw {self.yaw:+.0f}°", "팔이 편한 조작 중심에 컨트롤러를 두고 트리거를 당기세요."]
        if self.step == 2:
            return ["3/4 확인", "그립을 쥐고 로봇 쪽으로 10 cm 움직인 뒤 놓으세요. 로봇이 +X로 가야 합니다.",
                    "" if self.verify_delta is None else self._verify_text()]
        return ["4/4 저장", self._verify_text(), "A = config.yaml에 저장 · B = 저장 않고 종료"]

    def _verify_text(self) -> str:
        if self.verify_delta is None:
            return ""
        d = self.verify_delta
        ok = self.verify_err_deg is not None and self.verify_err_deg < 20
        return f"로봇 이동 X{d[0]:+.2f} Y{d[1]:+.2f} Z{d[2]:+.2f} m, 오차 {self.verify_err_deg:.0f}° {'✓' if ok else '✗ (1단계부터 다시)'}"

    def update(self, c: Ctx) -> None:
        if c.b and self.step != 3:
            self.s.mapper.cfg.operator_yaw_deg = self._old_yaw
            self.done, self.result = True, "보정 취소"
            return
        if self.step == 0:
            self.allow_teleop = False
            if c.ctrl is None:
                return
            st, v, n = self.gesture.feed(c.ctrl)
            if st == "short":
                self.s.flash(f"너무 짧음 ({n*100:.0f} cm), 다시")
            elif st == "done":
                try:
                    self.yaw = yaw_from_forward_gesture(v, self.s.mapper.cfg.r_map)
                except ValueError:
                    self.s.flash("수평으로 움직이세요")
                    return
                self.s.mapper.cfg.operator_yaw_deg = self.yaw
                self.step = 1
        elif self.step == 1:
            if c.ctrl is not None and c.trig:
                self.center_T = c.ctrl.T.copy()
                self.tcp_center = c.tcp.copy()
                self.step = 2
                self.allow_teleop = True
                self.s.mapper.disengage()
        elif self.step == 2:
            self.allow_teleop = True
        elif self.step == 3:
            self.allow_teleop = False
            if c.a:
                self._apply(save=True)
            elif c.b:
                self._apply(save=False)

    def after_cycle(self, tcp: np.ndarray, engaged: bool) -> None:
        if self.step == 2:
            if engaged and not self._was_engaged:
                self._engage_tcp = tcp.copy()
            if self._was_engaged and not engaged and self._engage_tcp is not None:
                d = tcp[:3] - self._engage_tcp[:3]
                if np.linalg.norm(d) > 0.03:
                    self.verify_delta = d
                    self.verify_err_deg = math.degrees(math.atan2(math.hypot(d[1], d[2]), d[0]))
                    if self.verify_err_deg < 20:
                        self.step = 3
                    else:
                        self.step = 0
                        self.s.flash("방향이 맞지 않습니다. 1단계부터 다시.")
                        self.s.mapper.cfg.operator_yaw_deg = self._old_yaw
        self._was_engaged = engaged

    def _apply(self, save: bool) -> None:
        cfg = self.s.mapper.cfg
        cfg.center_T, cfg.tcp_center = self.center_T, self.tcp_center
        self.done = True
        if save:
            q = R.from_matrix(self.center_T[:3, :3]).as_quat().round(6).tolist()
            self.s.save_mapping({"operator_yaw_deg": round(self.yaw, 2),
                                 "center_xr": {"p": self.center_T[:3, 3].round(4).tolist(), "q": q},
                                 "tcp_center": [round(float(v), 4) for v in self.tcp_center]})
            self.result = f"보정 저장됨 (yaw {self.yaw:+.0f}°)"
        else:
            self.result = f"보정 적용(이번 세션만), yaw {self.yaw:+.0f}°"


class Calib3Flow(Flow):
    """Full R_map from three gestures (same maths as calibrate_frame.py)."""

    name = "calib3"
    title = "정밀 보정: 3축 제스처"
    AXES = ["+X (로봇 앞쪽)", "+Y (로봇 왼쪽)", "+Z (위)"]

    def __init__(self, session) -> None:
        super().__init__(session)
        self.gesture = GestureCapture()
        self.vs: list[np.ndarray] = []
        self.r_map: np.ndarray | None = None

    def n_steps(self) -> int:
        return 4

    def lines(self) -> list[str]:
        if self.step < 3:
            return [f"{self.step + 1}/4 로봇 {self.AXES[self.step]}",
                    "그립을 쥔 채 그 방향으로 30 cm 움직이고 놓으세요.", "기록 중..." if self.gesture.recording else "B = 취소"]
        return ["4/4 저장", "R_map 계산됨 (det=1)", "A = config.yaml에 저장 · B = 취소"]

    def update(self, c: Ctx) -> None:
        if c.b:
            self.done, self.result = True, "정밀 보정 취소"
            return
        if self.step < 3:
            if c.ctrl is None:
                return
            st, v, n = self.gesture.feed(c.ctrl)
            if st == "short":
                self.s.flash(f"너무 짧음 ({n*100:.0f} cm), 다시")
            elif st == "done":
                self.vs.append(v)
                self.step += 1
                if self.step == 3:
                    self.r_map = r_map_from_gestures(self.vs)
        elif c.a:
            cfg = self.s.mapper.cfg
            cfg.r_map, cfg.operator_yaw_deg = self.r_map, 0.0
            self.s.save_mapping({"r_map": np.round(self.r_map, 6).tolist(), "operator_yaw_deg": 0})
            self.done, self.result = True, "R_map 저장됨"


# ---------------------------------------------------------------- session
class UISession:
    SCALES = [0.25, 0.5, 1.0, 1.5]

    def __init__(self, ui_cfg: dict, mapper: TeleopMapper, rec_dir: str = "runs",
                 config_path: str | None = None, hand: str = "right") -> None:
        self.mapper = mapper
        self.rec_dir = rec_dir
        self.config_path = config_path
        self.hand = hand
        self.mode_cfg = str(ui_cfg.get("mode", "auto"))
        self.detected = "headless"
        self.head = HeadMotion(float(ui_cfg.get("head_still_s", 3.0)), float(ui_cfg.get("head_still_m", 0.02)),
                               float(ui_cfg.get("head_still_deg", 3.0)))
        self.menu_open = False
        self.cursor = 0
        self.flow: Flow | None = None
        self.recorder: Recorder | None = None
        self.player: TrajectoryPlayer | None = None
        self.edges = Edges()
        self._nav_armed = True
        self._virtual: set[str] = set()
        self.ghost_T: np.ndarray | None = None      # XR <- robot-base transform for the AR hologram
        self.ghost_snap_pending = True             # snap on the first grip (and on request)
        self.path_epoch = 0                        # bump -> clients clear their drawn paths
        self.home_tcp: np.ndarray | None = None       # TCP at bridge start (menu '초기 자세로')
        self.robot_info: dict = {}                   # back-end extras (Isaac cams/view/tcp_offset)
        self.robot_cmds: list[dict] = []             # requests for the robot back-end (bridge pops them)
        self.a_opens_menu = True                     # A = options menu (headset); stick click also works
        self.ghost_move: dict | None = None          # '홀로그램 옮기기' mode state
        self._msg, self._msg_until = "", 0.0
        self.stick_pressed_this_cycle = False
        self.last_result = ""
        self.meta: dict = {}

    # ---- mode ----------------------------------------------------------
    @property
    def mode(self) -> str:
        return self.detected if self.mode_cfg == "auto" else self.mode_cfg

    def flash(self, text: str, seconds: float = 3.0) -> None:
        self._msg, self._msg_until = text, time.monotonic() + seconds
        print(f"[ui] {text}")

    # ---- menu ----------------------------------------------------------
    def _items(self) -> list[tuple[str, str, str]]:
        cfg = self.mapper.cfg
        onoff = lambda b: "켜짐" if b else "꺼짐"  # noqa: E731
        rec = self.recorder
        items = [
            ("home", "초기 자세로", "" if self.home_tcp is not None else "(시작 TCP 없음)"),
            ("rot", "회전 추종", onoff(cfg.rot_enabled)),
            ("scene_reset", "Isaac 씬 초기화", f"{self.robot_info.get('resets', 0)}회" if "resets" in self.robot_info else "(Isaac 전용)"),
            ("tutorial", "튜토리얼", ""),
            ("calib", "보정 (로봇 방향·조작 위치)", f"yaw {cfg.operator_yaw_deg:+.0f}°"),
            ("calib3", "정밀 보정 (3축 제스처)", ""),
            ("lock_x", "X축 고정", onoff(cfg.lock_x)),
            ("lock_y", "Y축 고정", onoff(cfg.lock_y)),
            ("lock_z", "Z축 고정", onoff(cfg.lock_z)),
            ("lock_rot", "회전 고정", onoff(cfg.lock_rot)),
            ("record", "녹화 정지" if rec else "녹화 시작", f"{rec.seconds:.0f}s {rec.frames}f" if rec else ""),
            ("replay", "마지막 녹화 재생", os.path.basename(self._last_recording() or "없음")),
            ("scale", "이동 배율", f"x{cfg.pos_scale:g}"),
            ("map_mode", "매핑 방식", "절대" if cfg.mode == "absolute" else "상대(래칫)"),
            ("cam_ceiling", "천장 카메라 오버레이", onoff(self.robot_info.get("cams", {}).get("ceiling"))),
            ("cam_side", "사이드 카메라 오버레이", onoff(self.robot_info.get("cams", {}).get("side"))),
            ("cam_ee", "EE 카메라 오버레이", onoff(self.robot_info.get("cams", {}).get("ee"))),
            ("view", "Isaac 시점 전환", str(self.robot_info.get("view", "-"))),
            ("ghost", "홀로그램 손에 맞추기", "표시 중" if self.ghost_T is not None else "숨김"),
            ("ghost_move", "홀로그램 옮기기/돌리기", "진행 중" if self.ghost_move is not None else ""),
            ("path_clear", "경로 지우기", ""),
            ("close", "메뉴 닫기", ""),
        ]
        return items

    def _last_recording(self) -> str | None:
        files = list_recordings(self.rec_dir, 1)
        if self.recorder and files and os.path.abspath(files[0]) == os.path.abspath(self.recorder.path):
            files = list_recordings(self.rec_dir, 2)[1:]
        return files[0] if files else None

    def _select(self, key: str) -> None:
        cfg = self.mapper.cfg
        if key == "close":
            self.menu_open = False
        elif key in ("tutorial", "calib", "calib3"):
            self.menu_open = False
            self.start_flow(key)
        elif key.startswith("lock_"):
            self.set_lock(key[5:], None)
        elif key == "record":
            self.set_recording(self.recorder is None)
        elif key == "replay":
            self.menu_open = False
            self.start_replay(None)
        elif key == "scale":
            i = (self.SCALES.index(cfg.pos_scale) + 1) % len(self.SCALES) if cfg.pos_scale in self.SCALES else 0
            cfg.pos_scale = self.SCALES[i]
            self.flash(f"이동 배율 x{cfg.pos_scale:g}")
        elif key == "map_mode":
            self.set_map_mode("relative" if cfg.mode == "absolute" else "absolute")
        elif key == "home":
            self.menu_open = False
            self.start_home()
        elif key == "rot":
            cfg.rot_enabled = not cfg.rot_enabled
            self.flash(f"자세 추종 {'켜짐' if cfg.rot_enabled else '꺼짐'}")
        elif key == "scene_reset":
            self.menu_open = False
            self.scene_reset()
        elif key.startswith("cam_"):
            self.set_cam(key[4:], None)
        elif key == "view":
            self.cycle_view()
        elif key == "ghost_move":
            self.menu_open = False
            self.start_ghost_move()
        elif key == "ghost":
            self.menu_open = False
            self.ghost_snap_pending = True
            self.flash("그립을 쥐면 홀로그램 툴이 손 위치에 맞춰집니다")
        elif key == "path_clear":
            self.clear_path()

    # ---- feature setters (menu + commands share them) ---------------------
    def set_lock(self, axis: str, on: bool | None) -> None:
        cfg = self.mapper.cfg
        attr = f"lock_{axis}"
        if not hasattr(cfg, attr):
            self.flash(f"알 수 없는 축 {axis}")
            return
        setattr(cfg, attr, (not getattr(cfg, attr)) if on is None else bool(on))
        self.mapper.disengage()  # re-latch so the lock holds the current pose
        self.flash(f"{axis.upper()} 고정 {'켜짐' if getattr(cfg, attr) else '꺼짐'}")

    def set_recording(self, on: bool) -> None:
        if on and self.recorder is None:
            path = os.path.join(self.rec_dir, time.strftime("rec_%Y%m%d_%H%M%S.jsonl"))
            self.recorder = Recorder(path, meta=dict(self.meta, started=time.strftime("%Y-%m-%d %H:%M:%S")))
            self.flash(f"녹화 시작 {os.path.basename(path)}")
        elif not on and self.recorder is not None:
            self.flash(f"녹화 저장 {os.path.basename(self.recorder.path)} ({self.recorder.frames}f)")
            self.recorder.close()
            self.recorder = None

    def set_map_mode(self, mode: str) -> None:
        cfg = self.mapper.cfg
        if mode == "absolute" and (cfg.center_T is None or cfg.tcp_center is None):
            self.flash("절대 모드는 먼저 보정(조작 위치)이 필요합니다")
            return
        cfg.mode = mode
        self.mapper.disengage()
        self.flash("매핑: " + ("절대" if mode == "absolute" else "상대(래칫)"))

    def start_flow(self, name: str) -> None:
        cls = {"tutorial": TutorialFlow, "calib": CalibFlow, "calib3": Calib3Flow}.get(name)
        if cls is None:
            self.flash(f"알 수 없는 흐름 {name}")
            return
        self.stop_replay()
        self.mapper.disengage()
        self.flow = cls(self)
        self.flash(f"{self.flow.title} 시작")

    def start_replay(self, path: str | None, speed: float = 1.0) -> None:
        path = path or self._last_recording()
        if not path or not os.path.exists(path):
            self.flash("재생할 녹화가 없습니다")
            return
        frames = load_tcp_frames(path)
        if not frames:
            self.flash("녹화에 TCP 데이터가 없습니다")
            return
        cfg = self.mapper.cfg
        self.player = TrajectoryPlayer(frames, cfg.max_lin_speed, cfg.max_ang_speed, speed)
        self.player.path = path  # type: ignore[attr-defined]
        self.mapper.disengage()
        self.flash(f"재생 {os.path.basename(path)} ({len(frames)}f) — B로 중단")

    def stop_replay(self) -> None:
        if self.player is not None:
            self.player = None
            self.flash("재생 중단")

    def save_mapping(self, updates: dict) -> None:
        if not self.config_path:
            self.flash("config 경로 없음: 이번 세션에만 적용")
            return
        with open(self.config_path, encoding="utf-8") as f:
            full = yaml.safe_load(f) or {}
        full.setdefault("mapping", {}).update(updates)
        with open(self.config_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(full, f, sort_keys=False, allow_unicode=True)
        print(f"[ui] saved {list(updates)} to {self.config_path}")

    # ---- commands from the monitor / client ------------------------------
    def command(self, d: dict) -> None:
        c = d.get("cmd")
        if c == "menu":
            self.menu_open = not self.menu_open
        elif c in ("up", "down") and self.menu_open:
            self.cursor = (self.cursor + (-1 if c == "up" else 1)) % len(self._items())
        elif c == "select" and self.menu_open:
            self._select(self._items()[self.cursor][0])
        elif c == "back":
            self.menu_open = False
        elif c == "lock":
            self.set_lock(str(d.get("axis", "")), d.get("on"))
        elif c == "record":
            self.set_recording(bool(d.get("on", self.recorder is None)))
        elif c == "replay":
            self.start_replay(d.get("path"), float(d.get("speed", 1.0)))
        elif c == "stop":
            self.stop_replay()
            self.ghost_move = None
            if self.flow:
                self.flow, self.last_result = None, "취소됨"
            self.menu_open = False
            self.mapper.disengage()
        elif c == "scale":
            self.mapper.cfg.pos_scale = float(d.get("value", 1.0))
            self.flash(f"이동 배율 x{self.mapper.cfg.pos_scale:g}")
        elif c == "map_mode":
            self.set_map_mode(str(d.get("value", "relative")))
        elif c == "ui_mode":
            v = str(d.get("value", "auto"))
            if v in ("auto", "headless", "hmd"):
                self.mode_cfg = v
                self.flash(f"UI 모드 {v}")
        elif c == "rot":
            self.mapper.cfg.rot_enabled = bool(d.get("on", not self.mapper.cfg.rot_enabled))
            self.flash(f"자세 추종 {'켜짐' if self.mapper.cfg.rot_enabled else '꺼짐'}")
        elif c == "yaw":
            self.mapper.cfg.operator_yaw_deg = float(d.get("value", 0.0))
            self.flash(f"yaw {self.mapper.cfg.operator_yaw_deg:+.0f}°")
        elif c in ("tutorial", "calib", "calib3"):
            self.start_flow(c)
        elif c in ("confirm", "cancel", "trigger"):
            self._virtual.add({"confirm": "a", "cancel": "b", "trigger": "trig"}[c])
        elif c == "home":
            self.start_home()
        elif c == "scene_reset":
            self.scene_reset()
        elif c == "cam":
            self.set_cam(str(d.get("name", "")), d.get("on"))
        elif c == "view":
            v = d.get("value")
            if v:
                self.robot_cmds.append({"view": str(v)})
                self.flash(f"Isaac 시점 {v}")
            else:
                self.cycle_view()
        elif c == "ghost":
            if d.get("hide"):
                self.ghost_T = None
                self.flash("홀로그램 숨김")
            else:
                self.ghost_snap_pending = True
                self.flash("그립을 쥐면 홀로그램 툴이 손 위치에 맞춰집니다")
        elif c == "path_clear":
            self.clear_path()
        elif c == "ghost_move":
            if d.get("stop"):
                self.finish_ghost_move()
            else:
                self.start_ghost_move()
        else:
            self.flash(f"알 수 없는 명령 {c}")

    # ---- home pose / Isaac cameras / viewport ------------------------------
    VIEWS = ["default", "top", "side", "ee"]

    def start_home(self) -> None:
        """Drive the TCP back to the pose recorded at start (rate-limited approach, gripper open)."""
        if self.home_tcp is None:
            self.flash("시작 자세가 기록되지 않았습니다")
            return
        cfg = self.mapper.cfg
        frames = [{"t": 0.0, "tcp": [float(v) for v in self.home_tcp], "gripper": 0.0},
                  {"t": 0.2, "tcp": [float(v) for v in self.home_tcp], "gripper": 0.0}]
        self.player = TrajectoryPlayer(frames, cfg.max_lin_speed, cfg.max_ang_speed, 1.0)
        self.player.path = "초기 자세"  # type: ignore[attr-defined]
        self.mapper.disengage()
        self.flash("초기 자세로 이동 — B로 중단")

    def scene_reset(self) -> None:
        """Isaac only: re-initialise the whole scene (physics, home joints, gripper, cube)."""
        if "resets" not in self.robot_info:
            self.flash("씬 초기화는 Isaac 시뮬에서만 됩니다")
            return
        self.stop_replay()
        self.mapper.disengage()
        self.ghost_T = None
        self.ghost_snap_pending = True
        self.path_epoch += 1
        self.robot_cmds.append({"reset": 1})
        self.flash("Isaac 씬 초기화 중 — 다시 그립을 쥐면 홀로그램이 손에 맞춰집니다")

    def set_cam(self, name: str, on: bool | None) -> None:
        cams = self.robot_info.get("cams")
        if not cams or name not in cams:
            self.flash("이 로봇에는 오버레이 카메라가 없습니다 (Isaac 창 모드만)")
            return
        if on is None:
            on = not cams.get(name, False)
        self.robot_cmds.append({"cam": {name: bool(on)}})
        self.flash(f"{name} 카메라 {'켜짐' if on else '꺼짐'}")

    def cycle_view(self) -> None:
        if "view" not in self.robot_info:
            self.flash("시점 전환은 Isaac 창 모드에서만")
            return
        cur = str(self.robot_info.get("view", "default"))
        nxt = self.VIEWS[(self.VIEWS.index(cur) + 1) % len(self.VIEWS)] if cur in self.VIEWS else "default"
        self.robot_cmds.append({"view": nxt})
        self.flash(f"Isaac 시점 {nxt}")

    # ---- AR ghost robot: move / rotate the robot's virtual base with the hand ---------
    def start_ghost_move(self) -> None:
        self.stop_replay()
        self.mapper.disengage()
        self.ghost_move = {"grabbed": False}
        self.flash("홀로그램 옮기기: 그립을 쥐고 끌거나 손목을 돌리세요. A/B로 완료")

    def finish_ghost_move(self) -> None:
        if self.ghost_move is None:
            return
        self.ghost_move = None
        if self.ghost_T is not None:
            # keep the hand->robot mapping consistent with where the hologram now stands:
            # effective_r_map = Rz(yaw) @ r_map must equal ghost_R^T  ->  Rz(yaw) = ghost_R^T @ r_map^T
            cfg = self.mapper.cfg
            yaw_m = self.ghost_T[:3, :3].T @ cfg.r_map.T
            cfg.operator_yaw_deg = float(np.degrees(np.arctan2(yaw_m[1, 0], yaw_m[0, 0])))
            self.flash(f"홀로그램 배치 완료 (yaw {cfg.operator_yaw_deg:+.0f}°)")
        self.ghost_snap_pending = False

    def _ghost_move_step(self, ctrl, tcp, a: bool, b: bool) -> "Action":
        st = self.ghost_move
        if a or b:
            self.finish_ghost_move()
            return Action(state="홀로그램 배치 완료", allow_teleop=False)
        if ctrl is not None:
            if self.ghost_T is None:
                self.ghost_snap(ctrl.T, tcp)
            held = ctrl.grip > self.mapper.cfg.clutch_threshold and ctrl.tracked
            if held and not st["grabbed"]:
                st.update(grabbed=True, hand0=ctrl.T.copy(), ghost0=self.ghost_T.copy())
            elif held:
                h0, g0 = st["hand0"], st["ghost0"]
                R_rel = ctrl.T[:3, :3] @ h0[:3, :3].T
                yaw = float(np.arctan2(R_rel[0, 2], R_rel[2, 2]))          # rotation of the hand about XR +Y
                Ry = R.from_euler("y", yaw).as_matrix()
                T = np.eye(4)
                T[:3, :3] = Ry @ g0[:3, :3]
                T[:3, 3] = ctrl.T[:3, 3] + Ry @ (g0[:3, 3] - h0[:3, 3])    # pivot about the hand
                self.ghost_T = T
            elif not held:
                st["grabbed"] = False
        return Action(state="홀로그램 옮기기 (그립: 끌기/돌리기, A·B: 완료)", allow_teleop=False)

    def clear_path(self) -> None:
        self.path_epoch += 1
        self.flash("경로 지움")

    def ghost_snap(self, ctrl_T: np.ndarray, tcp: np.ndarray) -> None:
        """Place the ghost robot base so that its TCP (robot frame) coincides with the
        controller grip position (XR frame).  R_xr_rob = effective_r_map^T."""
        Rm = self.mapper.cfg.effective_r_map()
        R_xr_rob = Rm.T
        p_base = ctrl_T[:3, 3] - R_xr_rob @ np.asarray(tcp[:3], dtype=float)
        T = np.eye(4)
        T[:3, :3] = R_xr_rob
        T[:3, 3] = p_base
        self.ghost_T = T
        self.ghost_snap_pending = False
        self.flash("홀로그램을 손에 맞췄습니다")

    def ghost_pose(self) -> dict | None:
        if self.ghost_T is None:
            return None
        q = R.from_matrix(self.ghost_T[:3, :3]).as_quat()
        return {"p": [round(float(v), 4) for v in self.ghost_T[:3, 3]], "q": [round(float(v), 6) for v in q]}

    # ---- per-cycle -------------------------------------------------------
    def update(self, sample, ctrl, now: float, tcp: np.ndarray, dt: float, new_sample: bool = True) -> Action:
        """Called every cycle before the mapper.  ``ctrl`` is None when there is
        no fresh controller sample; ``new_sample`` is False when ``sample`` was
        already seen in an earlier cycle."""
        if new_sample and sample is not None and sample.head is not None:
            r = self.head.update(now, sample.head)
            if r == "still":
                self.detected = "headless"
            elif r == "moving":
                self.detected = "hmd"

        a = b = trig = stick = False
        if ctrl is not None:
            a = self.edges.rising("a", ctrl.primary)
            b = self.edges.rising("b", ctrl.secondary)
            trig = self.edges.rising("trig", ctrl.trigger > 0.8)
            stick = self.edges.rising("stick", ctrl.thumbstick_click)
        a, b, trig = a or ("a" in self._virtual), b or ("b" in self._virtual), trig or ("trig" in self._virtual)
        self._virtual.clear()
        self.stick_pressed_this_cycle = stick

        if stick and self.mode != "hmd":
            print("[ui] stick click ignored (headless mode: use the monitor page or --ui.mode hmd)")
        if stick and self.mode == "hmd" and self.flow is None and self.player is None:
            self.menu_open = not self.menu_open
            self.cursor = min(self.cursor, len(self._items()) - 1)
        elif a and self.a_opens_menu and not self.menu_open and self.mode == "hmd" and self.flow is None and self.player is None and self.ghost_move is None:
            self.menu_open = True
            self.cursor = min(self.cursor, len(self._items()) - 1)
            a = False

        if self.menu_open:
            if ctrl is not None:
                y = float(ctrl.thumbstick[1]) if len(ctrl.thumbstick) > 1 else 0.0
                if self._nav_armed and abs(y) > 0.6:
                    self.cursor = (self.cursor + (-1 if y < 0 else 1)) % len(self._items())
                    self._nav_armed = False
                elif abs(y) < 0.3:
                    self._nav_armed = True
            if a:
                self._select(self._items()[self.cursor][0])
            elif b:
                self.menu_open = False
            return Action(state="MENU (robot holding)", allow_teleop=False)

        if self.ghost_move is not None:
            return self._ghost_move_step(ctrl, tcp, a, b)

        if self.flow is not None:
            self.flow.update(Ctx(ctrl, tcp, now, dt, a, b, trig, self.mapper))
            if self.flow.done:
                self.last_result = self.flow.result
                self.flash(self.flow.result)
                self.flow = None
                self.mapper.disengage()
                return Action(state="flow finished", allow_teleop=False)
            return Action(state=f"{self.flow.title} {self.flow.step + 1}/{self.flow.n_steps()}",
                          allow_teleop=self.flow.allow_teleop)

        if self.player is not None:
            if b or (ctrl is not None and ctrl.secondary):
                self.stop_replay()
                return Action(state="replay aborted", allow_teleop=False)
            pose = self.player.step(tcp, dt, now)
            if pose is None:
                self.player = None
                self.flash("재생 완료")
                return Action(state="replay done", allow_teleop=False)
            st = "PLAYING approach" if self.player.phase == "approach" else f"PLAYING {self.player.progress()*100:.0f}%"
            return Action(state=st, allow_teleop=False, servo_pose=pose, gripper=self.player.gripper())

        return Action()

    def after_cycle(self, sample_json: dict | None, tcp: np.ndarray, engaged: bool, gripper: float) -> None:
        if self.recorder is not None and sample_json is not None:
            self.recorder.write(sample_json, tcp, engaged, gripper)
        if self.flow is not None:
            self.flow.after_cycle(tcp, engaged)

    def close(self) -> None:
        if self.recorder is not None:
            self.set_recording(False)

    # ---- status for clients ------------------------------------------------
    def ui(self) -> dict:
        cfg = self.mapper.cfg
        now = time.monotonic()
        d: dict = {
            "mode": self.mode, "mode_cfg": self.mode_cfg,
            "locks": {"x": cfg.lock_x, "y": cfg.lock_y, "z": cfg.lock_z, "rot": cfg.lock_rot},
            "scale": cfg.pos_scale, "map_mode": cfg.mode, "yaw": round(cfg.operator_yaw_deg, 1),
            "msg": self._msg if now < self._msg_until else "",
            "menu": None, "flow": None, "recording": None, "playing": None,
            "ghost": self.ghost_pose(), "path_epoch": self.path_epoch,
            "ghost_move": self.ghost_move is not None,
            "cams": self.robot_info.get("cams"), "view": self.robot_info.get("view"),
            "tcp_offset": self.robot_info.get("tcp_offset"), "home": self.home_tcp is not None,
        }
        if self.menu_open:
            d["menu"] = {"items": [{"key": k, "label": l, "value": v, "sel": i == self.cursor}
                                   for i, (k, l, v) in enumerate(self._items())]}
        if self.flow is not None:
            d["flow"] = {"name": self.flow.name, "title": self.flow.title, "step": self.flow.step,
                         "n": self.flow.n_steps(), "lines": [x for x in self.flow.lines() if x]}
        if self.recorder is not None:
            d["recording"] = {"path": os.path.basename(self.recorder.path), "frames": self.recorder.frames,
                              "seconds": round(self.recorder.seconds, 1)}
        if self.player is not None:
            d["playing"] = {"path": os.path.basename(getattr(self.player, "path", "")), "phase": self.player.phase,
                            "progress": round(self.player.progress(), 3)}
        return d
