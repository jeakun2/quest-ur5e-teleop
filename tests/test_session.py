"""Unit tests for bridge/session.py (menu, mode detection, flows, locks,
recording/playback) and bridge/trajectory.py, driven with synthetic
controller samples - no headset, no robot."""
import json
import os
import sys

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bridge.mapping import DEFAULT_R_MAP, MapperConfig, TeleopMapper  # noqa: E402
from bridge.session import (HeadMotion, UISession, r_map_from_gestures,  # noqa: E402
                            yaw_from_forward_gesture)
from bridge.sources.base import Controller, Sample  # noqa: E402
from bridge.trajectory import Recorder, TrajectoryPlayer, load_tcp_frames  # noqa: E402

TCP0 = np.array([0.4, -0.1, 0.35, 2.22, -2.22, 0.0])
DT = 1 / 125


def ctrl(p=(0, 1, 0), grip=0.0, trigger=0.0, a=False, b=False, stick=False, axes=(0.0, 0.0), rotvec=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(rotvec).as_matrix()
    T[:3, 3] = p
    return Controller(T=T, grip=grip, trigger=trigger, primary=a, secondary=b, thumbstick=axes, thumbstick_click=stick)


def head(p=(0, 1.6, 0)):
    T = np.eye(4)
    T[:3, 3] = p
    return T


def mk(tmp_path, mode="auto", **mcfg):
    mapper = TeleopMapper(MapperConfig(pos_filter_alpha=1.0, rot_filter_alpha=1.0, max_lin_speed=5.0, **mcfg))
    cfg_path = tmp_path / "config.yaml"
    cfg_path.write_text(yaml.safe_dump({"mapping": {"operator_yaw_deg": 0}}), encoding="utf-8")
    s = UISession({"mode": mode, "head_still_s": 1.0}, mapper, rec_dir=str(tmp_path / "runs"), config_path=str(cfg_path))
    return s, mapper, cfg_path


def tick(s, c, t, tcp=TCP0, hp=None):
    smp = Sample(t=t, right=c, head=head(hp) if hp is not None else None)
    return s.update(smp, c, t, tcp, DT)


# ---------------------------------------------------------------- mode detection
def test_head_motion_classifier():
    hm = HeadMotion(window_s=1.0, pos_m=0.02, ang_deg=3)
    assert hm.update(0.0, head()) is None
    assert hm.update(0.5, head()) is None                      # window not full yet
    assert hm.update(1.0, head()) == "still"
    assert hm.update(1.5, head((0.1, 1.6, 0))) == "moving"


def test_auto_mode_headless_when_head_still_and_menu_ignored(tmp_path):
    s, m, _ = mk(tmp_path)
    assert s.mode == "headless"
    for i in range(30):                                        # head moving -> hmd
        tick(s, ctrl(), i * 0.1, hp=(0.05 * np.sin(i), 1.6, 0))
    assert s.mode == "hmd"
    for i in range(30):                                        # head still for >1 s -> headless
        tick(s, ctrl(), 3 + i * 0.1, hp=(0, 1.6, 0))
    assert s.mode == "headless"
    tick(s, ctrl(stick=True), 7.0, hp=(0, 1.6, 0))             # stick click ignored when headless
    assert not s.menu_open
    tick(s, ctrl(), 7.1, hp=(0, 1.6, 0))
    s.mode_cfg = "hmd"                                         # forced hmd: stick opens the menu
    tick(s, ctrl(stick=True), 7.2, hp=(0, 1.6, 0))
    assert s.menu_open


def test_teleop_still_allowed_headless_without_head(tmp_path):
    s, m, _ = mk(tmp_path, mode="auto")
    act = tick(s, ctrl(grip=1.0), 0.0)
    assert act.allow_teleop and act.servo_pose is None and s.mode == "headless"


# ---------------------------------------------------------------- menu
def test_menu_navigation_and_lock_toggle(tmp_path):
    s, m, _ = mk(tmp_path, mode="hmd")
    act = tick(s, ctrl(stick=True), 0.0)
    assert s.menu_open and not act.allow_teleop
    tick(s, ctrl(), 0.01)
    keys = [k for k, _, _ in s._items()]
    target = keys.index("lock_z")
    for i in range(target):                                    # stick down (positive y) moves the cursor
        tick(s, ctrl(axes=(0, 1.0)), 0.1 + i * 0.1)
        tick(s, ctrl(axes=(0, 0.0)), 0.15 + i * 0.1)
    assert s.cursor == target
    tick(s, ctrl(a=True), 5.0)                                 # A = select -> Z lock on
    assert m.cfg.lock_z is True
    tick(s, ctrl(), 5.1)
    tick(s, ctrl(b=True), 5.2)                                 # B closes the menu
    assert not s.menu_open
    tick(s, ctrl(), 5.3)
    assert tick(s, ctrl(grip=1.0), 5.4).allow_teleop


def test_commands_from_monitor(tmp_path):
    s, m, _ = mk(tmp_path)
    s.command({"cmd": "lock", "axis": "x", "on": True})
    s.command({"cmd": "scale", "value": 0.5})
    s.command({"cmd": "ui_mode", "value": "hmd"})
    assert m.cfg.lock_x and m.cfg.pos_scale == 0.5 and s.mode == "hmd"
    s.command({"cmd": "map_mode", "value": "absolute"})        # refused: not calibrated
    assert m.cfg.mode == "relative"
    ui = s.ui()
    assert ui["locks"]["x"] and ui["scale"] == 0.5 and "절대" in ui["msg"]


# ---------------------------------------------------------------- calibration maths
def test_yaw_from_forward_gesture():
    assert yaw_from_forward_gesture(np.array([0, 0, -1.0]), DEFAULT_R_MAP) == pytest.approx(0.0)
    # operator stands at the robot's left, facing robot -Y: pushing "forward" (-Z xr) must map to +X
    # default map sends -Z xr -> +X, so a gesture that the default maps to +Y needs yaw -90
    v = np.array([-1.0, 0, 0])                                 # default: -X xr -> +Y robot
    yaw = yaw_from_forward_gesture(v, DEFAULT_R_MAP)
    assert yaw == pytest.approx(-90.0)
    cfg = MapperConfig(operator_yaw_deg=yaw)
    assert np.allclose(cfg.effective_r_map() @ v, [1, 0, 0], atol=1e-9)


def test_r_map_from_gestures_recovers_default():
    vs = [DEFAULT_R_MAP.T @ e for e in np.eye(3)]              # XR directions of robot X, Y, Z
    Rm = r_map_from_gestures(vs)
    assert np.allclose(Rm, DEFAULT_R_MAP, atol=1e-9)


def test_calib_flow_end_to_end(tmp_path):
    s, m, cfg_path = mk(tmp_path, mode="hmd")
    s.command({"cmd": "calib"})
    assert s.flow is not None and s.flow.name == "calib"
    t = 0.0
    # step 1: grip + push 25 cm along XR -X (operator stands at the robot's left) + release
    for i in range(20):
        act = tick(s, ctrl(p=(-0.0125 * i, 1, 0), grip=1.0), t); t += DT
        assert not act.allow_teleop
    tick(s, ctrl(p=(-0.25, 1, 0), grip=0.0), t); t += DT
    assert s.flow.step == 1 and m.cfg.operator_yaw_deg == pytest.approx(-90.0)
    # step 2: trigger at the neutral position
    tick(s, ctrl(p=(-0.25, 1, 0), trigger=1.0), t); t += DT
    assert s.flow.step == 2 and s.flow.center_T is not None
    tick(s, ctrl(p=(-0.25, 1, 0)), t); t += DT
    # step 3: verify by teleoperating 10 cm "forward" (XR -X): the bridge runs the mapper
    tcp = TCP0.copy()
    for i in range(30):
        c = ctrl(p=(-0.25 - 0.1 * min(i / 20, 1), 1, 0), grip=1.0)
        act = tick(s, c, t, tcp)
        assert act.allow_teleop
        out = m.update(c.T, c.grip, c.trigger, tcp, DT)
        if out.target is not None:
            tcp = out.target
        s.after_cycle(None, tcp, m.engaged, 0.0)
        t += DT
    c = ctrl(p=(-0.35, 1, 0), grip=0.0)
    tick(s, c, t, tcp); m.update(c.T, 0.0, 0.0, tcp, DT); s.after_cycle(None, tcp, m.engaged, 0.0); t += DT
    assert s.flow.step == 3, s.flow.lines()
    assert s.flow.verify_delta[0] == pytest.approx(0.10, abs=0.01) and s.flow.verify_err_deg < 5
    # step 4: A saves to config.yaml
    tick(s, ctrl(p=(-0.35, 1, 0), a=True), t); t += DT
    assert s.flow is None and "저장" in s.last_result
    saved = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))["mapping"]
    assert saved["operator_yaw_deg"] == pytest.approx(-90.0) and "center_xr" in saved and "tcp_center" in saved
    assert m.cfg.center_T is not None
    s.command({"cmd": "map_mode", "value": "absolute"})        # now allowed
    assert m.cfg.mode == "absolute"


def test_calib_flow_cancel_restores_yaw(tmp_path):
    s, m, _ = mk(tmp_path, operator_yaw_deg=30.0)
    s.command({"cmd": "calib"})
    for i in range(10):
        tick(s, ctrl(p=(0, 1, -0.03 * i), grip=1.0), i * DT)
    tick(s, ctrl(p=(0, 1, -0.3)), 0.2)
    assert s.flow.step == 1 and m.cfg.operator_yaw_deg == pytest.approx(0.0)
    s.command({"cmd": "cancel"})
    tick(s, ctrl(p=(0, 1, -0.3)), 0.3)
    assert s.flow is None and m.cfg.operator_yaw_deg == pytest.approx(30.0)


# ---------------------------------------------------------------- tutorial
def test_tutorial_progresses_through_all_steps(tmp_path):
    s, m, _ = mk(tmp_path, mode="hmd")
    s.command({"cmd": "tutorial"})
    t = 0.0
    tcp = TCP0.copy()

    def cycle(c):
        nonlocal t, tcp
        act = tick(s, c, t, tcp)
        if act.allow_teleop:
            out = m.update(c.T, c.grip, c.trigger, tcp, DT)
            if out.target is not None:
                tcp = out.target
        s.after_cycle(None, tcp, m.engaged, c.trigger)
        t += DT

    for i in range(30):                                        # step 1: grip + move 10 cm
        cycle(ctrl(p=(0, 1, -0.01 * i), grip=1.0))
    assert s.flow.step == 1
    cycle(ctrl(p=(0, 1, -0.3)))                                # release
    cycle(ctrl(p=(0, 1, 0), grip=1.0))                         # re-grip elsewhere
    assert s.flow.step == 2
    cycle(ctrl(trigger=1.0))                                   # step 3: trigger
    assert s.flow.step == 3
    cycle(ctrl(a=True)); cycle(ctrl()); cycle(ctrl(a=True)); cycle(ctrl())   # step 4: A twice
    assert s.flow.step == 4
    cycle(ctrl(b=True)); cycle(ctrl())                         # step 5: B
    assert s.flow.step == 5
    cycle(ctrl(stick=True))                                    # step 6: stick click ends it
    assert s.flow is None and s.last_result == "튜토리얼 완료"
    assert np.allclose(tcp[:3], TCP0[:3] + [0.29, 0, 0], atol=1e-6)   # the robot really moved


# ---------------------------------------------------------------- recording / playback
def test_recorder_and_player(tmp_path):
    path = str(tmp_path / "runs" / "rec.jsonl")
    rec = Recorder(path, meta={"robot": "dummy"})
    for i in range(50):
        tcp = TCP0 + [0.002 * i, 0, 0, 0, 0, 0]
        rec.write({"t": i * 0.02, "right": None, "left": None}, tcp, True, 0.3)
    rec.close()
    frames = load_tcp_frames(path)
    assert len(frames) == 50 and frames[0]["gripper"] == 0.3
    with open(path, encoding="utf-8") as f:
        assert json.loads(f.readline())["meta"]["robot"] == "dummy"

    # robot starts 20 cm away: the player must approach within the speed limit first, then follow
    start = TCP0 + [-0.2, 0, 0, 0, 0, 0]
    pl = TrajectoryPlayer(frames, max_lin_speed=0.5, max_ang_speed=1.0)
    cur = start.copy()
    now = 100.0
    steps = 0
    prev = cur.copy()
    while True:
        pose = pl.step(cur, DT, now)
        if pose is None:
            break
        assert np.linalg.norm(pose[:3] - prev[:3]) <= 0.5 * DT + 1e-9
        prev = pose
        cur = pose
        now += DT
        steps += 1
        if steps == 10:
            assert pl.phase == "approach"
    assert pl.done and np.allclose(cur[:3], frames[-1]["tcp"][:3], atol=1e-6)
    assert 0.2 / (0.5 * DT) < steps < 0.2 / (0.5 * DT) + 1.0 / DT + 5   # approach ~0.4 s + play 1 s


def test_session_record_then_replay(tmp_path):
    s, m, _ = mk(tmp_path, mode="hmd")
    s.command({"cmd": "record"})
    assert s.recorder is not None
    t = 0.0
    tcp = TCP0.copy()
    for i in range(40):
        c = ctrl(p=(0, 1, -0.005 * i), grip=1.0)
        act = tick(s, c, t, tcp)
        out = m.update(c.T, c.grip, 0.0, tcp, DT)
        tcp = out.target if out.target is not None else tcp
        s.after_cycle({"t": t, "right": {"p": [0, 1, -0.005 * i]}, "left": None}, tcp, m.engaged, 0.0)
        t += DT
    s.command({"cmd": "record", "on": False})
    assert s.recorder is None and s.ui()["recording"] is None
    s.command({"cmd": "replay"})
    assert s.player is not None
    # playback drives the robot through servo_pose while teleop is blocked
    cur = TCP0.copy()
    n = 0
    while s.player is not None and n < 2000:
        act = tick(s, ctrl(), t, cur)
        assert not act.allow_teleop
        if act.servo_pose is not None:
            cur = act.servo_pose
        t += DT
        n += 1
    assert s.player is None and np.allclose(cur[:3], tcp[:3], atol=1e-6)
    # B aborts a running playback
    s.command({"cmd": "replay"})
    act = tick(s, ctrl(b=True), t, cur)
    assert s.player is None and "aborted" in act.state
