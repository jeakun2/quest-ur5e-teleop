"""Options menu: A opens it, home pose playback, Isaac camera/view requests, rotation toggle."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bridge.mapping import MapperConfig, TeleopMapper  # noqa: E402
from bridge.session import UISession  # noqa: E402
from bridge.sources.base import Controller, Sample  # noqa: E402


def ctrl(a=False, b=False, stick=False):
    T = np.eye(4); T[:3, 3] = [0, 1, -0.4]
    return Controller(T=T, grip=0.0, trigger=0.0, primary=a, secondary=b, thumbstick_click=stick,
                      thumbstick=(0.0, 0.0), tracked=True)


def make(tmp_path):
    m = TeleopMapper(MapperConfig())
    s = UISession({"mode": "hmd"}, m, rec_dir=str(tmp_path), config_path=None, hand="right")
    return s, m


def test_a_opens_menu_and_selects(tmp_path):
    s, m = make(tmp_path)
    tcp = np.array([0.4, -0.1, 0.35, 2.22, -2.22, 0.0])
    s.update(None, ctrl(), 0.0, tcp, 0.008)
    act = s.update(None, ctrl(a=True), 0.1, tcp, 0.008)     # A rising edge -> menu opens, robot holds
    assert s.menu_open and not act.allow_teleop
    keys = [k for k, _, _ in s._items()]
    assert keys[:2] == ["home", "rot"] and "cam_ee" in keys and "view" in keys
    s.update(None, ctrl(), 0.2, tcp, 0.008)                   # release
    s.cursor = keys.index("rot")
    s.update(None, ctrl(a=True), 0.3, tcp, 0.008)             # A selects -> rotation following toggled
    assert m.cfg.rot_enabled is False
    s.update(None, ctrl(), 0.4, tcp, 0.008)
    s.update(None, ctrl(b=True), 0.5, tcp, 0.008)             # B closes
    assert not s.menu_open


def test_home_pose_playback(tmp_path):
    s, m = make(tmp_path)
    home = np.array([0.4, -0.1, 0.35, 2.22, -2.22, 0.0])
    s.home_tcp = home.copy()
    s.command({"cmd": "home"})
    assert s.player is not None
    tcp = home + np.array([0.3, 0, 0, 0, 0, 0])
    now = 0.0
    for _ in range(600):
        act = s.update(None, None, now, tcp, 0.008)
        if act.servo_pose is None:
            break
        tcp = act.servo_pose
        now += 0.008
    assert s.player is None and np.allclose(tcp[:3], home[:3], atol=1e-3)


def test_cam_and_view_requests_go_to_robot(tmp_path):
    s, m = make(tmp_path)
    s.command({"cmd": "cam", "name": "ee", "on": True})
    assert s.robot_cmds == [] and "오버레이" in s.ui()["msg"]      # no cams on this robot -> refused
    s.robot_info = {"cams": {"ceiling": False, "side": False, "ee": False}, "view": "default", "tcp_offset": 0.163}
    s.command({"cmd": "cam", "name": "ee", "on": True})
    s.command({"cmd": "view"})
    assert s.robot_cmds == [{"cam": {"ee": True}}, {"view": "top"}]
    ui = s.ui()
    assert ui["cams"]["ee"] is False and ui["tcp_offset"] == 0.163 and ui["home"] is False
