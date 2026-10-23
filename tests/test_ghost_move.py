"""'홀로그램 옮기기': drag the hologram with the grip, rotate it with the wrist, finish with B."""
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bridge.mapping import MapperConfig, TeleopMapper  # noqa: E402
from bridge.session import UISession  # noqa: E402
from bridge.sources.base import Controller  # noqa: E402


def ctrl(p, yaw_deg=0.0, grip=0.0, b=False):
    T = np.eye(4)
    T[:3, :3] = R.from_euler("y", yaw_deg, degrees=True).as_matrix()
    T[:3, 3] = p
    return Controller(T=T, grip=grip, trigger=0.0, primary=False, secondary=b, thumbstick_click=False,
                      thumbstick=(0.0, 0.0), tracked=True)


def test_ghost_move_translates_rotates_and_updates_yaw(tmp_path):
    m = TeleopMapper(MapperConfig())
    s = UISession({"mode": "hmd"}, m, rec_dir=str(tmp_path), config_path=None, hand="right")
    tcp = np.array([-0.49, -0.13, 0.37, 2.22, 2.22, 0.0])
    s.ghost_snap(ctrl([0.0, 1.0, -0.5]).T, tcp)
    g0 = s.ghost_T.copy()
    s.command({"cmd": "ghost_move"})
    assert s.ghost_move is not None and s.ui()["ghost_move"]
    act = s.update(None, ctrl([0.0, 1.0, -0.5], grip=1.0), 0.0, tcp, 0.008)   # grab at the same spot
    assert not act.allow_teleop
    s.update(None, ctrl([0.2, 1.0, -0.5], yaw_deg=90, grip=1.0), 0.1, tcp, 0.008)   # drag +0.2 m X and turn 90 deg
    T = s.ghost_T
    # pivot about the hand: hand moved +0.2 and the base rotated 90 deg about XR +Y around the hand position
    expect_p = np.array([0.2, 1.0, -0.5]) + R.from_euler("y", 90, degrees=True).apply(g0[:3, 3] - np.array([0.0, 1.0, -0.5]))
    assert np.allclose(T[:3, 3], expect_p, atol=1e-6)
    assert np.allclose(T[:3, :3] @ np.array([0, 0, 1.0]), [0, 1, 0], atol=1e-9)   # robot Z still up
    s.update(None, ctrl([0.2, 1.0, -0.5], yaw_deg=90, grip=0.0), 0.2, tcp, 0.008)   # release: stays
    assert np.allclose(s.ghost_T, T)
    s.update(None, ctrl([0.2, 1.0, -0.5], yaw_deg=90, b=True), 0.3, tcp, 0.008)     # B finishes
    assert s.ghost_move is None
    assert abs(abs(m.cfg.operator_yaw_deg) - 90.0) < 1e-6
    # the mapping now matches the hologram: effective_r_map^T == ghost rotation
    assert np.allclose(m.cfg.effective_r_map().T, s.ghost_T[:3, :3], atol=1e-6)
