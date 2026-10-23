import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bridge.mapping import DEFAULT_R_MAP, MapperConfig, TeleopMapper, pose_to_T, T_to_pose  # noqa: E402

TCP0 = np.array([0.4, -0.1, 0.35, 2.22, -2.22, 0.0])
DT = 1 / 125


def xr_pose(p, rotvec=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(rotvec).as_matrix()
    T[:3, 3] = p
    return T


def settle(m, T, grip, tcp, n=400):
    out = None
    for _ in range(n):
        out = m.update(T, grip, 0.0, tcp, DT)
        tcp = out.target if out.target is not None else tcp
    return out, tcp


def test_default_r_map_is_rotation():
    assert np.allclose(DEFAULT_R_MAP @ DEFAULT_R_MAP.T, np.eye(3))
    assert np.isclose(np.linalg.det(DEFAULT_R_MAP), 1.0)
    # forward (-Z xr) -> +X robot, up (+Y xr) -> +Z robot, left (-X xr) -> +Y robot
    assert np.allclose(DEFAULT_R_MAP @ [0, 0, -1], [1, 0, 0])
    assert np.allclose(DEFAULT_R_MAP @ [0, 1, 0], [0, 0, 1])
    assert np.allclose(DEFAULT_R_MAP @ [-1, 0, 0], [0, 1, 0])


def test_pose_roundtrip():
    assert np.allclose(T_to_pose(pose_to_T(TCP0)), TCP0)


def test_no_motion_without_clutch():
    m = TeleopMapper(MapperConfig(pos_filter_alpha=1.0))
    out = m.update(xr_pose([0, 1, 0]), grip=0.0, trigger=0.0, robot_tcp=TCP0, dt=DT)
    assert not out.engaged and out.target is None
    out = m.update(xr_pose([0.5, 1, 0]), grip=0.0, trigger=0.0, robot_tcp=TCP0, dt=DT)
    assert not out.engaged


def test_engage_latches_and_tracks_forward_motion():
    m = TeleopMapper(MapperConfig(pos_filter_alpha=1.0, rot_filter_alpha=1.0))
    out = m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT)
    assert out.engaged and np.allclose(out.target, TCP0, atol=1e-9)  # latch = no jump
    out, tcp = settle(m, xr_pose([0, 1, -0.10]), 1.0, TCP0)           # 10 cm forward in XR
    assert np.allclose(tcp[:3], TCP0[:3] + [0.10, 0, 0], atol=1e-6)
    assert np.allclose(tcp[3:], TCP0[3:], atol=1e-6)


def test_scale_and_up_motion():
    m = TeleopMapper(MapperConfig(pos_scale=0.5, pos_filter_alpha=1.0))
    m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT)
    out, tcp = settle(m, xr_pose([0, 1.2, 0]), 1.0, TCP0)
    assert np.allclose(tcp[:3], TCP0[:3] + [0, 0, 0.10], atol=1e-6)


def test_release_regrip_ratchets():
    """Move 10 cm, release, move the controller back, re-grip: robot stays."""
    m = TeleopMapper(MapperConfig(pos_filter_alpha=1.0))
    m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT)
    out, tcp = settle(m, xr_pose([0, 1, -0.10]), 1.0, TCP0)
    out = m.update(xr_pose([0, 1, -0.10]), 0.0, 0.0, tcp, DT)
    assert not out.engaged
    out = m.update(xr_pose([0, 1, 0]), 0.0, 0.0, tcp, DT)  # controller back, released
    out = m.update(xr_pose([0, 1, 0]), 1.0, 0.0, tcp, DT)  # re-grip: latch at current tcp
    assert out.engaged and np.allclose(out.target, tcp, atol=1e-9)
    out, tcp2 = settle(m, xr_pose([0, 1, -0.10]), 1.0, tcp)
    assert np.allclose(tcp2[:3], TCP0[:3] + [0.20, 0, 0], atol=1e-6)


def test_rate_limit_per_cycle():
    m = TeleopMapper(MapperConfig(pos_filter_alpha=1.0, max_lin_speed=0.25))
    m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT)
    out = m.update(xr_pose([0, 1, -0.30]), 1.0, 0.0, TCP0, DT)  # 30 cm jump
    assert out.rate_limited
    assert np.linalg.norm(out.target[:3] - TCP0[:3]) <= 0.25 * DT + 1e-9


def test_workspace_clamp_and_reach():
    cfg = MapperConfig(pos_filter_alpha=1.0, workspace_max=np.array([0.5, 0.8, 0.8]))
    m = TeleopMapper(cfg)
    m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT)
    out, tcp = settle(m, xr_pose([0, 1, -0.50]), 1.0, TCP0, n=1500)  # wants x=0.9
    assert out.clamped and np.isclose(tcp[0], 0.5, atol=1e-6)
    cfg2 = MapperConfig(pos_filter_alpha=1.0, max_reach=0.6)
    m2 = TeleopMapper(cfg2)
    m2.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT)
    out, tcp = settle(m2, xr_pose([0, 1, -0.50]), 1.0, TCP0, n=1500)
    assert np.linalg.norm(tcp[:3]) <= 0.6 + 1e-6


def test_rotation_mapping_yaw():
    """Yaw the controller +90 deg about XR up: robot tool yaws +90 deg about base Z."""
    m = TeleopMapper(MapperConfig(pos_filter_alpha=1.0, rot_filter_alpha=1.0, max_ang_speed=10.0))
    m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT)
    out, tcp = settle(m, xr_pose([0, 1, 0], rotvec=[0, np.pi / 2, 0]), 1.0, TCP0, n=800)
    expected = (R.from_euler("z", 90, degrees=True) * R.from_rotvec(TCP0[3:])).as_rotvec()
    assert np.allclose(R.from_rotvec(tcp[3:]).as_matrix(), R.from_rotvec(expected).as_matrix(), atol=1e-5)


def test_rotation_disabled_keeps_orientation():
    m = TeleopMapper(MapperConfig(pos_filter_alpha=1.0, rot_enabled=False))
    m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT)
    out, tcp = settle(m, xr_pose([0, 1, -0.1], rotvec=[0.5, 0.3, 0.1]), 1.0, TCP0)
    assert np.allclose(tcp[3:], TCP0[3:], atol=1e-9)


def test_operator_yaw_offset():
    """With operator_yaw_deg=90 the operator's 'forward' maps to robot +Y."""
    m = TeleopMapper(MapperConfig(pos_filter_alpha=1.0, operator_yaw_deg=90))
    m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT)
    out, tcp = settle(m, xr_pose([0, 1, -0.10]), 1.0, TCP0)
    assert np.allclose(tcp[:3], TCP0[:3] + [0, 0.10, 0], atol=1e-6)


def test_untracked_disengages():
    m = TeleopMapper(MapperConfig(pos_filter_alpha=1.0))
    assert m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT).engaged
    assert not m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT, tracked=False).engaged


def test_toggle_clutch():
    m = TeleopMapper(MapperConfig(pos_filter_alpha=1.0, clutch_toggle=True))
    assert m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT).engaged   # press
    assert m.update(xr_pose([0, 1, 0]), 0.0, 0.0, TCP0, DT).engaged   # release: still engaged
    assert not m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT).engaged  # press again: off


def test_axis_locks_hold_latched_value():
    m = TeleopMapper(MapperConfig(pos_filter_alpha=1.0, rot_filter_alpha=1.0, lock_z=True, lock_rot=True, max_ang_speed=10))
    m.update(xr_pose([0, 1, 0]), 1.0, 0.0, TCP0, DT)
    out, tcp = settle(m, xr_pose([0, 1.2, -0.1], rotvec=[0, 0.8, 0]), 1.0, TCP0, n=800)
    assert np.allclose(tcp[:3], TCP0[:3] + [0.10, 0, 0], atol=1e-6)   # z stayed, x moved
    assert np.allclose(tcp[3:], TCP0[3:], atol=1e-9)                   # rotation locked


def test_absolute_mode_is_repeatable():
    """Same hand position -> same TCP, even after releasing and re-gripping elsewhere."""
    center = xr_pose([0, 1, 0])
    cfg = MapperConfig(pos_filter_alpha=1.0, rot_filter_alpha=1.0, mode="absolute", center_T=center,
                       tcp_center=TCP0.copy(), max_lin_speed=5.0)
    m = TeleopMapper(cfg)
    out = m.update(xr_pose([0, 1, -0.2]), 1.0, 0.0, TCP0, DT)
    assert out.engaged and out.rate_limited                               # no jump: rate-limited from the current TCP
    assert np.linalg.norm(out.target[:3] - TCP0[:3]) <= 5.0 * DT + 1e-9
    out, tcp = settle(m, xr_pose([0, 1, -0.2]), 1.0, TCP0, n=400)
    assert np.allclose(tcp[:3], TCP0[:3] + [0.20, 0, 0], atol=1e-6)
    m.update(xr_pose([0, 1, -0.2]), 0.0, 0.0, tcp, DT)                   # release
    out, tcp2 = settle(m, xr_pose([0, 1, 0]), 1.0, tcp, n=400)           # re-grip at the centre -> back to TCP0
    assert np.allclose(tcp2[:3], TCP0[:3], atol=1e-6)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
