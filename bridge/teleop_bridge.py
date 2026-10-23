"""Quest 3 -> UR5e teleoperation bridge (main entry point).

    python bridge/teleop_bridge.py                       # config.yaml defaults (webxr + dummy)
    python bridge/teleop_bridge.py --robot.type ur_rtde --robot.ip 192.168.1.10
    python bridge/teleop_bridge.py --source.type oculus_reader
    python bridge/teleop_bridge.py --record runs/session1.jsonl
    python bridge/teleop_bridge.py --source.type replay --source.path runs/session1.jsonl
    python bridge/teleop_bridge.py --ui.mode headless     # controller only, menu disabled

Loop (robot.frequency Hz):
    sample  = source.latest()
    action  = session.update(...)        menu / flows / playback / mode detection
    target  = mapper.update(controller pose, clutch, robot TCP)
    robot.servo(target)   (or robot.hold() when disengaged / stale / unsafe)

Basic control (works with the headset off, see docs/OPERATION.md):
    hold GRIP (middle finger) + move  -> TCP follows in XYZ and orientation
    TRIGGER                            -> gripper
    A                                  -> orientation following on/off
    B                                  -> stop while held
    thumbstick click (headset worn)    -> menu: tutorial, calibration, axis locks,
                                          recording / playback, scale, mapping mode
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from bridge.gripper import make_gripper  # noqa: E402
from bridge.mapping import MapperConfig, TeleopMapper  # noqa: E402
from bridge.robots.dummy_robot import DummyRobot  # noqa: E402
from bridge.session import UISession  # noqa: E402
from bridge.sources.base import Sample  # noqa: E402
from bridge.trajectory import Recorder  # noqa: E402

DEFAULT_CONFIG = os.path.join(_HERE, "config.yaml")
ROOT = os.path.dirname(_HERE)


# ---------------------------------------------------------------- config
def _set_dotted(d: dict, key: str, value: str) -> None:
    parts = key.split(".")
    for p in parts[:-1]:
        d = d.setdefault(p, {})
    try:
        d[parts[-1]] = yaml.safe_load(value)
    except Exception:
        d[parts[-1]] = value


def load_config(argv=None) -> tuple[dict, argparse.Namespace]:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--record", default="", help="record from start into this JSONL file (controller + TCP)")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = run forever)")
    ap.add_argument("--quiet", action="store_true")
    args, rest = ap.parse_known_args(argv)
    with open(args.config, encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    i = 0
    while i < len(rest):
        tok = rest[i]
        if tok.startswith("--") and "." in tok:
            if "=" in tok:
                k, v = tok[2:].split("=", 1)
            else:
                k, v = tok[2:], rest[i + 1] if i + 1 < len(rest) else "true"
                i += 1
            _set_dotted(cfg, k, v)
        else:
            ap.error(f"unknown argument {tok}")
        i += 1
    return cfg, args


# ---------------------------------------------------------------- factories
def make_source(cfg: dict):
    t = cfg.get("type", "webxr")
    if t == "webxr":
        from bridge.sources.webxr_source import WebXRSource
        return WebXRSource(port=int(cfg.get("port", 8765)), tls=bool(cfg.get("tls", True)),
                           cam_proxy=str(cfg.get("cam_proxy", "http://127.0.0.1:5561")))
    if t == "oculus_reader":
        from bridge.sources.oculus_reader_source import OculusReaderSource
        return OculusReaderSource(ip_address=cfg.get("ip_address") or None, adb_path=cfg.get("adb_path") or None,
                                  keep_awake=bool(cfg.get("keep_awake", True)))
    if t == "replay":
        from bridge.sources.replay_source import ReplaySource
        return ReplaySource(cfg["path"], loop=bool(cfg.get("loop", True)))
    raise SystemExit(f"unknown source.type {t}")


def make_monitor(cfg: dict, source):
    """Optional second WebXRSource that only serves web/monitor.html + status/commands
    when the main source is not the WebXR server (oculus_reader, Unity on another port)."""
    ui = cfg.get("ui", {})
    want = ui.get("monitor", "auto")
    stype = cfg.get("source", {}).get("type", "webxr")
    if want == "auto":
        want = stype == "oculus_reader"
    if not want or stype == "webxr":
        return None
    from bridge.sources.webxr_source import WebXRSource
    return WebXRSource(port=int(ui.get("monitor_port", 8765)), tls=bool(ui.get("monitor_tls", True)))


def make_robot(cfg: dict):
    t = cfg.get("type", "dummy")
    if t == "dummy":
        return DummyRobot()
    if t == "ur_rtde":
        from bridge.robots.ur_rtde_robot import URRtdeRobot
        return URRtdeRobot(cfg["ip"], float(cfg.get("frequency", 125)), float(cfg.get("lookahead", 0.1)),
                           float(cfg.get("gain", 300)), bool(cfg.get("auto_power", False)),
                           float(cfg.get("speed_slider", 1.0)))
    if t == "isaac":
        from bridge.robots.isaac_robot import IsaacSimRobot
        return IsaacSimRobot(str(cfg.get("host", "127.0.0.1")), int(cfg.get("port", 5560)))
    raise SystemExit(f"unknown robot.type {t}")


def make_mapper(cfg: dict) -> TeleopMapper:
    mc = MapperConfig()
    for k in ("operator_yaw_deg", "pos_scale", "rot_enabled", "clutch", "clutch_threshold", "clutch_toggle",
              "pos_filter_alpha", "rot_filter_alpha", "max_lin_speed", "max_ang_speed", "max_reach",
              "min_reach", "stale_timeout", "lock_x", "lock_y", "lock_z", "lock_rot", "mode"):
        if k in cfg and cfg[k] is not None:
            setattr(mc, k, cfg[k])
    if "workspace_min" in cfg:
        mc.workspace_min = np.array(cfg["workspace_min"], dtype=float)
    if "workspace_max" in cfg:
        mc.workspace_max = np.array(cfg["workspace_max"], dtype=float)
    if cfg.get("r_map"):
        mc.r_map = np.array(cfg["r_map"], dtype=float).reshape(3, 3)
    c = cfg.get("center_xr")
    if c and cfg.get("tcp_center"):
        T = np.eye(4)
        T[:3, :3] = R.from_quat(c["q"]).as_matrix()
        T[:3, 3] = c["p"]
        mc.center_T = T
        mc.tcp_center = np.array(cfg["tcp_center"], dtype=float)
    return TeleopMapper(mc)


# ---------------------------------------------------------------- recording
def sample_to_json(s: Sample) -> dict:
    def c(ctrl):
        if ctrl is None:
            return None
        return {"p": ctrl.T[:3, 3].round(5).tolist(), "q": R.from_matrix(ctrl.T[:3, :3]).as_quat().round(6).tolist(),
                "grip": round(ctrl.grip, 3), "trigger": round(ctrl.trigger, 3), "a": ctrl.primary, "b": ctrl.secondary,
                "stick": ctrl.thumbstick_click, "axes": [round(float(a), 3) for a in ctrl.thumbstick],
                "tracked": ctrl.tracked}
    d = {"t": s.t, "right": c(s.right), "left": c(s.left)}
    if s.head is not None:
        d["head"] = {"p": s.head[:3, 3].round(5).tolist(), "q": R.from_matrix(s.head[:3, :3]).as_quat().round(6).tolist()}
    return d


# ---------------------------------------------------------------- main loop
def run(cfg: dict, args: argparse.Namespace) -> int:
    source = make_source(cfg.get("source", {}))
    monitor = make_monitor(cfg, source)
    robot = make_robot(cfg.get("robot", {}))
    gripper = make_gripper(cfg.get("gripper", {}), robot=robot)
    mapper = make_mapper(cfg.get("mapping", {}))
    hand = cfg.get("mapping", {}).get("hand", "right")
    btn = cfg.get("buttons", {})
    rot_toggle_btn = btn.get("rot_toggle", "none")          # A now opens the options menu (session)
    soft_stop_btn = btn.get("soft_stop", "secondary")
    dt = 1.0 / float(cfg.get("robot", {}).get("frequency", 125))
    rec_dir = cfg.get("recording", {}).get("dir", os.path.join(ROOT, "runs"))
    if not os.path.isabs(rec_dir):
        rec_dir = os.path.join(ROOT, rec_dir)

    session = UISession(cfg.get("ui", {}), mapper, rec_dir=rec_dir, config_path=getattr(args, "config", None), hand=hand)
    session.meta = {"source": source.name, "robot": robot.name, "hand": hand,
                    "pos_scale": mapper.cfg.pos_scale, "yaw": mapper.cfg.operator_yaw_deg}
    if args.record:
        session.recorder = Recorder(args.record, meta=session.meta)
    stop = {"flag": False}
    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("flag", True))

    print(f"[bridge] source={source.name} robot={robot.name} gripper={gripper.name} hand={hand} dt={dt*1000:.1f}ms "
          f"ui.mode={session.mode_cfg}")
    robot.connect()
    gripper.connect()
    try:
        session.home_tcp = np.asarray(robot.tcp_pose(), dtype=float).copy()
        session.robot_info = dict(robot.extra())
    except Exception:
        session.home_tcp = None
    source.start()
    if monitor is not None:
        monitor.start()
    scfg = cfg.get("source", {})
    if scfg.get("type", "webxr") == "webxr" and str(scfg.get("qr", False)).lower() in ("1", "true", "yes"):
        from bridge.netinfo import print_quest_qr
        print_quest_qr(int(scfg.get("port", 8765)), bool(scfg.get("tls", True)), os.path.join(ROOT, "qr", "quest_url.png"))
    print("[bridge] running. Ctrl+C to stop.")

    t_start = time.monotonic()
    last_seen_count = -1
    last_sample_t = 0.0
    last_print = time.monotonic()
    cycles_at_print = 0
    last_status_push = 0.0
    rot_prev = False
    servoing = False
    state = "idle"
    cycles = 0
    try:
        while not stop["flag"]:
            t0 = robot.begin_cycle()
            now = time.monotonic()
            if args.duration and now - t_start > args.duration:
                break

            for c in source.pop_commands() + (monitor.pop_commands() if monitor else []):
                session.command(c)
            while session.robot_cmds:
                robot.command(session.robot_cmds.pop(0))

            ok, why = robot.ok()
            s = source.latest()
            ctrl = None
            new_sample = False
            if s is not None:
                ctrl = s.right if hand == "right" else s.left
                if source.count != last_seen_count:
                    last_seen_count = source.count
                    last_sample_t = now
                    new_sample = True
            fresh = (now - last_sample_t) < mapper.cfg.stale_timeout
            tcp = robot.tcp_pose()
            act = session.update(s, ctrl if fresh else None, now, tcp, dt, new_sample)
            gripper_cmd = ctrl.trigger if ctrl is not None else 0.0

            if not ok:
                state = f"UNSAFE: {why}"
                mapper.disengage()
                session.stop_replay()
                robot.hold()
                servoing = False
            elif act.servo_pose is not None:                      # trajectory playback
                robot.servo(act.servo_pose, dt)
                servoing = True
                state = act.state or "PLAYING"
                if act.gripper is not None:
                    gripper_cmd = act.gripper
                    gripper.command(gripper_cmd)
            elif not act.allow_teleop:                            # menu / flow step without teleop
                if mapper.engaged:
                    mapper.disengage()
                if servoing:
                    robot.hold()
                    servoing = False
                state = act.state or "paused"
            elif ctrl is None or not fresh:
                state = "no controller" if ctrl is None else "stale"
                if mapper.engaged:
                    mapper.disengage()
                if servoing:
                    robot.hold()
                    servoing = False
            else:
                # buttons
                rot_now = getattr(ctrl, rot_toggle_btn, False) if rot_toggle_btn != "none" else False
                if rot_now and not rot_prev:
                    mapper.cfg.rot_enabled = not mapper.cfg.rot_enabled
                    print(f"[bridge] orientation following -> {mapper.cfg.rot_enabled}")
                rot_prev = rot_now
                if getattr(ctrl, soft_stop_btn, False):
                    state = "soft stop (B held)"
                    mapper.disengage()
                    if servoing:
                        robot.hold()
                        servoing = False
                else:
                    out = mapper.update(ctrl.T, ctrl.grip, ctrl.trigger, tcp, dt, ctrl.tracked)
                    if out.engaged and session.ghost_snap_pending:
                        session.ghost_snap(ctrl.T, tcp)
                    if os.environ.get("TELEOP_DEBUG") and out.engaged and cycles % 15 == 0:
                        print(f"[dbg] ctrl={np.round(ctrl.T[:3, 3], 3).tolist()} tcp={np.round(tcp[:3], 3).tolist()} "
                              f"target={None if out.target is None else np.round(out.target[:3], 3).tolist()} "
                              f"lim={out.rate_limited} clamp={out.clamped}", flush=True)
                    if out.engaged and out.target is not None:
                        robot.servo(out.target, dt)
                        servoing = True
                        state = "ENGAGED" + (" clamp" if out.clamped else "") + (" rate" if out.rate_limited else "")
                    else:
                        if servoing:
                            robot.hold()
                            servoing = False
                        state = "holding (squeeze grip to engage)"
                    gripper.command(ctrl.trigger)
                if act.state:
                    state = f"{act.state} | {state}"

            session.after_cycle(sample_to_json(s) if new_sample else None, robot.tcp_pose(), mapper.engaged, gripper_cmd)

            cycles += 1
            if not args.quiet and now - last_print > 1.0:
                print(f"[bridge] {state:<36} mode={session.mode} {(cycles - cycles_at_print) / (now - last_print):.0f}Hz "
                      f"{source.status()}")
                last_print, cycles_at_print = now, cycles
            if now - last_status_push > 1.0 / 15:
                last_status_push = now
                q = robot.joints()
                session.robot_info = dict(robot.extra())
                msg = {"type": "status", "state": state, "rot": mapper.cfg.rot_enabled, "hand": hand,
                       "tcp": [round(float(v), 4) for v in robot.tcp_pose()], "robot": robot.name,
                       "q": None if q is None else [round(float(v), 4) for v in q],
                       "gripper": round(float(gripper_cmd), 3), "engaged": bool(mapper.engaged),
                       "ui": session.ui()}
                for src in (source, monitor):
                    if src is not None and hasattr(src, "push_status"):
                        src.push_status(msg)
            robot.end_cycle(t0, dt)
    finally:
        print("[bridge] stopping ...")
        robot.hold()
        session.close()
        source.stop()
        if monitor is not None:
            monitor.stop()
        gripper.disconnect()
        robot.disconnect()
        print(f"[bridge] done. cycles={cycles} samples={source.count}")
    return 0


def main(argv=None) -> int:
    cfg, args = load_config(argv)
    return run(cfg, args)


if __name__ == "__main__":
    sys.exit(main())
