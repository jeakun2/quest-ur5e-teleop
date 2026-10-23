"""UR5e (+ Robotiq 2F-85) in Isaac Sim 4.2 as a servo target for the teleop bridge.

Run with a python that has Isaac Sim 4.2 + numpy/scipy (e.g. the Isaac Lab conda env) through tools/isaac_sim.py:

    .venv\\Scripts\\python tools\\isaac_sim.py --headless          # physics only, fastest
    .venv\\Scripts\\python tools\\isaac_sim.py                     # with the Isaac Sim window
    .venv\\Scripts\\python tools\\isaac_sim.py --headless --selftest   # no bridge: square test

Then start the bridge with ``--robot.type isaac`` (host/port below).

Protocol (TCP, one JSON object per line, the sim is the server, several clients allowed):

    -> {"servo": [x, y, z, rx, ry, rz], "gripper": 0..1}   track this TCP pose (UR "Base" frame, rotation vector)
    -> {"hold": 1}                       freeze at the current pose
    -> {"get": 1, "gripper": 0..1}       just report (gripper optional on any message)
    -> {"shot": "C:/path/img.png"}       save a viewport screenshot (GUI mode only)
    -> {"quit": 1}                       close the simulator
    -> {"cam": {"ceiling": true}}        enable/disable overlay cameras (ceiling | side | ee); JPEGs at http://127.0.0.1:5561/cam/<name>.jpg
    -> {"view": "default|top|side|ee"}   main viewport camera preset
    -> {"reset": 1}                      re-initialise the whole scene (physics reset, home joints, gripper open, cube back);
                                         also done automatically when the physics state blows up (NaN / huge joint values)
    <- {"tcp": [6], "q": [6], "grip": 0..1, "t": sim_time, "hz": loop_hz, "tracking": bool, "ok": true,
        "cams": {"ceiling": bool, "side": bool, "ee": bool}, "view": "...", "tcp_offset": m}
    TCP = tool0 + tcp_offset along the tool Z axis (fingertip centre of the 2F-85 by default).

The UR5e is tracked with Lula RMPflow (Isaac Sim's bundled ur5e config, end effector ``tool0``).
A Robotiq 2F-85 is attached to ``tool0`` with the Robot Assembler (``--no-gripper`` to skip) and a
5 cm cube is placed under the home TCP for grasp tests.  Poses are reported in the UR controller's
*Base* frame (= URDF ``base_link`` rotated 180 deg about Z) so that a bridge config calibrated
here also fits a real UR5e.  If no servo arrives for ``--stale`` seconds the arm holds.
"""
import argparse
import json
import math
import os
import socket
import sys
import threading
import time

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--port", type=int, default=5560)
parser.add_argument("--host", default="127.0.0.1")
parser.add_argument("--hz", type=float, default=120.0, help="physics rate")
parser.add_argument("--render-hz", type=float, default=60.0, help="GUI render rate (physics substeps in between)")
parser.add_argument("--stale", type=float, default=0.3, help="s without servo -> hold")
parser.add_argument("--home", default="0,-1.5708,1.5708,-1.5708,-1.5708,0", help="home joints (rad)")
parser.add_argument("--usd", default="", help="override the UR5e USD path/URL")
parser.add_argument("--no-gripper", action="store_true", help="do not attach the Robotiq 2F-85")
parser.add_argument("--no-cube", action="store_true", help="do not add the test cube")
parser.add_argument("--finger-kp", type=float, default=0.0, help="override finger_joint drive stiffness (0 = keep USD)")
parser.add_argument("--finger-kd", type=float, default=0.0)
parser.add_argument("--finger-max-effort", type=float, default=0.0, help="override finger_joint max effort (0 = keep)")
parser.add_argument("--finger-rate", type=float, default=1.5, help="rad/s ramp of the finger target")
parser.add_argument("--finger-close", type=float, default=0.70, help="rad at gripper=1 (2F-85 pads touch ~0.7)")
parser.add_argument("--tcp-offset", type=float, default=-1.0, help="TCP = tool0 + this along tool Z (m); -1 = fingertip centre (auto)")
parser.add_argument("--no-cams", action="store_true", help="no overlay cameras (ceiling/side/ee)")
parser.add_argument("--cam-port", type=int, default=5561, help="HTTP port serving /cam/<name>.jpg")
parser.add_argument("--cam-res", default="320x240")
parser.add_argument("--cam-hz", type=float, default=8.0)
parser.add_argument("--log", default="", help="log file (default isaac/logs/sim_<time>.log)")
parser.add_argument("--selftest", action="store_true", help="track a 10 cm square without a client, then exit")
parser.add_argument("--duration", type=float, default=0.0, help="exit after this many seconds (0 = run until closed)")
parser.add_argument("--shot", default="", help="GUI: save a screenshot here once the scene is up")
args = parser.parse_args()

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_PATH = args.log or os.path.join(ROOT, "isaac", "logs", time.strftime("sim_%Y%m%d_%H%M%S.log"))
os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
_log = open(LOG_PATH, "w", encoding="utf-8")


def log(*a):
    s = "[isaac] " + " ".join(str(x) for x in a)
    print(s, flush=True)
    _log.write(s + "\n")
    _log.flush()


log("starting SimulationApp headless=%s log=%s" % (args.headless, LOG_PATH))
from omni.isaac.kit import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720,
                                "anti_aliasing": 0, "samples_per_pixel_per_frame": 1, "denoiser": False})
try:
    import carb.settings
    _st = carb.settings.get_settings()
    for k, v in {"/rtx/ambientOcclusion/enabled": False, "/rtx/reflections/enabled": False,
                 "/rtx/indirectDiffuse/enabled": False, "/rtx/post/dlss/execMode": 0,
                 "/rtx/post/aa/op": 0, "/rtx/shadows/enabled": True, "/rtx/sceneDb/ambientLightIntensity": 0.8}.items():
        _st.set(k, v)
except Exception:
    pass

import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

import carb  # noqa: E402
from omni.isaac.core import World  # noqa: E402
from omni.isaac.core.robots import Robot  # noqa: E402
from omni.isaac.core.objects import DynamicCuboid  # noqa: E402
from omni.isaac.core.utils.stage import add_reference_to_stage  # noqa: E402
from omni.isaac.core.utils.prims import get_prim_at_path  # noqa: E402
from omni.isaac.core.utils.types import ArticulationAction  # noqa: E402
from omni.isaac.motion_generation import ArticulationMotionPolicy, RmpFlow  # noqa: E402
from omni.isaac.motion_generation import interface_config_loader  # noqa: E402

try:
    import ctypes
    ctypes.windll.winmm.timeBeginPeriod(1)  # 1 ms sleep resolution (Windows)
except Exception:
    pass

# ------------------------------------------------------------------ frames
RZ = R.from_euler("z", math.pi)          # URDF base_link -> UR "Base" frame


def link_to_base(p, rot):
    """stage/base_link pose -> UR Base frame [x,y,z,rx,ry,rz]."""
    pb = RZ.apply(np.asarray(p, dtype=float))
    rb = RZ * rot
    return np.concatenate([pb, rb.as_rotvec()])


def base_to_link(pose6):
    pose6 = np.asarray(pose6, dtype=float)
    p = RZ.inv().apply(pose6[:3])
    rot = RZ.inv() * R.from_rotvec(pose6[3:])
    return p, rot


def quat_wxyz(rot):
    q = rot.as_quat()  # x y z w
    return np.array([q[3], q[0], q[1], q[2]])


# ------------------------------------------------------------------ scene
physics_dt = 1.0 / args.hz
render_dt = 1.0 / args.render_hz
# GUI: one app update per render frame; physics sub-steps render_dt/physics_dt times inside it.
loop_dt = physics_dt if args.headless else render_dt
world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)
world.scene.add_default_ground_plane()

usd = args.usd
assets_root = None
if not usd:
    from omni.isaac.nucleus import get_assets_root_path
    assets_root = get_assets_root_path()
    if assets_root is None:
        assets_root = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2"
        log("assets root not found via Nucleus, using", assets_root)
    usd = assets_root + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
log("UR5e usd:", usd)
ROBOT_PATH = "/World/ur5e"
add_reference_to_stage(usd_path=usd, prim_path=ROBOT_PATH)


def child_paths(path, depth=2):
    out = []
    prim = get_prim_at_path(path)
    if not prim or not prim.IsValid():
        return out
    stack = [(prim, 0)]
    while stack:
        p, d = stack.pop()
        for c in p.GetChildren():
            out.append(c.GetPath().pathString)
            if d + 1 < depth:
                stack.append((c, d + 1))
    return out


gripper_info = None
if not args.no_gripper:
    try:
        from omni.isaac.core.utils.extensions import enable_extension
        enable_extension("omni.isaac.robot_assembler")   # not loaded by the python app by default
        simulation_app.update()
        from omni.isaac.robot_assembler import RobotAssembler
        root = assets_root or "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2"
        gusd = root + "/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd"
        GRIP_PATH = "/World/gripper"
        add_reference_to_stage(usd_path=gusd, prim_path=GRIP_PATH)
        from pxr import Usd, UsdPhysics
        ur_children = child_paths(ROBOT_PATH, 1)
        log("ur5e prims:", [p.split("/")[-1] for p in ur_children])
        names = [p.split("/")[-1] for p in ur_children]
        base_mount = "/tool0" if "tool0" in names else ("/flange" if "flange" in names else "/wrist_3_link")
        # Link names must be unique inside the merged articulation (physx tensors: "Duplicate link name
        # 'base_link'" otherwise scrambles joint indexing) -> rename colliding gripper links.
        import omni.kit.commands
        ur_links = set(names)
        for pr in list(Usd.PrimRange(get_prim_at_path(GRIP_PATH))):
            if pr.HasAPI(UsdPhysics.RigidBodyAPI) and pr.GetName() in ur_links:
                src = pr.GetPath().pathString
                dst = src.rsplit("/", 1)[0] + "/gripper_" + pr.GetName()
                omni.kit.commands.execute("MovePrim", path_from=src, path_to=dst, keep_world_transform=False)
                log("renamed gripper link %s -> %s" % (src, dst))
        # the gripper's mount frame must be one of its rigid-body links (the fixed joint targets bodies)
        bodies = [pr.GetPath().pathString for pr in Usd.PrimRange(get_prim_at_path(GRIP_PATH))
                  if pr.HasAPI(UsdPhysics.RigidBodyAPI)]
        log("gripper rigid bodies:", [b[len(GRIP_PATH):] for b in bodies])
        cand = [b for b in bodies if "base" in b.split("/")[-1].lower()] or bodies[:1]
        attach_mount = cand[0][len(GRIP_PATH):] if cand else ""
        log("assembling gripper: base mount %s, attach mount %s" % (base_mount, attach_mount or "<root>"))
        asm = RobotAssembler()
        asm_res = asm.assemble_articulations(ROBOT_PATH, GRIP_PATH, base_mount, attach_mount,
                                             mask_all_collisions=True, single_robot=True)
        gripper_info = {"path": GRIP_PATH, "mount": base_mount, "asm": asm_res, "base_link": cand[0] if cand else GRIP_PATH}
    except Exception as e:
        log("gripper attach failed (continuing without):", repr(e))
        gripper_info = None
        try:
            from omni.isaac.core.utils.prims import delete_prim
            delete_prim("/World/gripper")
        except Exception:
            pass

GRIP_TIP = None          # fingertip centre in tool0 frame (after alignment)


def _xf(path):
    """world transform (4x4 numpy) of a prim at default time"""
    from pxr import UsdGeom, Usd
    m = UsdGeom.Xformable(get_prim_at_path(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(m).T   # pxr matrices are row-vector convention


def _find_prim(name):
    """path of the first prim named ``name`` under /World (the assembler may reparent the gripper)"""
    from pxr import Usd
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    for pr in Usd.PrimRange(stage.GetPrimAtPath("/World")):
        if pr.GetName() == name:
            return pr.GetPath().pathString
    raise RuntimeError("prim not found: " + name)


def _gripper_geom():
    """fingertip centre and opening axis in the tool0 frame, from the USD at default time:
    the finger links' bounding boxes (meshes carry the offsets; the link origins all sit at the base)."""
    from pxr import Usd, UsdGeom
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    def centre(path):
        r = cache.ComputeWorldBound(get_prim_at_path(path)).ComputeAlignedRange()
        return 0.5 * (np.array(r.GetMin(), dtype=float) + np.array(r.GetMax(), dtype=float))

    T_tool = _xf(ROBOT_PATH + gripper_info["mount"])
    pl, pr = centre(_find_prim("left_inner_finger")), centre(_find_prim("right_inner_finger"))
    if not getattr(_gripper_geom, "logged", False):
        _gripper_geom.logged = True
        log("tool0 at", np.round(T_tool[:3, 3], 3).tolist(), "left pad at", np.round(pl, 3).tolist(), "right pad at", np.round(pr, 3).tolist())
    inv = np.linalg.inv(T_tool)
    to_tool = lambda w: (inv @ np.array([w[0], w[1], w[2], 1.0]))[:3]  # noqa: E731
    pl_t, pr_t = to_tool(pl), to_tool(pr)
    return 0.5 * (pl_t + pr_t), pl_t - pr_t


TCP_OFFSET = 0.0

def align_gripper():
    """Rotate the gripper on the flange so its fingers point along tool0 +Z and open along tool0 +X.
    set_fixed_joint_transform also moves the gripper prim in USD, so this runs before the first reset."""
    global GRIP_TIP
    tip, opn = _gripper_geom()
    pz = tip / max(1e-9, np.linalg.norm(tip))
    ox = opn - pz * float(opn @ pz)
    ox = ox / max(1e-9, np.linalg.norm(ox))
    log("gripper measured in tool0: tip=%s open_axis=%s" % (np.round(tip, 3).tolist(), np.round(ox, 3).tolist()))
    if np.linalg.norm(tip) < 0.02:
        log("gripper geometry unavailable; leaving the mount as is")
        return
    if np.linalg.norm(pz - np.array([0, 0, 1.0])) > 0.05 or abs(abs(ox[0]) - 1) > 0.05:
        M_src = np.stack([ox, np.cross(pz, ox), pz], axis=1)
        best = None
        for cand_R in (M_src.T, M_src):
            q = R.from_matrix(cand_R).as_quat()
            gripper_info["asm"].set_fixed_joint_transform(np.zeros(3), np.array([q[3], q[0], q[1], q[2]]))
            simulation_app.update()
            tip2, opn2 = _gripper_geom()
            pz2 = tip2 / max(1e-9, np.linalg.norm(tip2))
            score = float(pz2[2]) + abs(float(opn2[0])) / max(1e-9, np.linalg.norm(opn2))
            log("  mount try: tip=%s pointing_z=%.2f score=%.2f" % (np.round(tip2, 3).tolist(), pz2[2], score))
            if best is None or score > best[0]:
                best = (score, cand_R)
        q = R.from_matrix(best[1]).as_quat()
        gripper_info["asm"].set_fixed_joint_transform(np.zeros(3), np.array([q[3], q[0], q[1], q[2]]))
        simulation_app.update()
        tip, opn = _gripper_geom()
    GRIP_TIP = tip
    log("gripper aligned: fingertip centre in tool0 = %s (len %.3f m), opening along %s" % (
        np.round(tip, 3).tolist(), float(np.linalg.norm(tip)), np.round(opn / max(1e-9, np.linalg.norm(opn)), 2).tolist()))


if gripper_info is not None:
    try:
        align_gripper()
    except Exception as e:
        log("gripper alignment failed:", repr(e))
TCP_OFFSET = args.tcp_offset if args.tcp_offset >= 0 else (float(GRIP_TIP[2]) if GRIP_TIP is not None else 0.0)
log("TCP offset along tool Z: %.3f m" % TCP_OFFSET)

robot = world.scene.add(Robot(prim_path=ROBOT_PATH, name="ur5e"))
FINGERS = []
if gripper_info is not None:
    try:
        from omni.isaac.core.prims import RigidPrim
        for n_ in ("left_inner_finger", "right_inner_finger"):
            FINGERS.append(world.scene.add(RigidPrim(prim_path=_find_prim(n_), name=n_)))
    except Exception as e:
        log("finger prims unavailable:", repr(e))
        FINGERS = []

home_q = np.array([float(v) for v in args.home.split(",")], dtype=float)
cube = None

if not args.headless:
    from omni.isaac.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([1.6, 1.4, 1.1]), target=np.array([0.0, 0.0, 0.35]))

world.reset()

if not args.headless:
    # Fit the render period once, with physics running: with a shared GPU a frame can take 20-70 ms.
    # rendering_dt = n * physics_dt (n sub-steps per rendered frame) keeps simulated time ~ wall time.
    # (Changing it while the servo loop runs freezes Kit, so it is done here, followed by a second reset.)
    for _ in range(30):
        world.step(render=True)
    t0 = time.perf_counter()
    N = 60
    for _ in range(N):
        world.step(render=True)
    frame = (time.perf_counter() - t0) / N
    # generous margin: frames get slower when the shared GPU is busy; a too-short period makes the sim lag
    n = int(min(12, max(6, round(frame * 2.0 / physics_dt))))
    log("measured render frame %.1f ms -> render period %.1f ms (%d physics substeps)" % (frame * 1000, n * physics_dt * 1000, n))
    if abs(n * physics_dt - render_dt) > 1e-6:
        render_dt = n * physics_dt
        loop_dt = render_dt
        world.stop()
        world.set_simulation_dt(physics_dt=physics_dt, rendering_dt=render_dt)
        world.reset()

rmp_cfg = interface_config_loader.load_supported_motion_policy_config("UR5e", "RMPflow")
rmp = RmpFlow(**rmp_cfg)
rmp.set_robot_base_pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
policy = ArticulationMotionPolicy(robot, rmp, loop_dt)
active = policy.get_active_joints_subset()
dof_names = list(robot.dof_names)
log("dofs (%d):" % len(dof_names), dof_names)
log("active joints:", rmp.get_active_joints())

finger_idx = None
FINGER_RANGE = (0.0, 0.8)
if gripper_info is not None:
    for cand in ("finger_joint", "robotiq_85_left_knuckle_joint", "left_outer_knuckle_joint"):
        if cand in dof_names:
            finger_idx = dof_names.index(cand)
            break
    if finger_idx is None:
        extra = [n for n in dof_names if n not in rmp.get_active_joints()]
        if extra:
            finger_idx = dof_names.index(extra[0])
    log("gripper drive dof:", dof_names[finger_idx] if finger_idx is not None else None)
    if finger_idx is not None:
        try:
            lo, hi = robot.dof_properties["lower"][finger_idx], robot.dof_properties["upper"][finger_idx]
            if np.isfinite(lo) and np.isfinite(hi) and hi > lo:
                FINGER_RANGE = (float(lo), float(hi))
        except Exception:
            pass
        log("gripper dof range:", FINGER_RANGE)
        try:
            dp = robot.dof_properties
            for n_ in dof_names[6:]:
                i_ = dof_names.index(n_)
                log("  dof %-32s type=%s stiff=%.1f damp=%.1f maxF=%.1f lim=(%.2f,%.2f)" % (
                    n_, dp["type"][i_], dp["stiffness"][i_], dp["damping"][i_], dp["maxEffort"][i_], dp["lower"][i_], dp["upper"][i_]))
            if args.finger_kp > 0:
                av = robot._articulation_view
                av.set_gains(kps=np.array([[args.finger_kp]]), kds=np.array([[args.finger_kd]]), joint_indices=np.array([finger_idx]))
                log("finger_joint gains set kp=%.1f kd=%.1f" % (args.finger_kp, args.finger_kd))
            if args.finger_max_effort > 0:
                robot._articulation_view.set_max_efforts(np.array([[args.finger_max_effort]]), joint_indices=np.array([finger_idx]))
                log("finger_joint max effort %.1f" % args.finger_max_effort)
        except Exception as e:
            log("dof property log failed:", repr(e))

active.set_joint_positions(home_q)
active.set_joint_velocities(np.zeros(6))
world.step(render=not args.headless)


def ee_pose_base():
    """TCP (= tool0 + TCP_OFFSET along tool Z) in the UR Base frame, and the arm joints."""
    q = np.asarray(active.get_joint_positions(), dtype=float)
    p, rotm = rmp.get_end_effector_pose(q)
    rot = R.from_matrix(rotm)
    p_tcp = np.asarray(p, dtype=float) + rot.apply([0.0, 0.0, TCP_OFFSET])
    return link_to_base(p_tcp, rot), q


def tcp_target_to_tool0(pose6):
    """bridge TCP target (Base frame) -> tool0 target in the stage frame for RMPflow"""
    p, rot = base_to_link(pose6)
    return p - rot.apply([0.0, 0.0, TCP_OFFSET]), rot


tcp0, _ = ee_pose_base()
log("home joints:", np.round(home_q, 3).tolist(), "TCP(base):", np.round(tcp0, 3).tolist())

if not args.no_cube:
    # 5 cm cube on the ground right under the home TCP (UR Base frame -> stage frame via RZ^-1)
    p_cube = RZ.inv().apply(np.array([tcp0[0], tcp0[1], 0.025]))
    cube = world.scene.add(DynamicCuboid(prim_path="/World/cube", name="cube", position=p_cube,
                                         size=0.05, color=np.array([0.9, 0.3, 0.1]), mass=0.05))
    log("test cube at base(x,y)=(%.3f, %.3f), z=0.025" % (tcp0[0], tcp0[1]))

# ------------------------------------------------------------------ overlay cameras
CAMS = {}            # name -> {"rp", "ann", "on", "jpg", "t"}
VIEW = {"name": "default"}
CAM_W, CAM_H = (int(v) for v in args.cam_res.lower().split("x"))


def _cam_setup():
    from omni.isaac.core.utils.extensions import enable_extension
    enable_extension("omni.replicator.core")
    simulation_app.update()
    import omni.replicator.core as rep
    from omni.isaac.core.utils.prims import create_prim
    from omni.isaac.core.utils.viewports import set_camera_view
    base_stage = RZ.inv().apply(np.array([tcp0[0], tcp0[1], 0.0]))     # under the home TCP, stage frame
    specs = {
        "ceiling": dict(eye=[0.0, 0.0, 2.3], target=[base_stage[0] * 0.5, base_stage[1] * 0.5, 0.0]),
        "side": dict(eye=[base_stage[0] + 0.1, -1.6, 0.6], target=[base_stage[0] * 0.5, 0.0, 0.35]),
    }
    for name, sp in specs.items():
        path = f"/World/cams/{name}"
        create_prim(path, "Camera", position=np.array(sp["eye"]), attributes={"focalLength": 18.0, "clippingRange": (0.05, 20.0)})
        set_camera_view(eye=np.array(sp["eye"]), target=np.array(sp["target"]), camera_prim_path=path)
        CAMS[name] = {"path": path}
    # end-effector camera: child of tool0, looking along tool +Z (camera looks down its -Z) from beside the gripper
    ee_path = ROBOT_PATH + "/tool0/ee_cam"
    q_ee = R.from_euler("x", 180, degrees=True) * R.from_euler("z", 0, degrees=True)
    qq = q_ee.as_quat()
    create_prim(ee_path, "Camera", translation=np.array([0.0, -0.07, 0.02]),
                orientation=np.array([qq[3], qq[0], qq[1], qq[2]]), attributes={"focalLength": 12.0, "clippingRange": (0.02, 10.0)})
    CAMS["ee"] = {"path": ee_path}
    for name, c in CAMS.items():
        c.update({"rp": None, "ann": None, "on": False, "jpg": None, "t": 0.0})
    log("overlay cameras ready (render products are created when a camera is switched on):",
        list(CAMS), "%dx%d" % (CAM_W, CAM_H))


def _cam_set(name, on):
    c = CAMS.get(name)
    if not c:
        return
    c["on"] = bool(on)
    import omni.replicator.core as rep
    if c["on"] and c.get("rp") is None:
        rp = rep.create.render_product(c["path"], (CAM_W, CAM_H), name=f"cam_{name}")
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach([rp])
        c.update({"rp": rp, "ann": ann, "tries": 0, "jpg": None})
    elif not c["on"] and c.get("rp") is not None:
        try:
            c["ann"].detach()
        except Exception:
            pass
        try:
            c["rp"].destroy()
        except Exception:
            pass
        c.update({"rp": None, "ann": None, "jpg": None})


def _cam_grab(now):
    import io as _io
    from PIL import Image
    for name, c in CAMS.items():
        if not c["on"] or c.get("ann") is None or now - c["t"] < 1.0 / args.cam_hz:
            continue
        try:
            data = c["ann"].get_data()
            c["tries"] = c.get("tries", 0) + 1
            if c["tries"] <= 3 or c["tries"] % 300 == 0:
                shp = getattr(data, "shape", None) if not isinstance(data, dict) else {k: getattr(v, "shape", type(v).__name__) for k, v in data.items()}
                log("cam %s get_data try %d: type=%s shape=%s" % (name, c["tries"], type(data).__name__, shp))
            if isinstance(data, dict):
                data = data.get("data", None)
            if data is None or getattr(data, "size", 0) == 0:
                continue
            img = Image.fromarray(np.asarray(data)[:, :, :3])
            buf = _io.BytesIO()
            img.save(buf, format="JPEG", quality=70)
            c["jpg"], c["t"] = buf.getvalue(), now
        except Exception as e:
            c["t"] = now
            if c.get("err") != repr(e):
                c["err"] = repr(e)
                log("cam %s grab failed: %r" % (name, e))


def _cam_http():
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path.startswith("/cam/") and self.path.endswith(".jpg"):
                name = self.path[5:-4].split("?")[0]
                c = CAMS.get(name)
                if c and c.get("jpg"):
                    self.send_response(200); self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(c["jpg"]))); self.send_header("Cache-Control", "no-store")
                    self.end_headers(); self.wfile.write(c["jpg"]); return
                self.send_response(404); self.end_headers(); return
            if self.path.startswith("/cams"):
                body = json.dumps({n: c.get("on", False) for n, c in CAMS.items()}).encode()
                self.send_response(200); self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body); return
            self.send_response(404); self.end_headers()

    srv = ThreadingHTTPServer(("127.0.0.1", args.cam_port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    log("camera http on 127.0.0.1:%d (/cam/<name>.jpg)" % args.cam_port)


VIEW_PRESETS = {
    "default": (np.array([1.6, 1.4, 1.1]), np.array([0.0, 0.0, 0.35])),
    "top": (np.array([0.0, 0.0, 2.6]), np.array([0.0, 0.0, 0.0])),
    "side": (np.array([0.0, -2.2, 0.7]), np.array([0.0, 0.0, 0.35])),
}


def _apply_view():
    if args.headless:
        return
    from omni.isaac.core.utils.viewports import set_camera_view
    name = VIEW["name"]
    if name == "ee":
        q = np.asarray(active.get_joint_positions(), dtype=float)
        p, rotm = rmp.get_end_effector_pose(q)
        rot = R.from_matrix(rotm)
        eye = np.asarray(p) + rot.apply([0.0, -0.25, -0.15])
        target = np.asarray(p) + rot.apply([0.0, 0.0, TCP_OFFSET + 0.1])
        set_camera_view(eye=eye, target=target)
    elif name in VIEW_PRESETS:
        eye, tgt = VIEW_PRESETS[name]
        set_camera_view(eye=eye, target=tgt)


if not args.headless and not args.no_cams:
    try:
        _cam_setup()
        _cam_http()
    except Exception as e:
        log("overlay cameras failed:", repr(e))
        CAMS.clear()

# ------------------------------------------------------------------ shared state
lock = threading.Lock()
state = {"target": None, "target_t": 0.0, "hold": True, "tcp": tcp0, "q": home_q, "t": 0.0,
         "hz": 0.0, "tracking": False, "shot": None, "clients": 0, "quit": False, "grip": 0.0,
         "cam_req": {}, "view_req": None, "reset_req": None, "resets": 0}


def handle_client(conn, addr):
    log("client connected", addr)
    with lock:
        state["clients"] += 1
    f = conn.makefile("rb")
    try:
        for line in f:
            try:
                d = json.loads(line)
            except Exception:
                continue
            with lock:
                if "servo" in d:
                    state["target"] = np.asarray(d["servo"], dtype=float)
                    state["target_t"] = time.monotonic()
                    state["hold"] = False
                elif "hold" in d:
                    state["hold"] = True
                if "gripper" in d:
                    try:
                        state["grip"] = min(1.0, max(0.0, float(d["gripper"])))
                    except (TypeError, ValueError):
                        pass
                if "shot" in d:
                    state["shot"] = d["shot"]
                if "quit" in d:
                    state["quit"] = True
                if isinstance(d.get("cam"), dict):
                    state["cam_req"].update({str(k): bool(v) for k, v in d["cam"].items()})
                if "view" in d:
                    state["view_req"] = str(d["view"])
                if "reset" in d:
                    state["reset_req"] = "requested by client"
                rep = {"tcp": [round(float(v), 5) for v in state["tcp"]],
                       "q": [round(float(v), 4) for v in state["q"]],
                       "grip": round(state["grip"], 3),
                       "t": round(state["t"], 4), "hz": round(state["hz"], 1),
                       "tracking": state["tracking"], "ok": True,
                       "cams": {n: c.get("on", False) for n, c in CAMS.items()}, "view": VIEW["name"],
                       "tcp_offset": round(TCP_OFFSET, 4), "resets": state["resets"]}
            conn.sendall((json.dumps(rep) + "\n").encode())
    except Exception as e:
        log("client error", addr, repr(e))
    finally:
        with lock:
            state["clients"] -= 1
        try:
            conn.close()
        except Exception:
            pass
        log("client closed", addr)


def serve():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(4)
    log("listening on %s:%d" % (args.host, args.port))
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()


if not args.selftest:
    threading.Thread(target=serve, daemon=True).start()

_last_shot = [0.0]


def take_shot(path):
    """Capture the viewport; keep stepping the world (rendered) until the capture lands.
    Captures closer than 1.5 s apart come out black, so those are skipped."""
    now = time.perf_counter()
    if now - _last_shot[0] < 1.5:
        log("screenshot skipped (too soon):", path)
        return
    _last_shot[0] = now
    try:
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        capture_viewport_to_file(get_active_viewport(), path)
        for _ in range(12):
            policy.move(loop_dt)
            world.step(render=True)
        log("screenshot ->", path)
    except Exception as e:
        log("screenshot failed:", repr(e))


_finger_cmd = [None]


def apply_gripper(g, dt):
    if finger_idx is None:
        return
    lo = FINGER_RANGE[0]
    hi = min(FINGER_RANGE[1], args.finger_close)
    want = lo + g * (hi - lo)
    cur = _finger_cmd[0]
    if cur is None:
        cur = float(active_all_q()[finger_idx])
    step = args.finger_rate * dt
    cur = cur + max(-step, min(step, want - cur))
    _finger_cmd[0] = cur
    robot.get_articulation_controller().apply_action(
        ArticulationAction(joint_positions=np.array([cur]), joint_indices=np.array([finger_idx])))


def active_all_q():
    return np.asarray(robot.get_joint_positions(), dtype=float)


# ------------------------------------------------------------------ scene reset
def scene_reset(reason):
    """Full re-initialisation: physics reset (bodies back to their USD start state), home joints,
    RMPflow state cleared, gripper open, cube back under the home TCP, targets dropped (hold)."""
    global hold_pose, tcp0
    log("SCENE RESET (%s)" % reason)
    try:
        world.reset()
        active.set_joint_positions(home_q)
        active.set_joint_velocities(np.zeros(6))
        if finger_idx is not None:
            try:
                robot.set_joint_positions(np.zeros(len(dof_names) - 6), joint_indices=np.arange(6, len(dof_names)))
                robot.set_joint_velocities(np.zeros(len(dof_names) - 6), joint_indices=np.arange(6, len(dof_names)))
            except Exception:
                pass
        _finger_cmd[0] = None
        try:
            rmp.reset()
            rmp.set_robot_base_pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
        except Exception as e:
            log("rmp reset:", repr(e))
        world.step(render=False)
        t, q = ee_pose_base()
        tcp0 = t
        with lock:
            state["target"], state["hold"], state["grip"] = None, True, 0.0
            state["tcp"], state["q"], state["tracking"] = t, q, False
            state["resets"] += 1
        hold_pose = None
        log("scene reset done: TCP(base)=%s" % np.round(t, 3).tolist())
        return True
    except Exception as e:
        log("scene reset FAILED:", repr(e))
        return False


def physics_broken(tcp, q):
    if not np.all(np.isfinite(tcp)) or not np.all(np.isfinite(q)):
        return "non-finite state"
    if np.any(np.abs(q) > 50.0) or np.linalg.norm(tcp[:3]) > 3.0:
        return "joint/TCP values exploded"
    return None


# ------------------------------------------------------------------ main loop
log("READY")
t_wall0 = time.perf_counter()
sim_t0 = world.current_time
next_t = t_wall0
cycles = 0
catchup = 0
last_report = t_wall0
cycles_at_report = 0
hold_pose = None
shot_pending = args.shot or None
shot_at = t_wall0 + (8.0 if shot_pending else 0)

if args.selftest:
    sq = [np.array([0.10, 0, 0, 0, 0, 0]), np.array([0.10, 0.10, 0, 0, 0, 0]),
          np.array([0, 0.10, 0, 0, 0, 0]), np.zeros(6)]
    st_results = []

try:
    while simulation_app.is_running():
        now = time.perf_counter()
        el = now - t_wall0
        if args.duration and el > args.duration:
            break
        if state["quit"]:
            log("quit requested")
            break
        if args.selftest:
            k = int(el // 2.0)
            if k >= len(sq) + 1:
                break
            tgt = tcp0 + sq[min(k, len(sq) - 1)] if k < len(sq) else tcp0
            with lock:
                state["target"], state["target_t"], state["hold"] = tgt, now, False
                state["grip"] = 1.0 if (k % 2 == 1) else 0.0

        with lock:
            tgt = state["target"]
            stale = (time.monotonic() - state["target_t"]) > args.stale
            hold = state["hold"] or stale or tgt is None
            shot = state["shot"]
            state["shot"] = None
            grip = state["grip"]
            cam_req, state["cam_req"] = state["cam_req"], {}
            view_req, state["view_req"] = state["view_req"], None
        for k, v in cam_req.items():
            _cam_set(k, v)
            log("cam %s -> %s" % (k, "on" if v else "off"))
        if view_req is not None and (view_req in VIEW_PRESETS or view_req == "ee"):
            VIEW["name"] = view_req
            log("view -> %s" % view_req)
        if VIEW["name"] == "ee" or view_req is not None:
            _apply_view()
        with lock:
            reset_req, state["reset_req"] = state["reset_req"], None
        if reset_req is None:
            bad = physics_broken(state["tcp"], state["q"])
            if bad:
                reset_req = "auto: " + bad
        if reset_req:
            scene_reset(reset_req)
            with lock:
                tgt = None
            hold = True
        if hold:
            if hold_pose is None:
                hold_pose = state["tcp"].copy()
            tgt = hold_pose
        else:
            hold_pose = None
        p, rot = tcp_target_to_tool0(tgt)
        rmp.set_end_effector_target(p, quat_wxyz(rot))
        policy.move(loop_dt)
        apply_gripper(grip, loop_dt)
        world.step(render=not args.headless)
        tcp, q = ee_pose_base()
        cycles += 1
        with lock:
            state["tcp"], state["q"], state["t"], state["tracking"] = tcp, q, world.current_time, not hold
        if CAMS:
            _cam_grab(now)

        if shot:
            take_shot(shot)
        if shot_pending and now > shot_at and not args.headless:
            take_shot(shot_pending)
            shot_pending = None

        if now - last_report > 1.0:
            hz = (cycles - cycles_at_report) / (now - last_report)
            with lock:
                state["hz"] = hz
            if not args.headless and hz < 0.7 / loop_dt:
                log("warning: render frames are slower than the fitted period (%.0f Hz < %.0f): sim time lags wall time"
                    % (hz, 1.0 / loop_dt))
            err = float(np.linalg.norm(tcp[:3] - tgt[:3]))
            log("t=%6.1fs %5.1f Hz %s tcp=%s err=%.4f grip=%.2f clients=%d sim_t=%.1f" % (
                el, hz, "HOLD " if hold else "TRACK", np.round(tcp[:3], 3).tolist(), err, grip, state["clients"],
                world.current_time - sim_t0))
            if args.selftest:
                st_results.append((round(el, 1), np.round(tcp[:3], 4).tolist(), np.round(tgt[:3], 4).tolist(), round(err, 4)))
            last_report, cycles_at_report = now, cycles

        # real-time pacing
        next_t += loop_dt
        rem = next_t - time.perf_counter()
        if rem > 0:
            time.sleep(rem)
        elif rem < -0.5:
            next_t = time.perf_counter()
except KeyboardInterrupt:
    pass
finally:
    if args.selftest:
        # targets switch every 2 s; judge the rows logged ~1 s after each switch (odd seconds)
        errs = [r[3] for r in st_results if int(r[0]) % 2 == 1]
        ok = bool(errs) and max(errs) < 0.01
        log("SELFTEST", "PASS" if ok else "FAIL", "max err after settle=%.4f" % (max(errs) if errs else -1))
        with open(os.path.join(os.path.dirname(LOG_PATH), "selftest.json"), "w") as f:
            json.dump({"ok": ok, "home_tcp": tcp0.tolist(), "rows": st_results,
                       "gripper": gripper_info is not None and finger_idx is not None}, f, indent=1)
    log("closing after %d cycles" % cycles)
    _log.close()
    simulation_app.close()
