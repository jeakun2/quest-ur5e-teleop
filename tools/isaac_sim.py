r"""Launch isaac/ur5e_sim.py with a python that has Isaac Sim 4.2 + numpy/scipy.

    .venv\Scripts\python tools\isaac_sim.py [--headless] [--log isaac\logs\sim.log] [sim args...]

Environment: by default the conda env ``isaaclab`` (created when installing Isaac Lab; the standalone
``python.bat`` of Isaac Sim lacks numpy). Change it with --conda-root/--conda-env, or pass --isaac <root>
to use <root>/python.bat instead. Everything after the launcher options goes to ur5e_sim.py.
"""
import argparse
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def conda_env_vars(conda_root: str, env: str) -> dict:
    out = subprocess.check_output(
        f'cmd /c "call "{conda_root}\\Scripts\\activate.bat" {env} >nul 2>&1 && set"', shell=True, text=True,
        errors="replace")
    e = {}
    for line in out.splitlines():
        if "=" in line and not line.startswith("="):
            k, v = line.split("=", 1)
            e[k] = v
    return e


def main() -> int:
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--isaac", default="", help="use <isaac>/python.bat instead of the conda env")
    ap.add_argument("--conda-root", default=r"D:\Anaconda")
    ap.add_argument("--conda-env", default="isaaclab")
    ap.add_argument("-h", "--help", action="store_true")
    a, rest = ap.parse_known_args()
    if a.help:
        print(__doc__)
        rest = ["--help"]
    script = os.path.join(ROOT, "isaac", "ur5e_sim.py")
    if a.isaac:
        bat = os.path.join(a.isaac, "python.bat")
        env = dict(os.environ)
        env.pop("PYTHONPATH", None)
        cmd = ["cmd", "/c", "call", bat, script] + rest
        cwd = a.isaac
    else:
        env = conda_env_vars(a.conda_root, a.conda_env)
        if "ISAAC_PATH" not in env:
            print(f"conda env {a.conda_env} did not set ISAAC_PATH (is it the Isaac Lab env?)")
            return 2
        py = os.path.join(a.conda_root, "envs", a.conda_env, "python.exe")
        cmd = [py, script] + rest
        cwd = env["ISAAC_PATH"]
    print("[isaac_sim] " + " ".join(cmd), flush=True)
    return subprocess.call(cmd, cwd=cwd, env=env)


if __name__ == "__main__":
    sys.exit(main())
