from __future__ import annotations

import numpy as np


class Robot:
    """Minimal Cartesian servo interface used by the bridge."""

    name = "base"

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def tcp_pose(self) -> np.ndarray:
        """Current TCP pose [x,y,z,rx,ry,rz] in the base frame."""
        raise NotImplementedError
    def servo(self, pose: np.ndarray, dt: float) -> None:
        """Track ``pose`` for one cycle of length ``dt``."""
        raise NotImplementedError
    def joints(self):
        """Joint angles (rad, URDF order) or None if the back-end cannot provide them."""
        return None
    def command(self, d: dict) -> None:
        """Back-end specific request (Isaac: overlay cameras / viewport preset). Ignored by default."""
    def extra(self) -> dict:
        """Back-end specific state for the status message (Isaac: cams, view, tcp_offset)."""
        return {}
    def hold(self) -> None:
        """Stop servoing and hold the current position (safe idle)."""
    def ok(self) -> tuple[bool, str]:
        """(healthy, reason)."""
        return True, ""
    def begin_cycle(self) -> float:
        import time
        return time.perf_counter()
    def end_cycle(self, t0: float, dt: float) -> None:
        import time
        rem = dt - (time.perf_counter() - t0)
        if rem > 0:
            time.sleep(rem)
