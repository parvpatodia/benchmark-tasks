"""stiffness.py: each arm's TCP stiffness (N/m) along x, y and z at its board's grip point at the seat, from
yam_arena's URDF IK: 1 / (d . J K^-1 J^T . d), J the TCP position's Jacobian, K the joints' drive stiffnesses. No sim.
Run with yam_arena on PYTHONPATH."""
import pathlib
import sys
import warnings

import numpy as np

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import SHOULDER, WRIST  # noqa: E402
from yam_arena.demo import grip_rotation  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

M = load(HERE / "dowel_joint.py").module


def tcp_stiffness(arm, pos, yaw, d):
    base = station.BASES[arm]
    q, err, _ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(grip_rotation(yaw)), None, base_pos=base,
                         iters=300)
    p0, _ = ik.tcp_pose(q, base)
    jac = np.zeros((3, 6))
    for i in range(6):
        dq = q.copy()
        dq[i] += 1e-6
        jac[:, i] = (ik.tcp_pose(dq, base)[0] - p0) / 1e-6
    kinv = np.diag([1 / SHOULDER["stiffness"]] * 3 + [1 / WRIST["stiffness"]] * 3)
    d = np.asarray(d, float)
    return float(1.0 / (d @ jac @ kinv @ jac.T @ d)), err


if __name__ == "__main__":
    seats = M.seat_centres()
    for arm, name, yaw in (("left", "hole_board", M.YAW_L), ("right", "dowel_board", M.YAW_R)):
        g = M.grip_point(name, *seats[name], 0.0)
        out = [tcp_stiffness(arm, (g[0], g[1], M.Z_GRIP), yaw, d)[0] for d in np.eye(3)]
        print(f"{arm}: grip ({g[0]:.3f}, {g[1]:.3f}) k_x {out[0]:.0f} k_y {out[1]:.0f} k_z {out[2]:.0f} N/m")
