"""sagcheck.py: where the right hand ends up under the bag, with and without the demo's load compensation. At 5 Hz
along every loaded stretch of the right plan (both ends of the segment in mower_bag.LOADED), it solves IK for the
commanded pose (mower_bag.commanded), then the arm's static balance under the bag, q = q_cmd + K^-1 J(q)^T w(q) (fixed
point, the nonlinear kinematics), and compares that TCP with the planned one. Same for the uncompensated plan. Prints
the worst errors per run; BAD when the compensated TCP is off by more than 2 mm or 1 deg, or a joint torque from the
bag passes 50% of its effort limit. Ends with SAG BAD n. Joint springs only: no friction, no contact, no sim."""
import math
import pathlib
import sys
import warnings

import numpy as np

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import SHOULDER, WRIST  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

M = load(HERE / "mower_bag.py").module
LIMIT = (0.002, math.radians(1.0))
EFFORT = np.array([SHOULDER["effort"]] * 3 + [WRIST["effort"]] * 3)


def settle(q_cmd, base):
    """Joint angles where the springs balance the bag, from the commanded ones (fixed point)."""
    q = np.array(q_cmd, float)
    for _ in range(30):
        _, R = ik.tcp_pose(q, base)
        q_new = q_cmd + M.sag_dq(q, base, R)
        if np.max(np.abs(q_new - q)) < 1e-7:
            return q_new
        q = q_new
    return q


def rot_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def check(name, plans):
    if "right" not in plans:
        return 0
    base = np.asarray(station.BASES["right"], float)
    want, cmd = ArmPlan(plans["right"], base), ArmPlan(M.commanded(plans)["right"], base)
    labels = [w.label for w in plans["right"]]
    worst = {"raw": [0.0, 0.0, 0.0], "fixed": [0.0, 0.0, 0.0]}
    torque, t, n = 0.0, 0.0, 0
    while t <= want.duration:
        p, R, _, label = want.target(t)
        k = labels.index(label) if label in labels else -1
        if label in M.LOADED and k > 0 and labels[k - 1] in M.LOADED:
            pc, Rc, _, _ = cmd.target(t)
            qw, _, _ = ik.solve(p, ik.matrix_to_quat_xyzw(R), want.q, base_pos=base, iters=200)
            qc, _, _ = ik.solve(pc, ik.matrix_to_quat_xyzw(Rc), cmd.q, base_pos=base, iters=200)
            want.q, cmd.q = qw, qc
            for key, q0 in (("raw", qw), ("fixed", qc)):
                pa, Ra = ik.tcp_pose(settle(q0, base), base)
                e = pa - p
                worst[key][0] = max(worst[key][0], float(np.linalg.norm(e)))
                worst[key][1] = max(worst[key][1], rot_deg(R, Ra))
                worst[key][2] = min(worst[key][2], float(e[2]))
            tau = M.tcp_jacobian(qw, base).T @ M.bag_wrench(R)
            torque = max(torque, float(np.max(np.abs(tau) / EFFORT)))
            n += 1
        t += 0.2
    bad = int(worst["fixed"][0] > LIMIT[0] or math.radians(worst["fixed"][1]) > LIMIT[1] or torque > 0.5)
    print(f"{name:16s} {n:4d} loaded samples  uncompensated: off {1000 * worst['raw'][0]:5.1f} mm "
          f"(z {1000 * worst['raw'][2]:6.1f}) {worst['raw'][1]:4.1f} deg   compensated: off "
          f"{1000 * worst['fixed'][0]:4.2f} mm {worst['fixed'][1]:4.2f} deg   torque {100 * torque:3.0f}% of effort"
          f"{'  BAD' if bad else ''}")
    return bad


if __name__ == "__main__":
    total = 0
    for s in range(10):
        total += check(f"demo seed {s}", M.plan(s, "demo"))
    total += check("demo run seed", M.plan(1415158438, "demo"))
    for v in ("one_arm", "lift", "one_hook", "tips", "ground", "hold", "prop", "pinch", "slam", "nudge", "shove"):
        total += check(f"s_{v}", M.plan(0, v))
    print("SAG BAD", total)
