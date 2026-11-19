"""ikcheck.py: IK error over every waypoint, interpolated at 20 Hz, for both arms, for the demo on seeds 0 to 9 and for
every stem (variant) of the task file, on the poses the demo commands (floor_hole_cover.commanded: loaded waypoints
moved against the part's sag). Waypoints the task file lists in FREE_AIR (no part within 7 mm) may err up to 3 mm /
2 deg; every other one 0.5 mm / 0.5 deg. Also prints each arm's smallest joint margin to its limits (BAD under
3 deg). Ends with TOTAL BAD n. No sim."""
import pathlib
import sys
import warnings

import numpy as np

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from yam_arena import ik, station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

LIMIT = (0.5, 0.5)
FREE_LIMIT = (3.0, 2.0)
MARGIN_DEG = 3.0


def check(name, plans, free):
    bad = 0
    for arm, W in plans.items():
        p = ArmPlan(W, station.BASES[arm])
        t, worst, margin = 0.0, {}, 1e9
        while t <= p.duration:
            pos, rot, _, label = p.target(t)
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), p.q, base_pos=p.base_pos, iters=60)
            p.q = q
            w = worst.setdefault(label, [0.0, 0.0])
            w[0], w[1] = max(w[0], pe * 1000), max(w[1], float(np.degrees(re)))
            if t > 3.0:   # WHY: the first move leaves READY, which sits on a joint stop
                margin = min(margin, float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q)).min()))
            t += 0.05
        over = [lab for lab, (mm, deg) in worst.items()
                if mm > (FREE_LIMIT if lab in free else LIMIT)[0] or deg > (FREE_LIMIT if lab in free else LIMIT)[1]]
        if margin < MARGIN_DEG:
            over.append(f"joint margin {margin:.1f} deg")
        hard = max((v[0] for k, v in worst.items() if k not in free), default=0.0)
        soft = max((v[0] for k, v in worst.items() if k in free), default=0.0)
        top = max(w.pos[2] for w in W) - station.TABLE_TOP
        bad += int(bool(over))
        print(f"{name:16s} {arm:5s} {p.duration:5.1f} s contact {hard:5.2f} mm free-air {soft:5.2f} mm "
              f"top TOP+{top:.3f} margin {margin:5.1f} deg{' BAD ' + str(over) if over else ''}")
    return bad


if __name__ == "__main__":
    M = load(HERE / "floor_hole_cover.py").module
    total = 0
    for s in range(10):
        total += check(f"demo seed {s}", M.commanded(M.plan(s, "demo")), M.FREE_AIR)
    for v in M.VARIANTS[2:]:
        total += check(f"s_{v}", M.commanded(M.plan(0, v)), M.FREE_AIR)
    print("TOTAL BAD", total)
