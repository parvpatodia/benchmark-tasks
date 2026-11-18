"""ikcheck.py: static IK error of every waypoint and the interpolated worst for both arms, for the demo on each seed's
layout and for every stem copy in sim/arm/ (made by sim/mk.sh). Ends with TOTAL BAD n. No sim."""
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

LIMIT_MM, LIMIT_DEG = 0.5, 0.5


def check(name, plans):
    bad = 0
    for arm, W in plans.items():
        p = ArmPlan(W, station.BASES[arm])
        t = 0.0
        while t <= p.duration:
            p.action7(t)
            t += 0.05
        top = max(w.pos[2] for w in W) - station.TABLE_TOP
        flag = p.worst_ik_mm > LIMIT_MM or p.worst_ik_deg > LIMIT_DEG
        bad += int(flag)
        print(f"{name:14s} {arm:5s} {p.duration:5.1f} s worst {p.worst_ik_mm:6.2f} mm {p.worst_ik_deg:5.2f} deg "
              f"top TOP+{top:.3f}{' BAD' if flag else ''}")
    return bad


if __name__ == "__main__":
    total = 0
    M = load(HERE / "dowel_joint.py").module
    for s in range(len(M.LAYOUTS)):
        total += check(f"demo seed {s}", M.plan(s))
    for f in sorted((HERE / "sim" / "arm").glob("[ps]_*.py")):
        m = load(f).module
        if m.VARIANT in ("left_only",):   # this stem is meant to run out of reach
            print(f"{f.stem:14s} (out of reach by design)")
            continue
        total += check(f.stem, m.plan(0))
    print("TOTAL BAD", total)
