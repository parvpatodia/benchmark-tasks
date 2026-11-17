"""ikcheck.py [seed=N]: per-waypoint static IK error of both arms' demo plans (each pose alone) and each plan's
interpolated worst, joint6 against its +-2.0944 rad limit, and the two TCPs' separation along x while both are low.
No sim."""
import math
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np  # noqa: E402
import lancet_fingerstick as F  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint  # noqa: E402

J6 = 2.0944
seed = None
for a in sys.argv[1:]:
    k, v = a.split("=")
    if k == "seed":
        seed = int(v)
plans = dict(zip(("right", "left"), F.plan(0.0, seed)))
total_bad = 0
for side, ws in plans.items():
    bad = []
    for w in ws:
        p = ArmPlan([Waypoint(1.0, w.pos, w.finger, "a", w.yaw), Waypoint(0.5, w.pos, w.finger, "b", w.yaw)],
                    station.BASES[side])
        t, j6 = 0.0, 0.0
        while t <= p.duration:
            q, _ = p.action7(t)
            j6 = max(j6, abs(float(q[5])))
            t += 0.1
        if p.worst_ik_mm > 0.5 or p.worst_ik_deg > 0.5 or j6 > J6:
            bad.append(f"  {w.label:40s} {p.worst_ik_mm:6.2f} mm {p.worst_ik_deg:6.2f} deg j6 {j6:5.2f} pos "
                       f"{tuple(round(v, 4) for v in w.pos)} yaw {math.degrees(w.yaw):.0f}")
    total_bad += len(bad)
    print(side, len(ws), "waypoints;", len(bad), "bad")
    if bad:
        print("\n".join(bad))
    p = ArmPlan(ws, station.BASES[side])
    t, j6 = 0.0, 0.0
    while t <= p.duration:
        q, _ = p.action7(t)
        j6 = max(j6, abs(float(q[5])))
        t += 0.02
    print(f"  interpolated: dur {p.duration:.1f} s worst {p.worst_ik_mm:.2f} mm {p.worst_ik_deg:.2f} deg; max |joint6| "
          f"{j6:.2f} rad (limit {J6})")


def tcp_at(ws, t, start):
    """The TCP target at time t (linear between waypoints, enough for the separation check)."""
    p0, t0 = np.asarray(start, float), 0.0
    for w in ws:
        if t <= t0 + w.duration:
            a = (t - t0) / w.duration
            return p0 + a * (np.asarray(w.pos) - p0)
        p0, t0 = np.asarray(w.pos, float), t0 + w.duration
    return p0


R, L = plans["right"], plans["left"]
dur = max(sum(w.duration for w in R), sum(w.duration for w in L))
worst, t = (9.0, None, None, None), 0.0
while t <= dur:
    r, lf = tcp_at(R, t, (0.323, -0.305, 1.008)), tcp_at(L, t, F.PARK_L)
    # WHY: G11 counts grippers that overlap in y (within 200 mm, two 96 mm wide bodies plus margin)
    if max(r[2], lf[2]) < F.TOP + 0.20 and abs(lf[1] - r[1]) < 0.200:   # both low and side by side in y
        dx = abs(lf[0] - r[0])
        if dx < worst[0]:
            worst = (dx, t, tuple(np.round(r, 3)), tuple(np.round(lf, 3)))
    t += 0.05
print(f"closest x separation of the two TCPs while both low and within 200 mm in y: {1000 * worst[0]:.0f} mm at t {worst[1]:.1f} s "
      f"(right {worst[2]}, left {worst[3]})")
print("TOTAL BAD", total_bad)
