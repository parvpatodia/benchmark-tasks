"""Per-waypoint static IK error of the demo plan (each pose alone), and the plan's interpolated worst."""
import sys, warnings, pathlib
warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import nail_trim as F
from yam_arena import station
from yam_arena.demo import ArmPlan, Waypoint
R, info = F.plan()
for arm, ws in (("right", R),):
    bad = []
    for w in ws:
        p = ArmPlan([Waypoint(1.0, w.pos, w.finger, "a", w.yaw), Waypoint(0.5, w.pos, w.finger, "b", w.yaw)], station.BASES[arm])
        t = 0.0
        while t <= p.duration:
            p.action7(t); t += 0.1
        if p.worst_ik_mm > 0.5 or p.worst_ik_deg > 0.5:
            bad.append(f"  {w.label:45s} {p.worst_ik_mm:6.2f} mm {p.worst_ik_deg:6.2f} deg  pos {tuple(round(v,4) for v in w.pos)} yaw {w.yaw*57.3:.0f}")
    print(arm, len(ws), "waypoints;", len(bad), "bad"); print("\n".join(bad[:30]))

print("interpolated:")
for arm, ws in (("right", R),):
    p = ArmPlan(ws, station.BASES[arm])
    segs, t0 = [], 0.0
    for w in ws:
        segs.append((t0, t0 + w.duration, w.label)); t0 += w.duration
    t, prev, hits = 0.0, (0.0, 0.0), {}
    while t <= p.duration:
        p.action7(t)
        cur = (p.worst_ik_mm, p.worst_ik_deg)
        if cur[0] > prev[0] + 0.05 or cur[1] > prev[1] + 0.05:
            lab = next(l for a, b, l in segs if a <= t <= b)
            hits[lab] = cur
        prev = cur; t += 0.02
    print(arm, f"dur {p.duration:.1f} s worst {p.worst_ik_mm:.2f} mm {p.worst_ik_deg:.2f} deg", {k: (round(v[0], 2), round(v[1], 2)) for k, v in hits.items()})
