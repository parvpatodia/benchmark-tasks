"""ikcheck.py [seed=N]: per-waypoint static IK error of the right arm's demo plan (each pose alone), and the plan's
interpolated worst, for one layout. No sim."""
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import mug_handle as F  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint  # noqa: E402

seed = None
for a in sys.argv[1:]:
    k, v = a.split("=")
    if k == "seed":
        seed = int(v)
R, info = F.plan(0.0, seed)
bad = []
for w in R:
    p = ArmPlan([Waypoint(1.0, w.pos, w.finger, "a", w.yaw), Waypoint(0.5, w.pos, w.finger, "b", w.yaw)],
                station.BASES["right"])
    t = 0.0
    while t <= p.duration:
        p.action7(t)
        t += 0.1
    if p.worst_ik_mm > 0.5 or p.worst_ik_deg > 0.5:
        bad.append(f"  {w.label:45s} {p.worst_ik_mm:6.2f} mm {p.worst_ik_deg:6.2f} deg  pos "
                   f"{tuple(round(v, 4) for v in w.pos)} yaw {w.yaw * 57.3:.0f}")
print("right", len(R), "waypoints;", len(bad), "bad")
print("\n".join(bad[:30]))
p = ArmPlan(R, station.BASES["right"])
segs, t0 = [], 0.0
for w in R:
    segs.append((t0, t0 + w.duration, w.label))
    t0 += w.duration
t, prev, hits = 0.0, (0.0, 0.0), {}
while t <= p.duration:
    p.action7(t)
    cur = (p.worst_ik_mm, p.worst_ik_deg)
    if cur[0] > prev[0] + 0.05 or cur[1] > prev[1] + 0.05:
        hits[next(lab for a, b, lab in segs if a <= t <= b)] = cur
    prev = cur
    t += 0.02
print(f"interpolated: dur {p.duration:.1f} s worst {p.worst_ik_mm:.2f} mm {p.worst_ik_deg:.2f} deg",
      {k: (round(v[0], 2), round(v[1], 2)) for k, v in hits.items()})
