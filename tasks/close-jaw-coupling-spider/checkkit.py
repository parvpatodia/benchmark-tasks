# robocurve-kit checkkit.py c8e2d5295692
"""checkkit.py: offline gates for a yam_arena task's demo plan. NumPy and SciPy, no simulator (runs on the Mac).

    PYTHONPATH=<yam_arena source> python3 checkkit.py task.py [--modes demo,one_arm] [--seeds 0,1] [-v]
    PYTHONPATH=<yam_arena source> python3 checkkit.py reach 0.30 -0.10 0.06     which grips reach x, y, height

One PASS/FAIL line per gate; the exit code is the number of FAILs. Gates, for every mode and seed:
  IK         the IK error along each arm's plan at 20 Hz (yam_arena's own ArmPlan path). Contact stretches (motionkit
             CONTACT, or every label outside the task's FREE_AIR when it lists one) within IK_CONTACT, the rest within
             IK_FREE; and each joint IK_MARGIN_DEG from its limits once the first waypoint is done.
  CLEARANCE  every collider link of both arms (base, link1 to link5, gripper, both tips: URDF meshes sampled every
             --step) against every part (fixed objects, mechanisms at rest, loose bodies at their start pose or where
             a motionkit pick carried them) and the table top. A part picked by name with motionkit rides with the
             TCP from its close to its let-go, and is then checked against the other parts. Floors: PART_FLOOR to a
             part, TABLE_FLOOR to the table (base and link1 are mounted on it and skipped), PART_OVERLAP for a carried
             part against another part.
  ARMGAP     the least distance between the two arms, all links, against ARM_FLOOR.
  RUNLIMIT   plan + yam_scripted's 3 s hold fits the run limit (task.json run_limit or RUN_LIMIT_STEPS) at 20 Hz.

What the task file may define (all optional): plan(seed, mode) (else demo()), MODES, FREE_AIR, CHECK_SKIP
{(arm, part): (label or "label prefix:", ...)} for touches by design, start_poses(seed) {name: ((x, y, z), yaw)},
check_parts(seed, mode) [(name, world points whose convex hull is the part)], and any floor below as a constant.
Planned poses: the arm's lag and sag are not modelled (fastrun.py measures them). Seeds are CLI seeds, turned into
the trial seed the harness passes (crc32 of "<seed>:None:0", as inspect_robots.rollout.derive_seed).
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import re
import sys
import warnings
import zlib

import numpy as np

warnings.simplefilter("ignore", RuntimeWarning)
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import motionkit as mk  # noqa: E402
from scipy.spatial import ConvexHull, cKDTree  # noqa: E402
from yam_arena import ik, station, task_file  # noqa: E402
from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint  # noqa: E402

HZ = 20
HOLD_S = 3.0               # yam_scripted holds the last pose this long before it stops the trial
IK_CONTACT = (0.5, 0.5)    # mm, deg [from our ikcheck.py tools]
IK_FREE = (3.0, 2.0)
IK_MARGIN_DEG = 3.0
PART_FLOOR = 0.003         # m, any arm link to a part
TABLE_FLOOR = 0.001        # m, any arm link but base and link1 to the table top
PART_OVERLAP = -0.001      # m, a carried part may sit this far into another (resting contact reads about 0)
ARM_FLOOR = 0.010          # m, arm to arm [the lancet's grippers.py floor: over the 2 to 10 mm lag]
NOT_ON_TABLE = ("base", "link1")


def trial_seed(cli_seed: int) -> int:
    return zlib.crc32(f"{cli_seed}:None:0".encode()) & 0xFFFFFFFF


def _rz(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# -- parts as convex hulls ---------------------------------------------------------------------------------------------

def shape_points(shape) -> np.ndarray:
    """Points whose convex hull is a mechanism Shape (its own frame): box corners, cylinder or capsule end rings (with
    capsule caps), or a hull's own points [from the lancet's grippers.py]."""
    if shape.kind == "hull":
        return np.asarray(shape.points, float)
    lo, hi = (np.asarray(v, float) for v in shape.bounds())
    if shape.kind == "box":
        return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    r, length = shape.size
    ax = "XYZ".index(shape.axis)
    o = [i for i in range(3) if i != ax]
    c = (lo + hi) / 2
    rings = ((0.0, r),) + (((0.38, r * 0.92), (0.7, r * 0.71), (0.92, r * 0.38), (1.0, 0.0)) if shape.kind == "capsule" else ())
    out = []
    for e in (-1, 1):
        for th in np.linspace(0, 2 * math.pi, 24, endpoint=False):
            for k, rr in rings:
                p = np.array(c, float)
                p[ax] += e * (length / 2 + k * r * (shape.kind == "capsule"))
                p[o[0]] += rr * math.cos(th)
                p[o[1]] += rr * math.sin(th)
                out.append(p)
    return np.array(out)


class Part:
    """A part as convex hulls (local frame), posed in the world by (R, p)."""

    def __init__(self, name: str, hulls: list, free: bool = False, step: float = 0.005):
        self.name, self.free, self.step, self._tree = name, free, step, None
        self.eq, self.surf = [], []
        for pts in hulls:
            h = ConvexHull(np.asarray(pts, float), qhull_options="QJ")
            self.eq.append(h.equations)                    # n . x + off <= 0 inside, n outward
            self.surf.append(mk._sample(h.points[h.simplices], step))
        self.R, self.p = np.eye(3), np.zeros(3)

    def pose(self, R, p) -> "Part":
        self.R, self.p = np.asarray(R, float), np.asarray(p, float)
        return self

    def surface(self) -> np.ndarray:
        return np.vstack(self.surf) @ self.R.T + self.p

    def signed(self, pts: np.ndarray) -> np.ndarray:
        """Per point: inside a hull, minus its depth (the max face-plane distance); outside, a lower bound on the
        distance to the part, the larger of the plane bound (low near edges) and the distance to the nearest
        surface sample less 0.71 of the sample step (the farthest a face point can be from its nearest sample)."""
        local = (pts - self.p) @ self.R
        best = np.full(len(pts), np.inf)
        for eq in self.eq:
            best = np.minimum(best, (local @ eq[:, :3].T + eq[:, 3]).max(axis=1))
        out = best > 0
        if out.any():
            if self._tree is None:
                self._tree = cKDTree(np.vstack(self.surf))
            d = self._tree.query(local[out])[0] - 0.71 * self.step
            best[out] = np.maximum(best[out], d)
        return best


def parts_from_objects(module, seed) -> list[Part]:
    """Parts from the task's OBJECTS: fixed Box/Cylinder/Sphere, every Mechanism link at rest, loose Bodies and free
    primitives at their spawn pose (or start_poses(seed)). Library Assets have no offline geometry and are skipped."""
    starts = module.start_poses(seed) if hasattr(module, "start_poses") else {}
    out = []
    for obj in getattr(module, "OBJECTS", ()):
        name = obj.name
        if hasattr(obj, "all_links"):        # mechanism.Mechanism
            hulls = [shape_points(s) for link in obj.all_links() for s in link.shapes if s.collide]
            pos, yaw, free = obj.pos, obj.yaw, bool(getattr(obj, "free", False))
        elif hasattr(obj, "corners") and hasattr(obj, "shapes"):   # mechanism.Body
            hulls = [shape_points(s) for s in obj.shapes if s.collide]
            pos, yaw, free = obj.pos, obj.yaw, True
        elif isinstance(obj, task_file.Box):
            sx, sy, sz = obj.size
            hulls = [np.array([[x, y, z] for x in (-sx / 2, sx / 2) for y in (-sy / 2, sy / 2) for z in (-sz / 2, sz / 2)])]
            pos, yaw, free = obj.pos, obj.yaw, obj.free
        elif isinstance(obj, task_file.Cylinder):
            th = np.linspace(0, 2 * math.pi, 24, endpoint=False)
            ring = np.c_[obj.radius * np.cos(th), obj.radius * np.sin(th)]
            hulls = [np.vstack([np.c_[ring, np.full(24, z)] for z in (-obj.height / 2, obj.height / 2)])]
            pos, yaw, free = obj.pos, obj.yaw, obj.free
        elif isinstance(obj, task_file.Sphere):
            g = np.array([(x, y, z) for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1) if (x, y, z) != (0, 0, 0)], float)
            hulls = [obj.radius * g / np.linalg.norm(g, axis=1)[:, None]]
            pos, yaw, free = obj.pos, obj.yaw, obj.free
        else:
            continue
        if name in starts:
            pos, yaw = starts[name]
        if hulls:
            out.append(Part(name, hulls, free).pose(_rz(float(yaw)), np.asarray(pos, float)))
    return out


# -- sweeping a plan ---------------------------------------------------------------------------------------------------

def _matches(label: str, entries) -> bool:
    return any(label == e or (e.endswith(":") and label.startswith(e)) or label.startswith(e + ":") for e in entries)


class Worst:
    def __init__(self):
        self.v: dict = {}

    def add(self, key, d: float, where: str):
        if key not in self.v or d < self.v[key][0]:
            self.v[key] = (float(d), where)


def bases_for(module) -> dict:
    if getattr(module, "SCENE", "station") == "station":
        return dict(station.BASES)
    return {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}


def sweep(module, plans: dict, name: str, seed, cfg: dict, step: float, hz: int, worst: dict, verbose: bool):
    """One plan: fill `worst` (IK, clearance, armgap) with each key's least margin and where."""
    bases = bases_for(module)
    top = task_file.TABLE_TOP[getattr(module, "SCENE", "station")]
    A = {arm: ArmPlan(list(W), bases[arm]) for arm, W in plans.items()}
    T = max((a.duration for a in A.values()), default=0.0)
    if hasattr(module, "check_parts"):
        parts = [Part(n, [p]) for n, p in module.check_parts(seed, name.split(" ")[0])]
    else:
        parts = parts_from_objects(module, seed)
    by_name = {p.name: p for p in parts}
    free_air = getattr(module, "FREE_AIR", None)
    skip = getattr(module, "CHECK_SKIP", {})
    first_end = {arm: (a.waypoints[0].duration if a.waypoints else 0.0) for arm, a in A.items()}
    held: dict = {}                      # arm -> (part, R_rel, p_rel)
    letting: dict = {}                   # arm -> the part this arm is letting go of, while its release label plays
    every = max(1, round(HZ / hz))
    for k in range(int(T * HZ) + 2):
        t = min(k / HZ, T)
        state = {}
        for arm, a in A.items():
            pos, rot, finger, label = a.target(min(t, a.duration))
            q, pe, re_ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), a.q, base_pos=a.base_pos, iters=60)
            a.q = q
            contact = label in mk.CONTACT.get(arm, ()) or (free_air is not None and label not in free_air)
            kind = "contact" if contact else "free"
            lim = IK_CONTACT if contact else IK_FREE
            where = f"{name} {arm} t {t:.2f} '{label}'"
            worst["ik"].add((kind, "mm"), lim[0] - pe * 1000, f"{pe * 1000:.2f} mm {where}")
            worst["ik"].add((kind, "deg"), lim[1] - math.degrees(re_), f"{math.degrees(re_):.2f} deg {where}")
            if t > first_end[arm]:
                m = float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q)).min())
                worst["ik"].add(("margin", "deg"), m - IK_MARGIN_DEG, f"{m:.1f} deg {where}")
            if label in mk.GRASP.get(arm, {}) and arm not in held and mk.GRASP[arm][label] in by_name:
                part = by_name[mk.GRASP[arm][label]]
                held[arm] = (part, rot.T @ part.R, rot.T @ (part.p - pos))
            if label in mk.RELEASE.get(arm, set()):
                if arm in held:
                    letting[arm] = held.pop(arm)[0]
            else:
                letting.pop(arm, None)
            if arm in held:
                part, rr, pr = held[arm]
                part.pose(rot @ rr, pos + rot @ pr)
            state[arm] = (q, finger, label, pos, rot)
        if k % every:
            continue
        pts = {}
        for arm, (q, finger, label, _, _) in state.items():
            P, names = mk.arm_points(arm, q, finger, step=step, base=bases[arm])
            pts[arm] = (P, names)
            where = f"{name} {arm} t {t:.2f} '{label}'"
            hand = np.isin(names, mk.HAND_LINKS)
            gripping = {mk.GRASP[arm][label]} if label in mk.GRASP.get(arm, {}) else set()
            for part in parts:
                if (arm in held and held[arm][0] is part) or part.name in gripping or letting.get(arm) is part:
                    continue
                lo, hi = part.surface().min(0) - 0.05, part.surface().max(0) + 0.05
                near = np.all((P >= lo) & (P <= hi), axis=1)
                if _matches(label, skip.get((arm, part.name), ())):
                    near &= ~hand
                if near.any():
                    s = part.signed(P[near])
                    i = int(np.argmin(s))
                    worst["clear"].add((arm, part.name), s[i] - cfg["PART_FLOOR"],
                                       f"{1000 * s[i]:.1f} mm {arm} {names[near][i]} vs {part.name} {where}")
            on = ~np.isin(names, NOT_ON_TABLE)
            z = P[on, 2] - top
            i = int(np.argmin(z))
            worst["clear"].add((arm, "table"), z[i] - cfg["TABLE_FLOOR"], f"{1000 * z[i]:.1f} mm {arm} {names[on][i]} vs table {where}")
            if arm in held:
                part = held[arm][0]
                S = part.surface()
                worst["clear"].add((part.name, "table"), S[:, 2].min() - top - cfg["PART_OVERLAP"],
                                   f"{1000 * (S[:, 2].min() - top):.1f} mm carried {part.name} vs table {where}")
                for other in parts:
                    if other is part:
                        continue
                    s = other.signed(S)
                    i = int(np.argmin(s))
                    worst["clear"].add((part.name, other.name), s[i] - cfg["PART_OVERLAP"],
                                       f"{1000 * s[i]:.1f} mm carried {part.name} vs {other.name} {where}")
        if len(pts) == 2:
            (a, na), (b, nb) = pts["left"], pts["right"]
            pad = cfg["ARM_FLOOR"] + 0.03
            ka = np.all((a >= b.min(0) - pad) & (a <= b.max(0) + pad), axis=1)
            kb = np.all((b >= a.min(0) - pad) & (b <= a.max(0) + pad), axis=1)
            if ka.any() and kb.any():
                d, i = cKDTree(a[ka]).query(b[kb])
                j = int(np.argmin(d))
                worst["arm"].add("arms", d[j] - cfg["ARM_FLOOR"],
                                 f"{1000 * d[j]:.1f} mm left {na[ka][i[j]]} vs right {nb[kb][j]} {name} t {t:.2f} "
                                 f"'{state['left'][2]}' / '{state['right'][2]}'")
            else:   # WHY: no point of either arm within `pad` of the other's box, so they are at least that far apart
                worst["arm"].add("arms", pad - cfg["ARM_FLOOR"], f"over {1000 * pad:.0f} mm apart {name} t {t:.2f}")
    if verbose:
        print(f"  swept {name}: {T:.1f} s")


def run_limit(module, path: pathlib.Path) -> int | None:
    if hasattr(module, "RUN_LIMIT_STEPS"):
        return int(module.RUN_LIMIT_STEPS)
    tj = path.parent / "task.json"
    if tj.exists():
        found = re.search(r"[\d,]+", str(json.loads(tj.read_text()).get("run_limit", "")))
        if found:
            return int(found.group().replace(",", ""))
    return None


def plans_for(module, seed, mode):
    if hasattr(module, "plan"):
        return module.plan(seed, mode)
    if mode not in (None, "demo"):
        raise SystemExit("the task has no plan(seed, mode), only demo(): use --modes demo")
    return module.demo()


def check(path: str, modes=None, seeds=(0,), step: float = 0.004, hz: int = 10, verbose: bool = False,
          raw_seeds: bool = False) -> list[tuple[str, bool, str]]:
    """Every gate's (name, passed, detail)."""
    path = pathlib.Path(path).resolve()
    module = task_file.load(path).module
    cfg = {k: getattr(module, k, globals()[k]) for k in ("PART_FLOOR", "TABLE_FLOOR", "PART_OVERLAP", "ARM_FLOOR")}
    modes = list(modes or getattr(module, "MODES", None) or ["demo"])
    worst = {"ik": Worst(), "clear": Worst(), "arm": Worst()}
    durations = {}
    for mode in modes:
        for s in seeds:
            seed = s if raw_seeds else trial_seed(s)
            plans = plans_for(module, seed, mode)
            name = f"{mode} s{s}"
            durations[name] = max((sum(w.duration for w in W) for W in plans.values()), default=0.0)
            sweep(module, plans, name, seed, cfg, step, hz, worst, verbose)
    out = []
    for gate, key in (("IK", "ik"), ("CLEARANCE", "clear"), ("ARMGAP", "arm")):
        v = worst[key].v
        if not v:
            out.append((gate, True, "nothing to check (one arm or no plan)"))
            continue
        bad = {k: x for k, x in v.items() if x[0] < 0}
        lowest = min(v.items(), key=lambda kv: kv[1][0])
        detail = f"{len(bad)} under floor; least margin at {lowest[1][1]}"
        if verbose:
            detail += "".join(f"\n    {k}: {x[1]}" for k, x in sorted(v.items(), key=lambda kv: kv[1][0]))
        out.append((gate, not bad, detail))
    limit = run_limit(module, path)
    if limit is None:
        out.append(("RUNLIMIT", False, "no run limit: set run_limit in task.json or RUN_LIMIT_STEPS"))
    else:
        need = {n: math.ceil((d + HOLD_S) * HZ) for n, d in durations.items()}
        n, most = max(need.items(), key=lambda kv: kv[1])
        out.append(("RUNLIMIT", most <= limit, f"longest {n}: {durations[n]:.1f} s + {HOLD_S:.0f} s hold = {most} steps of {limit}"))
    return out


def reach(x: float, y: float, height: float) -> None:
    """Which grips each arm holds at x, y, height over the table: IK from READY within 2 mm and 2 deg. No collisions."""
    def holds(side, grip):
        p = ArmPlan([Waypoint(2.0, (x, y, mk.TOP + height), mk.FINGER_OPEN, "go", grip)], station.BASES[side])
        for t in (0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0):
            p.worst_ik_mm, p.worst_ik_deg = 0.0, 0.0
            p.action7(t)
        return p.worst_ik_mm < 2.0 and p.worst_ik_deg < 2.0

    phis = list(range(-90, 181, 30))
    print(f"TCP at x {x}, y {y}, {height} over the table. Angles that solve:")
    for side in mk.ARMS:
        print(f"  {side}: down, yaw " + " ".join(str(a) for a in range(-120, 121, 30) if holds(side, mk.down(a))))
        for th in (45, 60, 90):
            print(f"  {side}: tilt {th}, phi " + " ".join(str(p) for p in phis if holds(side, mk.tilted(th, p)))
                  + "  flipped " + " ".join(str(p) for p in phis if holds(side, mk.tilted(th, p, True))))


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == "reach":
        x, y, h = (float(v) for v in argv[1:4])
        reach(x, y, h)
        return 0
    ap = argparse.ArgumentParser(description="Offline gates for a task's demo plan; see the docstring.")
    ap.add_argument("task")
    ap.add_argument("--modes", help="comma list; default the task's MODES, else demo")
    ap.add_argument("--seeds", default="0", help="CLI seeds, comma list (default 0)")
    ap.add_argument("--raw-seeds", action="store_true", help="pass the seeds to plan() as given, not as trial seeds")
    ap.add_argument("--step", type=float, default=0.004, help="m between sampled mesh points (default 0.004)")
    ap.add_argument("--hz", type=int, default=10, help="clearance checks per second (default 10)")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    res = check(a.task, a.modes.split(",") if a.modes else None, [int(s) for s in a.seeds.split(",")], a.step, a.hz,
                a.verbose, a.raw_seeds)
    for gate, ok, detail in res:
        print(f"{gate:10s} {'PASS' if ok else 'FAIL'}  {detail}", flush=True)
    return sum(not ok for _, ok, _ in res)


if __name__ == "__main__":
    sys.exit(main())
