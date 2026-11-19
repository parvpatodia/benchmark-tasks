"""clearance.py: sweeps both grippers (finger slices and housing, as points) and the carried bag along the planned TCP
paths at 10 Hz, against every part: the mower's colliders, its flap (at the angle the plan gives it) and the bag (at
its start, carried rigidly by the right hand while gripped, then hung). Prints each arm's least clearance to each
part, and the bag's to the flap, with when and where. Parts a hand is meant to hold (the bag's bar while gripped,
the flap while pinched) are left out for that hand, and so is the bag against the hooks from the lowering into the
hooks on. Ends with CLEARANCE BAD n (any other clearance under 3 mm). Planned poses, no IK, no sim.

Gripper model (mm over the pad tips, along the approach axis): finger slices (from, to, thickness outward of the pad
face, half width along the pad axis) and the housing beyond 83.5 mm, 51 x 33 mm half sizes [EST from the YAM gripper
mesh, as other tasks in this repository measured it]."""
import math
import pathlib
import sys
import warnings

import numpy as np

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

M = load(HERE / "mower_bag.py").module
SLICES = [(0, 10, 5.4, 11), (10, 20, 7.1, 14.8), (20, 40, 12.6, 22.6), (40, 60, 20.2, 30.4), (60, 83.5, 26, 36)]
HOUSING = (83.5, 200.0, 51.0, 33.0)
TIP = M.PAD_TIP - M.TCP_OFFSET[2]      # pad tips past the TCP (m)
LIMIT = 0.003
PINCHED = {"pinch the flap", "raise the flap", "hold the flap up", "lower the flap", "keep holding",
           "let go of the flap"}   # WHY let go: the fingers open from round the plate
GRIPPED = {"grip the bar", "lift off the bench", "wait for the flap", "slide in, square", "lift over the lips",
           "lift to the hooks", "push in to the slots", "lower", "carry over the slots", "lower into the hooks",
           "settle in the hooks", "keep holding", "set the tail down", M.UNLOAD_LABEL, "line up the push", "roll the mower", "settle after the push",
           *M.SERVO_LABELS}


def gripper_points(finger):
    """Points in the TCP frame (x closing, y pad axis, z approach)."""
    f = abs(finger)
    pts = []
    for h0, h1, out, hw in SLICES:
        for h in np.linspace(h0, h1, 3) / 1000:
            for s in (-1.0, 1.0):
                for dx in np.linspace(0.0, out / 1000, 3):
                    for y in np.linspace(-hw / 1000, hw / 1000, 5):
                        pts.append((s * (f + dx), y, TIP - h))
    h0, h1, hx, hy = HOUSING
    for h in np.linspace(h0, h1, 7) / 1000:
        for x in np.linspace(-hx / 1000, hx / 1000, 7):
            for y in np.linspace(-hy / 1000, hy / 1000, 5):
                pts.append((x, y, TIP - h))
    return np.array(pts)


def box_dists(pts, boxes):
    """Least distance from each point (rows, in the boxes' frame) to a list of (lo, hi) boxes."""
    best = np.full(len(pts), np.inf)
    for lo, hi in boxes:
        q = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
        best = np.minimum(best, np.linalg.norm(q, axis=1))
    return best


def boxes(shapes):
    return [s.bounds() for s in shapes if s.collide]


MOWER_BOXES = boxes(M.MOWER.links[0].shapes)
HOOK_BOXES = boxes(M.hook_shapes(1) + M.hook_shapes(-1))
FLAP_BOXES = boxes(M.MOWER.links[1].shapes)
BAG_BOXES = boxes(M.BAG.shapes)
BAG_PTS = np.array([(x, y, z) for x in np.linspace(M.FRONT - M.BAG_L, 0.005, 12) for y in np.linspace(-M.BAG_W / 2,
                    M.BAG_W / 2, 7) for z in np.linspace(M.BOT_Z, M.BAR_Z + M.BAR_R, 6)
                    if any(np.all((lo - 1e-9 <= (x, y, z)) & ((x, y, z) <= hi + 1e-9)) for lo, hi in BAG_BOXES)])


def flap_angle(left_target):
    """The flap's angle from the left TCP while it pinches the plate (the pinch point PINCH_R out from the hinge)."""
    hx, hz = M.HINGE
    p = left_target - np.array([M.X_M, 0.0, M.TOP])
    return math.atan2(-(p[0] - hx), -(p[2] - hz))


def unloaded(W):
    """The right plan as the demo's unload pass leaves it: after the bag settles in the hooks the hand is aimed where it
    is, SEAT_PUSH over its commanded point (the hooks stop the bag; the plan pressed it SEAT_PUSH lower)."""
    out, after = [], False
    for w in W:
        if after:
            w = M.dataclasses.replace(w, pos=(w.pos[0], w.pos[1], w.pos[2] + M.SEAT_PUSH))
        out.append(w)
        after = after or w.label == M.UNLOAD_LABEL
    return out


def sweep(name, plans, seed):
    plans = {arm: (unloaded(W) if arm == "right" else W) for arm, W in plans.items()}
    A = {arm: ArmPlan(W, station.BASES[arm]) for arm, W in plans.items()}
    T = max((a.duration for a in A.values()), default=0.0) + 1.0
    x0, y0, yaw0 = M.bag_layout(seed)
    bag_R, bag_p = M._rz(yaw0), np.array([x0, y0, M.Z_LYING])
    hp, hq, _, _ = M.hung_pose()
    grip_rel = None
    theta, pinch_seen, low, bad = 0.0, False, {}, 0
    hooks_ok_from = None
    t = 0.0
    while t <= T:
        tg = {arm: a.target(min(t, a.duration)) for arm, a in A.items()}
        released = False
        if "left" in tg:
            label = tg["left"][3] if t < A["left"].duration else "done"
            if name == "s_prop" and label == "done":
                label = "keep holding"
            if label in PINCHED:
                theta, pinch_seen = flap_angle(tg["left"][0]), True
            elif pinch_seen:
                theta, released = {"s_pinch": M.UNDER, "s_tips": None, "s_hold": None}.get(name, M.REST), True
                if theta is None:       # the flap lands on a bag held higher than hung: not tracked
                    theta = M.REST
        if "right" in tg:
            pos, R, f, label = tg["right"]
            if t >= A["right"].duration:
                label = "done"
            if label in GRIPPED:
                if grip_rel is None:
                    grip_rel = (R.T @ bag_R, R.T @ (bag_p - pos))
                bag_R, bag_p = R @ grip_rel[0], pos + R @ grip_rel[1]
                if label in ("lower into the hooks", "lower") and hooks_ok_from is None:
                    hooks_ok_from = t
            elif grip_rel is not None:
                if name in ("s_ground", "s_one_arm"):     # let go short of the hooks: it drops, not tracked further
                    bag_R = None
                else:
                    bag_R, bag_p = M._rot3(hq), hp + np.array([{"s_nudge": M.NUDGE, "s_shove": M.SHOVE}.get(name, 0.0),
                                                     M.ONE_HOOK_DY if name == "s_one_hook" else 0.0, 0.0])
                grip_rel = None
        Rf = M._ry(theta)
        h = np.array([M.HINGE[0], 0.0, M.HINGE[1]])
        pm = np.array([M.X_M, 0.0, M.TOP])
        for arm, (pos, R, f, label) in tg.items():
            if t >= A[arm].duration:
                label = "keep holding" if (name == "s_prop" and arm == "left") else "done"
            P = pos + gripper_points(f) @ R.T
            parts = {"mower": box_dists(P - pm, MOWER_BOXES)}
            # WHY draw out: let go, the flap falls 30 deg onto the lower finger and stays on it until the hand slides out
            if not (arm == "left" and label in PINCHED | {"draw out sideways"}) and not (released and name == "s_pinch" and arm == "right"):
                parts["flap"] = box_dists((P - pm - h) @ Rf + h, FLAP_BOXES)   # world -> flap frame (rows: R^T p)
            if label == "done" and arm == "right" and name in ("s_hold", "s_tips"):
                label = "keep holding"
            if bag_R is not None and not (arm == "right" and label in GRIPPED | {"open round the bar"}):
                parts["bag"] = box_dists((P - bag_p) @ bag_R, BAG_BOXES)
            for part, d in parts.items():
                k = int(np.argmin(d))
                key = (arm, part)
                if key not in low or d[k] < low[key][0]:
                    low[key] = (float(d[k]), round(t, 1), label, (P[k] - pm).round(3))
        if bag_R is not None and not released:   # WHY: once let go the flap is meant to rest on the bag
            B = bag_p + BAG_PTS @ bag_R.T
            d = box_dists((B - pm - h) @ Rf + h, FLAP_BOXES)
            k = int(np.argmin(d))
            if ("bag", "flap") not in low or d[k] < low[("bag", "flap")][0]:
                low[("bag", "flap")] = (float(d[k]), round(t, 1), "", (B[k] - pm).round(3))
            if hooks_ok_from is None or t < hooks_ok_from:
                d = box_dists(B - pm, HOOK_BOXES)
                k = int(np.argmin(d))
                if ("bag", "hooks") not in low or d[k] < low[("bag", "hooks")][0]:
                    low[("bag", "hooks")] = (float(d[k]), round(t, 1), "", (B[k] - pm).round(3))
        t += 0.1
    for key in sorted(low):
        d, tt, label, where = low[key]
        flag = d < LIMIT and key not in ((("bag", "flap"),) if name in ("s_one_arm",) else ())
        bad += int(flag)
        print(f"{name:14s} {key[0]:5s} vs {key[1]:5s} {1000 * d:6.1f} mm at t {tt:5.1f} ({label}) M {where}"
              f"{'  BAD' if flag else ''}")
    return bad


if __name__ == "__main__":
    total = 0
    for s in (0, 1, 2):
        total += sweep("demo", M.plan(s, "demo"), s)
    for v in ("one_arm", "one_hook", "tips", "ground", "hold", "prop", "pinch", "slam", "nudge", "shove"):
        total += sweep(f"s_{v}", M.plan(0, v), 0)
    print("CLEARANCE BAD", total)
