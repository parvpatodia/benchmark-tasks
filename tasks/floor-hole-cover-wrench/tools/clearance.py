"""clearance.py: sweeps both grippers (finger slices and housing, as points: floor_hole_cover.gripper_points, the
scorer's own hand model) and the two loose parts along the planned TCP paths at 10 Hz, against every part: the
deck's four strips, the tote's floor and walls, the bench under the hole, the cover and the wrench. The cover sits at
its seeded start until the left hand grips it, is carried rigidly while gripped, and rests where the plan sets it
down (set aside on its cleats, or seated); the wrench likewise with the right hand (on the pocket's floor, carried,
on the tote's floor or the deck spot). A part that falls in a stem (s_slid_off, s_drop) is not tracked after
it is let go.

Left out on purpose: a hand against the part it grips, from closing on it to opening off it; a part against what it
is set down on, from the set-down on; the cover against the deck at its start (it rests there). Pad tips go to 1.5 mm
over the wrench's bottom, so the bench and the tote's floor are held to 1.0 mm; everything else to 3 mm. Prints each
pair's least clearance with when and where; ends with CLEARANCE BAD n. Also the least TCP-to-TCP distance of the two
hands. Planned poses, no IK, no sim."""
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

M = load(HERE / "floor_hole_cover.py").module
LIMIT, FLOOR_LIMIT = 0.003, 0.0010
FLOORISH = {"bench", "tote_floor"}
GRIP_L = {"grip the handle", "lift the cover", "carry to the side", "set it down", "rest on the deck", "let go",
          "grip it again", "lift it again", "carry over the hole", *M.SEAT_LABELS, "lower toward the hole",
          "lower the cleats in", "seat the cover", "let go of the cover", "carry over the edge", "lower over the edge",
          "let go over the edge"}
CARRY_L = {"carry to the side", "carry over the hole", *M.SEAT_LABELS, "lower toward the hole", "carry over the edge"}
# WHY split: lifting out of the hole and lowering the cleats in, the plate rests on the deck at one end of the move
# while the cleats pass the hole's walls (6 mm play), so only the cleats are swept; lifting off the set-aside spot and
# setting down, the cleats stand on the deck while the plate is 19 mm over it, so only the plate and handle are swept
CLEATS_ONLY = {"lift the cover", "lower the cleats in"}
PLATE_ONLY = {"lift it again", "set it down", "lower over the edge"}
HOLDING_TO_END = {"s_held": "keep holding", "s_touch": "keep gripping"}
GRIP_R = {"grip the wrench", "lift out of the hole", "carry to the tote", M.TOTE_LABEL, "lower into the tote",
          "rest on the tote floor", "let go of the wrench", "hold above the floor", "keep holding", "keep gripping",
          "drop height", "carry to the deck spot", "lower to the deck", "rest on the deck spot"}
CARRY_R = {"lift out of the hole", "carry to the tote", M.TOTE_LABEL, "lower into the tote", "hold above the floor",
           "keep holding", "drop height", "carry to the deck spot", "lower to the deck"}
FIXED = {name: (np.asarray(lo, float), np.asarray(hi, float)) for name, lo, hi, _ in M.fixed_boxes()}
FIXED["bench"] = (np.array([-1.0, -1.0, M.TOP - 0.05]), np.array([2.0, 2.0, M.TOP]))
DECK = {k for k in FIXED if k.startswith("deck")}


def box_dists(pts, boxes):
    best = np.full(len(pts), np.inf)
    for lo, hi in boxes:
        q = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
        best = np.minimum(best, np.linalg.norm(q, axis=1))
    return best


def part_points(shapes, n=5):
    """Points on each shape's bounding box: an n x n x n grid over it (corners included)."""
    out = []
    for s in shapes:
        lo, hi = s.bounds()
        g = [np.linspace(lo[i], hi[i], n) for i in range(3)]
        out += [(x, y, z) for x in g[0] for y in g[1] for z in g[2]]
    return np.array(out)


COVER_PTS, WRENCH_PTS = part_points(M.COVER_SHAPES), part_points(M.WRENCH_SHAPES)
CLEAT_PTS = part_points([s for s in M.COVER_SHAPES if s.bounds()[1][2] <= 1e-9])
PLATE_PTS = part_points([s for s in M.COVER_SHAPES if s.bounds()[0][2] >= -1e-9])
COVER_BOX = [s.bounds() for s in M.COVER_SHAPES]
WRENCH_BOX = [s.bounds() for s in M.WRENCH_SHAPES]


def label_at(A, t):
    return A.target(t)[3] if t < A.duration else "done"


def sweep(name, plans, seed):
    lay = M.layout(seed)
    A = {arm: ArmPlan(W, station.BASES[arm]) for arm, W in plans.items()}
    T = max((a.duration for a in A.values()), default=0.0) + 0.5
    cx, cy, cyaw = lay["cover"]
    cover = (M._rz(cyaw), np.array([cx, cy, M.TOP + M.DECK_Z]))
    wx, wy, wyaw = lay["wrench"]
    wrench = (M._rz(wyaw), np.array([wx, wy, M.TOP]))
    rel = {"left": None, "right": None}
    low, bad, t, apart = {}, 0, 0.0, np.inf
    cover_rest, wrench_rest = "start", "start"

    def note(key, d, where, lab):
        k = int(np.argmin(d))
        if key not in low or d[k] < low[key][0]:
            low[key] = (float(d[k]), round(t, 1), lab, np.round(where[k], 3))

    while t <= T:
        tg = {arm: a.target(min(t, a.duration)) for arm, a in A.items()}
        labs = {arm: label_at(A[arm], t) for arm in A}
        if labs.get("right") == "done" and name in HOLDING_TO_END:
            labs["right"] = HOLDING_TO_END[name]
        # the parts follow the hands that grip them
        for arm, part in (("left", "cover"), ("right", "wrench")):
            if arm not in tg:
                continue
            pos, R, _, lab = tg[arm]
            gripped = labs[arm] in (GRIP_L if arm == "left" else GRIP_R) and not labs[arm].startswith("let go")
            st = cover if part == "cover" else wrench
            if gripped and st is not None:
                if rel[arm] is None:
                    rel[arm] = (R.T @ st[0], R.T @ (st[1] - pos))
                new = (R @ rel[arm][0], pos + R @ rel[arm][1])
                if part == "cover":
                    cover, cover_rest = new, labs[arm]
                else:
                    wrench, wrench_rest = new, labs[arm]
            elif rel[arm] is not None:          # let go: rests where it was set down, or is no longer tracked
                rel[arm] = None
                if part == "cover" and name == "s_slid_off" and cover_rest != "rest on the deck":
                    cover = None
                if part == "wrench" and name == "s_drop":
                    wrench = None
        if "left" in tg and "right" in tg:
            apart = min(apart, float(np.linalg.norm(tg["left"][0] - tg["right"][0])))
        for arm, (pos, R, f, lab) in tg.items():
            lab = labs[arm]
            P = pos + M.gripper_points(f) @ R.T
            for part, (lo, hi) in FIXED.items():
                note((arm, part), box_dists(P, [(lo, hi)]), P, lab)
            if cover is not None and not (arm == "left" and lab in GRIP_L):
                note((arm, "cover"), box_dists((P - cover[1]) @ cover[0], COVER_BOX), P, lab)
            if wrench is not None and not (arm == "right" and lab in GRIP_R):
                note((arm, "wrench"), box_dists((P - wrench[1]) @ wrench[0], WRENCH_BOX), P, lab)
        lab_l = labs.get("left")
        if cover is not None and lab_l in CARRY_L | CLEATS_ONLY | PLATE_ONLY:
            pts = CLEAT_PTS if lab_l in CLEATS_ONLY else PLATE_PTS if lab_l in PLATE_ONLY else COVER_PTS
            C = cover[1] + pts @ cover[0].T
            for part, (lo, hi) in FIXED.items():
                if part == "bench":
                    continue
                key = ("cover lift" if lab_l in CLEATS_ONLY else "cover", part)
                note(key, box_dists(C, [(lo, hi)]), C, lab_l)
            if wrench is not None:
                note(("cover", "wrench"), box_dists((C - wrench[1]) @ wrench[0], WRENCH_BOX), C, labs["left"])
        if wrench is not None and "right" in labs and labs["right"] in CARRY_R:
            Wp = wrench[1] + WRENCH_PTS @ wrench[0].T
            for part, (lo, hi) in FIXED.items():
                # WHY no bench: the wrench starts on it; on_deck sets it down on the deck on purpose
                if part != "bench" and not (part.startswith("deck") and labs["right"] == "lower to the deck"):
                    note(("wrench", part), box_dists(Wp, [(lo, hi)]), Wp, labs["right"])
            if cover is not None:
                note(("wrench", "cover"), box_dists((Wp - cover[1]) @ cover[0], COVER_BOX), Wp, labs["right"])
        t += 0.1
    for key in sorted(low):
        d, tt, lab, where = low[key]
        # WHY 1 mm for the cover's lift: it rises straight up along the hole's vertical walls, so the cleats' seeded
        # offset (as close as 1.5 mm to a wall) cannot close; the bench and the tote's floor sit 1.5 mm under the tips
        lim = FLOOR_LIMIT if (key[1] in FLOORISH or key[0] == "cover lift"
                              or (name == "s_on_deck" and key[0] == "right" and key[1] in DECK)) else LIMIT
        flag = d < lim
        bad += int(flag)
        print(f"{name:12s} {key[0]:6s} vs {key[1]:10s} {1000 * d:7.1f} mm at t {tt:5.1f} ({lab}) at "
              f"{where}{'  BAD' if flag else ''}")
    if np.isfinite(apart):
        print(f"{name:12s} hands apart, least TCP distance {1000 * apart:.0f} mm{'  BAD' if apart < 0.10 else ''}")
        bad += int(apart < 0.10)
    return bad


if __name__ == "__main__":
    total = 0
    for s in (0, 1, 2, 3):
        total += sweep("demo", M.plan(s, "demo"), s)
    for v in M.VARIANTS[2:]:
        total += sweep(f"s_{v}", M.plan(0, v), 0)
    print("CLEARANCE BAD", total)
