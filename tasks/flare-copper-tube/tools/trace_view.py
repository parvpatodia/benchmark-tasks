"""trace_view.py <trace.csv> [delay]: the sim trace (sim/log_snippet.py) summarised at the end of each demo phase."""
import csv
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import flare_tool as F  # noqa: E402

rows = [r for r in csv.DictReader(l for l in open(sys.argv[1]) if not l.startswith("#"))]
delay = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
errs = [l.strip() for l in open(sys.argv[1]) if l.startswith("#")]
_, _, info = F.plan(delay)
ends = sorted({(b, arm, lab) for (arm, lab), (a, b) in info["times"].items()})
by_step = {int(r["step"]): r for r in rows}
steps = sorted(by_step)


def at(t):
    s = min(steps, key=lambda k: abs(k / 20.0 - t))
    return by_step[s]


keep = ("hinge", "slide", "feed", "clamp_turn", "clamp_x", "tube_x", "tube_y", "tube_top", "up", "tip_z", "formed",
        "p", "centre_err", "iron", "tight", "stage", "fail")
last = None
print("t_end  arm   phase" + " " * 36 + " ".join(k[:8].rjust(8) for k in keep))
for b, arm, lab in ends:
    if "turn clockwise" in lab or "turn back" in lab or lab.startswith(("open ", "up ", "down ", "close ")) and arm == "left":
        if not lab.endswith((" 1", "b1")) and "turn back" not in lab:
            continue
    r = at(b)
    if r is last:
        continue
    last = r
    vals = []
    for k in keep:
        v = r[k]
        try:
            f = float(v)
            v = f"{f * 1000:.2f}" if k in ("slide", "clamp_x", "tube_top", "tip_z", "p", "centre_err") else (
                f"{(f - F.BX) * 1000:.2f}" if k == "tube_x" else f"{(f - F.HY) * 1000:.2f}" if k == "tube_y" else f"{f:.3f}")
        except ValueError:
            v = v[:8]
        vals.append(v.rjust(8))
    print(f"{b:6.1f} {arm:5s} {lab[:40]:40s} " + " ".join(vals))
print("final:", {k: rows[-1][k] for k in keep}, "steps", rows[-1]["step"])
print("trace errors:", errs[:3])
