"""gen_taskjson.py: write ../task.json from dowel_joint.py's constants, so every number in the text comes from the code.
Keeps existing "scores" and "media" values. Run with yam_arena on PYTHONPATH."""
import json
import math
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
from yam_arena.task_file import load  # noqa: E402

M = load(HERE / "dowel_joint.py").module
mm = lambda v: f"{v * 1000:.2f}".rstrip("0").rstrip(".")  # noqa: E731
g = lambda v: f"{v:g}"  # noqa: E731
p = HERE / "task.json"
old = json.loads(p.read_text()) if p.exists() else {}
LAY = "; ".join(f"seed mod 3 = {i}: x {mm(dx)} mm, yaw {g(round(math.degrees(yw), 2))} deg"
                for i, (dx, yw) in enumerate(M.LAYOUTS))

d = {
    "title": "Dry-fit two loose boards end to end on three dowels: all three into their holes at once, seam closed and flush",
    "category": "Impact on jobs",
    "keywords": ["carpentry", "dowel joint", "dry fit", "multi-pin insertion", "alignment", "two loose parts",
                 "woodworking", "bimanual"],
    "instruction": " ".join(M.__doc__.split()),
    "context": (
        "Carpenters (O*NET 47-2031, Employment 2025: 889,700) have the Core task \"Assemble and fasten materials to "
        "make frameworks or props, using hand tools and wood screws, nails, dowel pins, or glue.\"; cabinetmakers "
        "(51-7011, 81,800): \"Attach parts or subassemblies together to form completed units, using glue, dowels, "
        "nails, screws, or clamps.\" A dowelling jig's manual ends each joint with \"Test fit the joint.\": the dry "
        "fit, the check made before a dowel joint is glued. Three 8 mm dowels must enter three holes together; a board turned a few degrees or off to "
        "one side stops on the dowels' chamfers. Each board starts within reach of one arm only."),
    "setup": (
        f"Coordinates: x forward from the arm bases (at y +-0.305), y left, z up from the table top. Two loose boards "
        f"lie flat on the bare bench, long axes along y: {mm(M.BOARD_L)} x {mm(M.BOARD_W)} x {mm(M.BOARD_T)} mm, "
        f"{g(M.BOARD_KG)} kg each, wood friction {g(M.WOOD['wood'][0])} static / {g(M.WOOD['wood'][1])} dynamic "
        f"(boards, dowels and bore walls), the bench {g(M.TABLE_MU[0])} / {g(M.TABLE_MU[1])}, averaged per pair. "
        f"Pale hole board (sRGB {M.PALE}) centred at ({g(M.B_START[0])}, {g(M.B_START[1])}), yaw 0; three 8 mm round "
        f"holes {mm(M.HOLE_DEPTH)} mm deep in its -y end. Dark dowel board (sRGB {M.DARK}) centred at "
        f"({g(M.A_START[0])}, {g(M.A_START[1])}) shifted along x and turned by seed ({LAY}); three fluted hardwood "
        f"dowels, 8 mm x 40 mm, half in its +y end, {mm(M.DOWEL_OUT)} mm standing out, tips chamfered "
        f"{mm(M.DOWEL_CH)} mm x 45 deg. Dowels and holes {mm(M.PITCH)} mm apart across the board, at mid-thickness. "
        f"The two ends start {mm(M.B_START[1] - M.A_START[1] - M.BOARD_L)} mm apart. In the sim each hole is a "
        f"16-sided bore whose flats are {mm(M.CLR)} mm outside the dowel (a 24-sided prism): a tip enters only within "
        f"{mm(M.CAPTURE)} mm of its hole's axis. Nothing is fixed, clamped or stopped."),
    "scoring": (
        "Only the last step counts. Measured from the two boards' poses in the dowel board's frame: the gap between "
        "the ends at each corner of the seam (along the dowel board, negative if they overlap), the hole board's "
        "sideways offset and height step at its end, each board's tilt, and each dowel's tip depth in its own hole and "
        "distance off that hole's axis. Every rung needs: both corner gaps at least "
        f"{mm(M.OVERLAP)} mm, the height step within {mm(M.LEVEL)} mm, both boards within {g(M.FLAT_DEG)} deg of "
        f"flat, and every dowel's tip at least {mm(M.TIP_IN)} mm into its own hole within {mm(M.CAPTURE)} mm of its "
        "axis (so stacked, overlapping or one-pitch-off boards score 0). 0.25: that. 0.5: and both corner gaps at "
        f"most {mm(M.HALF_GAP)} mm. 0.75: seated: both corner gaps at most {mm(M.SEAT_GAP)} mm and the edges flush "
        f"within {mm(M.FLUSH)} mm. 0.9: seated, every pad point of both grippers at least {mm(M.PAD_CLEAR)} mm from "
        f"both boards (and the dowels) and both boards under {mm(M.STILL)} mm/s. 1.0: that held "
        f"{g(M.SUCCESS_HOLD_S)} s (success). Nothing in the scene can break; no history is kept."),
    "run_limit": f"{M.RUN_LIMIT_STEPS:,} steps ({M.RUN_LIMIT_STEPS // 20} s at 20 Hz)",
    "reset_notes": "Dry fit, nothing consumed: pull the boards apart and set them on their taped outlines.",
    "fidelity": (
        f"Rigid boards and dowels with real contact. The sim's hole is {mm(M.CLR)} mm a side over the dowel, wider "
        "than the real 0 to 0.15 mm, as PhysX convex contacts jam in tighter fits. Kept: the entry capture "
        f"({mm(M.CAPTURE)} mm), the tilt bind (about {g(round(math.degrees(M.BIND_TILT), 1))} deg) and wood friction on "
        "the bore walls. Not kept: an interference press (a dowel a touch over the hole squeezing in), the flutes and "
        "the wood's give, so the sim press is lighter than a tight real fit; the real trial measures it with a force "
        "gauge."),
    "scores": old.get("scores", "Dream-1 run: not run; demo: not run"),
}
if old.get("media"):
    d["media"] = old["media"]
p.write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n")
print("wrote", p, {k: len(v) if isinstance(v, str) else len(", ".join(v)) for k, v in d.items()})
