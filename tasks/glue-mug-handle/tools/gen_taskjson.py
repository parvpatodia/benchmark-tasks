"""gen_taskjson.py: write ../task.json from mug_handle.py's constants, so every number in the text comes from the code.
Keeps an existing "scores" value. Run with yam_arena on PYTHONPATH."""
import json
import math
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
from yam_arena.task_file import load  # noqa: E402

M = load(HERE / "mug_handle.py").module
mm = lambda v: f"{v * 1000:.2f}".rstrip("0").rstrip(".")  # noqa: E731
g = lambda v: f"{v:g}"  # noqa: E731
r3 = lambda v: f"{v:.3f}".rstrip("0").rstrip(".")  # noqa: E731
p = HERE / "task.json"
old = json.loads(p.read_text()) if p.exists() else {}
LAY = "; ".join(f"layout {i}: handle shifted ({mm(dx)}, {mm(dy)}) mm and turned {g(round(math.degrees(yaw)))} deg, "
                f"mug shifted {mm(mdy)} mm along y" for i, (dx, dy, yaw, mdy) in enumerate(M.LAYOUTS))

d = {
    "title": "Glue a broken mug handle back on: dry-fit it, squeeze a thin bead onto each broken face, seat and press",
    "category": "Societal relatability",
    "keywords": ["mug", "handle", "glue", "repair", "ceramic", "squeeze tube", "dry fit", "household"],
    "instruction": " ".join(M.__doc__.split()),
    "context": (
        "A common home repair: LePage's ceramics guide says \"If the mug has simply "
        "cracked in half or the handle has fallen off, you can follow the steps above\" and names Extreme Glue No Drip "
        "Gel as \"rated safe for items that hold hot liquids\". The glue's technical data sheet (Henkel, LePage "
        "Extreme Glue No Drip Gel, a \"Squeeze Tube\" of \"20 g\") gives the order followed here (masking, "
        "dampening and closing the tube are not scored): \"Pre-fit all materials before applying adhesive\"; "
        "\"Mask off the adjacent areas with tape before gluing if necessary\"; \"Apply thin layer of adhesive to one "
        "surface. Immediately position and press parts firmly together within 5 minutes\"; \"Do not move or use parts "
        "for at least 2 hours\"; Open Time \"3-5 minutes\". The mug is sized from the IKEA DINERA (stoneware, "
        "Height \"9 cm\", \"30 cl\"; its radius is derived, not listed). The skill: meter a bead from a squeeze "
        "tube onto a small face, keep the glaze clean, then mate and press two ends at once."),
    "setup": (
        f"Coordinates: x forward from the arm bases (at y +-0.305), y left, z up from the table top. The demo uses "
        f"the right arm; either arm may be used (both are watched). Fixed: a foam cradle of two rails (inner edges "
        f"{mm(M.RAIL_IN)} mm either side of x {M.MX}, {mm(M.RAIL_H)} mm high, {mm(M.RAIL_L)} mm long along y); a felt "
        f"pad ({mm(2 * M.PAD_HALF)} mm square) at ({M.PAD[0]}, {M.PAD[1]}); a tube holder ({mm(M.HOLD_H)} mm high, a "
        f"{mm(M.HOLD_HOLE)} mm square well) at ({M.HOLD[0]}, {M.HOLD[1]}). Loose: a stoneware mug "
        f"({mm(2 * M.R_MUG)} mm across, {mm(M.L_MUG)} mm long, {g(M.MUG_KG * 1000)} g) lying in the cradle, its axis "
        f"along y at y {M.MY0} (layout 0), its mouth toward +y, two broken stubs on top {mm(M.STUB_Y[1] - M.STUB_Y[0])} "
        f"mm apart, each a {mm(M.SEC_W)} by {mm(M.SEC_D)} mm biscuit-coloured face {mm(M.STUB_H)} mm proud of the "
        f"glaze, each on a stiff spring ({g(M.K_STUB / 1000)} N/mm, {g(M.STUB_PRE)} N preload) that measures the press "
        f"on it; the handle ({g(M.HANDLE_KG * 1000)} g), an arch {mm(M.HANDLE_OUT - M.STUB_H)} mm high with its "
        f"{mm(M.SEC_W)} by {mm(M.SEC_D)} mm broken ends {mm(2 * M.ARCH_A)} mm apart, standing on its ends on the pad; "
        f"a {g(round(sum(M.TUBE_KG) * 1000))} g tube (20 g of gel) standing nozzle down in its holder, cap off, its two "
        f"walls {mm(M.TUBE_W)} mm apart, each sliding in up to {mm(M.SQUEEZE_MAX)} mm against a spring "
        f"({g(M.K_WALL / 1000)} N/mm, {g(M.WALL_PRE)} N preload). Gel leaves the nozzle only while the walls' mean "
        f"squeeze is over {g(M.F_YIELD)} N, at {g(M.FLOW_G)} uL/s per N of it over that, and lands where the "
        f"nozzle points. By seed ({LAY}). The broken faces are clean and dry (prep out of scope)."),
    "scoring": (
        f"Glue lands on a broken face when the nozzle's tip is over it (at least {mm(M.DEP_IN)} mm inside its edges, "
        f"up to {mm(M.DEP_H)} mm above, or resting on it) and points down within {g(round(math.degrees(math.acos(M.UPRIGHT))))} deg; "
        f"otherwise it is astray. A thin bead is {g(M.V_LO)} to {g(M.V_HI)} uL on a face. An end is seated when it is "
        f"centred within {mm(M.SEAT_XY)} mm of its face across and along the mug and every corner of it "
        f"{mm(-M.SEAT_Z[0])} mm into to {mm(M.SEAT_Z[1])} mm over the face in height, the handle "
        f"square to the mug within {g(round(math.degrees(math.acos(M.SEAT_TILT))))} deg. Dry fit: both ends seated "
        f"at once, then the handle standing on its ends on the pad (its centre {mm(M.PAD_HALF - M.PARK_IN)} mm or less "
        f"from the pad's centre each way, within {mm(M.PARK_DZ)} mm of its start height, upright within "
        f"{g(round(math.degrees(M.PARK_TILT)))} deg) with both hands clear, before any glue. Joined: both ends "
        f"seated on thin beads with the tube back in its holder, within {g(M.OPEN_TIME)} s of the first glue; "
        f"lifted off before let go (an end over {mm(M.LIFT_OFF)} mm up, or its centre over {mm(M.SEAT_XY)} mm from its face's centre), it must be seated "
        f"and pressed again. Pressed: then both stubs at {g(M.F_PRESS)} N or more at once for {g(M.PRESS_S)} s. Clear: each hand open (each finger "
        f"{mm(M.OPEN_CLEAR)} mm off the part) and its TCP {mm(M.HAND_CLEAR)} mm off, or {mm(M.FAR_CLEAR)} mm off. Success (1.0, held {g(M.HOLD_S)} s): pressed, both "
        f"hands clear, the handle still (under {g(M.STILL_V * 1000)} mm/s) and unmoved in the mug's frame since let go, the tube back in its holder (within {mm(M.HOME_XY)} mm across and "
        f"{mm(M.HOME_Z)} mm in height, upright within 10 deg), the mug in the cradle. Partial credit at the last step: all that but not yet held "
        f"{g(M.HOLD_S)} s 0.8; joined {g(M.LADDER[3])}; dry-fitted and both beads {g(M.LADDER[2])}; dry-fitted "
        f"{g(M.LADDER[1])}. Scored 0 from then on: glue before the dry fit or before the "
        f"handle is back on the pad, the handle seated on glue before the tube is back, over {g(M.STRAY_MAX)} uL astray, an end "
        f"seated on more than {g(M.V_HI)} uL (squeezed out), the handle moved over {mm(M.MOVED)} mm after it was let "
        f"go on its glue, the mug over {mm(M.IN_CRADLE)} mm out of place or turned over "
        f"{g(round(math.degrees(math.acos(M.MUG_UPRIGHT))))} deg, a part {mm(M.TOP - M.LOST_Z)} mm below the "
        f"table top, or the glue model not running."),
    "run_limit": f"{M.RUN_LIMIT_STEPS:,} steps ({M.RUN_LIMIT_STEPS / 20:g} s at 20 Hz)",
    "reset_notes": ("The mug in its cradle and the handle on its pad by seed; the tube in its holder, unsqueezed; no "
                    "glue anywhere. Nothing else moves."),
    "fidelity": (
        "Approximate. PhysX has no fluids or soft tubes: the tube is two sprung walls; the gel's flow, where it "
        "lands and the bead are our model (the bead is drawn pale and 8 to 14 times a real layer's thickness so "
        "it shows). The stubs are sprung only to measure the press. The mug collides as a "
        f"{M.MUG_SIDES}-sided prism, its centre of mass {mm(M.MUG_COM_DROP)} mm below its axis for the foam's grip. The glue has no strength. The 150 s "
        "run never reaches the open time or the 2 h hold. Handle and stub sizes are estimates."),
    "scores": old.get("scores", "pending"),
}
p.write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n")
for k in ("title", "instruction", "context", "setup", "scoring", "fidelity", "scores"):
    print(k, len(d[k]))
