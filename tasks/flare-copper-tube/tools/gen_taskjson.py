"""gen_taskjson.py: write ../task.json from flare_tool.py's constants, so every number in the text comes from the code.
Keeps an existing "scores" value. Run with yam_arena on PYTHONPATH."""
import json
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
from yam_arena.task_file import load  # noqa: E402

M = load(HERE / "flare_tool.py").module
mm = lambda v: f"{v * 1000:.1f}".rstrip("0").rstrip(".")  # noqa: E731
g = lambda v: f"{v:g}"  # noqa: E731
IN = 0.0254
p = HERE / "task.json"
old = json.loads(p.read_text()) if p.exists() else {}
sizes = ", ".join({0.125: "1/8", 0.1875: "3/16", 0.25: "1/4", 0.3125: "5/16", 0.375: "3/8", 0.5: "1/2", 0.625: "5/8",
                   0.75: "3/4"}[s] for s in M.SIZES_IN)

d = {
    "title": "Flare a copper tube in a precision flaring tool, step by step as its maker says",
    "category": "Impact on jobs",
    "keywords": ["plumbing", "HVAC", "copper tube", "flare", "flaring tool", "hand tool", "procedure", "bimanual"],
    "instruction": " ".join(M.__doc__.split()),
    "context": (
        "Plumbers (510,600 US workers, O*NET 47-2152) flare soft copper tube for flare fittings: the CDA Copper Tube "
        "Handbook names the water service's connections at the main and the meter, and fuel gas lines. A "
        "precision flare tool forms the 45 deg flare. The skill is setting up the tool to its tolerances before the one-way "
        "forming: tube end even with the bar's top, yoke centred on the tube, clamp tight, then turned until the clutch "
        "releases and on to iron the flare. RIDGID EC42865: \"Place the tube end even with top "
        "of flaring bar.\" "
        "\"Align the arrow on the yoke with the appropriate line on the flaring bar\" \"Securely tighten clamp screw\" "
        "\"Once the clutch releases, turn the handle at least two more full rotations to ensure a uniform, smooth "
        "flare surface.\" \"Loosen the feed screw to retract the flare cone.\" CDA Copper Tube Handbook: \"Position "
        "the yoke with the flaring cone over the tube end and clamp the yoke in place.\" \"Turn the handle of the "
        "yoke clockwise\""),
    "setup": (
        f"Coordinates: x forward from the arm bases (at y +-0.305), y left, z up from the table top; clockwise is seen "
        f"from above. Loose: the tube ({g(M.TUBE_OD / IN)} in OD, {g(M.TUBE_WALL / IN)} in wall, {mm(M.TUBE_L)} mm "
        f"long, {M.TUBE_KG:g} kg) in the closed bar's 3/8 opening, its end {mm(M.PROUD)} mm over the bar's top, "
        f"gripped by the bar. Fixed: the flaring bar, {g(M.BAR_L / IN)} in long and {mm(M.BAR_H)} mm deep, split along y "
        f"at x {M.BX}, its top {mm(M.Z_BAR - M.TOP)} mm over the table on two blocks; openings {sizes} in, smallest at "
        f"the hinge pin (y {M.HINGE_Y:.4f}), the 3/8 opening's centre at y {M.HY:.4f} with a white line beside each "
        f"opening and a {mm(M.SINK_D)} mm 45 deg countersink at the 3/8 opening's top. The bar's front half is hinged "
        f"at the pin on a light closing spring ({g(M.HINGE_K)} N m/rad, its half-hole {mm(M.PRESS_FIT)} mm into the tube's place at rest) that presses "
        f"its half-hole on the tube. The yoke slides along the bar "
        f"(at rest near the hinged end: its middle {mm(M.HINGE_Y - M.PARK_Y)} mm behind the pin, then 0, "
        f"{mm(M.PARK_OFF[1])} or {mm(M.PARK_OFF[2])} mm further along by seed); a red arrow on its front marks "
        f"the cone's axis. On it: the feed screw's handle (two wings {mm(M.WING_L)} mm tip to tip, top "
        f"{mm(M.WING_Z + M.WING_T - M.TOP)} mm over the table, turning freely about z; the feed screw's red top stands "
        f"up through its hub) and the clamp screw (along x, its head {mm(M.YOKE_X[0] - M.X_HEAD)} mm in front of the "
        f"yoke, its tip {mm(M.CLAMP_GAP)} mm off the bar's front face; a {mm(2 * M.TBAR_HALF)} mm T-bar through the "
        f"head, its -y end {g(M.TBAR_START)} deg up; right-hand, {g(M.CLAMP_PITCH * 1000)} mm per turn, tightened "
        f"clockwise seen from in front; its tip a {g(M.CLAMP_K / 1000)} N/mm spring past contact). The cone's tip "
        f"starts {mm(M.TIP_START[0])}, {mm(M.TIP_START[0] + 0.003)} or {mm(M.TIP_START[1])} mm over the bar's top by "
        f"seed and goes down {mm(M.PITCH)} mm per clockwise turn of the handle (the screw's top sinks with it); its "
        f"clutch releases {mm(M.SEAT_D)} mm under the bar's top (the flare formed), after which the handle "
        f"turns and the cone stays. While the cone presses the tube's end before the flare is formed, it pushes a loose "
        f"tube down with {g(M.THRUST)} N (the closed bar's spring alone does not hold that); once the clamp is "
        f"{g(M.CLAMP_TIGHT)} turn past contact the model stops pushing (in the real tool the clamp holds the tube). "
        f"A flare is formed once, where the tube's end stands when the cone reaches full depth. Tool sizes not in the "
        f"sources are estimates from RIDGID's product photo."),
    "scoring": (
        f"The flare is good when the cone's axis was within {mm(M.CENTRE_TOL)} mm of the tube's at first contact and "
        f"the tube's end was within {mm(M.FLARE_OK)} mm of even with the bar's top when the flare was formed (RIDGID's "
        f"standard flare; above or below changes its diameter), the clamp tight (below) when the cone first met the tube, and "
        f"the cone not brought over the tube from the side (its tip already more than {mm(M.STRIKE_D)} mm under the end); "
        f"otherwise it is scrap. Ironed: {g(M.IRON_TURNS)} or more clockwise turns of the handle after the clutch released, counted while the clamp is tight. Retracted: "
        f"the cone's tip {mm(M.RETRACT)} mm or more over the bar's top. Clamped: the clamp screw turned "
        f"{g(M.CLAMP_TIGHT)} turn or more past its tip meeting the bar."
        f" Set: the tube upright (its axis within 5 deg of vertical) in the 3/8 opening, its axis within {mm(M.IN_TOL)} mm of the opening's, "
        f"its end within {mm(M.FLARE_OK)} mm of the bar's top; on the line: the yoke's arrow within "
        f"{mm(M.CENTRE_TOL)} mm of the 3/8 opening's centre. Clear: both grippers' collision hulls "
        f"{mm(M.CLEAR_MIN)} mm or more from the tube and from the yoke's head, the handle's swing and the T-bar's "
        f"swing. Still: the tube under {mm(M.STILL_V)} mm/s and it and the handle under {M.STILL_W:g} rad/s. Success "
        f"(1.0, held 1 s): good flare, ironed, retracted, clear, still. Partial credit at the last step, each needing "
        f"the one before: retracted, not clear or not still 0.8; ironed, not retracted {M.LADDER[3]:g}; a good flare "
        f"ironed under {g(M.IRON_TURNS)} turns {M.LADDER[2]:g}; set, on the line and clamped, no flare yet {M.LADDER[1]:g}; else 0. "
        f"Scored 0 from then on: a scrap flare, the tube {mm(M.TOP - M.LOST_Z)} mm below the table top, or the forming "
        f"model not running (an error, or not stepping with the run)."),
    "run_limit": f"{M.RUN_LIMIT_STEPS:,} steps ({M.RUN_LIMIT_STEPS / 20:g} s at 20 Hz)",
    "reset_notes": ("The tube in the closed bar's 3/8 opening, its end 4 mm proud; the yoke at the seed's place, the T-bar up, "
                    "the cone set to the seed's start height. Nothing else moves."),
    "fidelity": (
        "Approximate. The clamp holds the tube by our model, not by contact: 0.4 turn past contact, the model stops "
        "the cone's 15 N push on the tube (a loose tube is pushed down). PhysX cannot form copper: the cone does not "
        "collide; the handle's turns set its height and the model records the flare (not drawn). No torque on the "
        "handle or T-bar; the clamp tip is posed from the T-bar's turns. The real bar has no spring: here a light "
        "one holds the tube until clamped. No flare nut. Sizes scaled from a photo."),
    "scores": old.get("scores", "Dream-1 run: pending; demo: pending"),
}
assert len(d["fidelity"]) <= 500, len(d["fidelity"])
p.write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n")
print({k: len(str(v)) for k, v in d.items()})
