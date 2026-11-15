"""gen_taskjson.py: write ../task.json from solder_joint.py's constants, so every number in the text comes from the code.
Keeps an existing "scores" value. Run with yam_arena on PYTHONPATH."""
import json
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
from yam_arena.task_file import load  # noqa: E402

M = load(HERE / "solder_joint.py").module
mm = lambda v: f"{v * 1000:.1f}".rstrip("0").rstrip(".")  # noqa: E731
g = lambda v: f"{v:g}"  # noqa: E731
IN = 0.0254
p = HERE / "task.json"
old = json.loads(p.read_text()) if p.exists() else {}
TM = [M.TAU * __import__("math").log(1 / (1 - (M.LIQUIDUS - M.T_ROOM) / (t - M.T_ROOM))) for t in M.T_SS]

d = {
    "title": "Sweat a copper water joint: heat it until the solder melts at a touch, feed it full, stop before the flux burns",
    "category": "Impact on jobs",
    "keywords": ["plumbing", "soldering", "copper tube", "torch", "sweat joint", "heat control", "test touch", "bimanual"],
    "instruction": " ".join(M.__doc__.split()),
    "context": (
        "Plumbers, pipefitters and steamfitters (510,600 US workers, O*NET 47-2152); a core task: \"Assemble pipe "
        "sections, tubing, or fittings, using couplings, clamps, screws, bolts, cement, plastic solvent, caulking, or "
        "soldering, brazing, or welding equipment\". CDA, Soldering and Brazing Copper Tube and Fittings: \"touch the "
        "solder to the joint. If the solder does not melt, remove it and continue heating.\" \"Do not overheat the "
        "joint or direct the flame into the face of the fitting cup.\" \"small drops may appear behind the point of "
        "solder application, indicating the joint is full\" \"Experience will indicate the amount of heat and the time "
        "needed.\" Unlike a TIG weld, where the arc shows the heat and filler is dabbed per segment, the joint's heat "
        "is hidden and found by touch, inside a window: solder flows only past its liquidus, the flux burns past 600 F "
        "(our rule, the top of its rated range), and the solder sets round a wire left on as it cools."),
    "setup": (
        f"Coordinates: x forward from the arm bases (at y +-0.305), y left, z up from the table top. Fixed (one bench; "
        f"the coupling's lower joint is already soldered): "
        f"a pipe vise ({mm(0.06)} mm square, {mm(M.VISE_H)} mm tall) at x {M.JX}, y 0 holding a vertical 1/2 in Type L "
        f"copper tube ({g(2 * M.TUBE_R / IN)} in OD) in a wrought coupling whose top (the cup's mouth) is "
        f"{mm(M.Z_MOUTH)} mm over the table; a top tube ({g(2 * M.TUBE_R / IN)} in OD) sits {g(M.CUP_D / IN)} in deep in "
        f"the cup and stands {mm(M.STUB_UP)} mm over it ({g(M.CLEARANCE)} in clearance, cleaned and fluxed). The torch's "
        f"rest (a block under its handle) and the solder's holder (a block with a {mm(M.HOLE)} mm square well, "
        f"{mm(M.HOLDER_H)} mm deep) at ({M.HOLD_XY[0]}, {M.HOLD_XY[1]}). Loose: a lit hose torch ("
        f"{g(sum(M.TORCH_KG))} kg, a {mm(M.HANDLE_W)} mm square handle {mm(M.HANDLE[1] - M.HANDLE[0])} mm long, a burner "
        f"tube and nozzle; its flame's inner cone {mm(-M.FLAME_TIP_Y)} mm ahead of the handle's grip point, which lies "
        f"at ({M.G_REST[0]}, {M.G_REST[1]}) with the flame heading {g(round(M.PHI * 180 / 3.141592653589793, 1))} deg "
        f"from +x); a coil of 1/8 in 95-5 solder wire ({M.COIL_KG * 1000:.0f} g, {mm(2 * M.COIL_R)} mm across, "
        f"{mm(M.COIL_H)} mm tall) with a {mm(M.WIRE_L)} mm straight end standing down in the holder's well. The joint's temperature starts at {g(M.T_ROOM)} F. With the flame's inner cone on the cup's base "
        f"(the flame's mark, facing the torch) the joint heats toward {', '.join(g(t) for t in M.T_SS[:2])} or "
        f"{g(M.T_SS[2])} F by seed (time constant {g(M.TAU)} s), reaching the solder's liquidus after about "
        f"{', '.join(f'{t:.0f}' for t in TM[:2])} or {TM[2]:.0f} s of steady heat; the heat falls off with the inner "
        f"cone's distance d from the mark as exp(-d^2 / (2 ({mm(M.SIGMA)} mm)^2)), and the joint cools by Newton's "
        f"law. Nothing in the scene shows the seed. The wire's end rests on the joint's face, a spring "
        f"({g(M.K_STOP / 1000)} N/mm) under the coupling's top; while the wire's end is on the joint's edge (its inner "
        f"edge within {mm(M.EDGE_R)} mm of the top tube) and pushes the face {mm(M.PRESS_D)} mm or more below the "
        f"solder's level and the joint is at or over {g(M.LIQUIDUS)} F, solder "
        f"melts in at up to {mm(M.FEED_RATE)} mm of wire per second and the face sinks with it (up to "
        f"{mm(M.FED_MAX)} mm). Full: {mm(M.FULL)} mm of wire fed (CDA Table 1: {g(M.FULL / IN)} in of 1/8 in wire for a "
        f"1/2 in joint at {g(M.CLEARANCE)} in clearance); then a ring of solder shows round the joint's edge. If the "
        f"flux burns, a dark ring shows there."),
    "scoring": (
        f"Melting: solder melts into the joint only while the wire's end is on the joint's edge (its inner edge "
        f"within {mm(M.EDGE_R)} mm of the top tube and the lowest point of its end within {mm(M.ON_TOL)} mm of the "
        f"face) and pushes it {mm(M.PRESS_D)} mm or more under the "
        f"solder's level, and the joint is at or over {g(M.LIQUIDUS)} F (the solder's liquidus); fed = the length of "
        f"wire melted in. Full: {mm(M.FULL)} mm or more fed. Set: the joint under {g(M.SOLIDUS)} F (the solidus). Away: "
        f"the flame's inner cone {mm(M.AWAY_FLAME / 10)} cm or more from the mark (the cup's base on the torch side) and "
        f"the wire's tip {mm(M.AWAY_WIRE / 10)} cm or more from the centre of the coupling's mouth, each tool back on its "
        f"stand (its frame within {mm(M.ON_STAND)} mm of its start) or held up (its frame {mm(M.TOOL_Z)} mm or more over "
        f"the table), not let go on the bench. Success (1.0, held "
        f"{g(M.HOLD_S)} s): full, set, away. Partial credit at the last step, each needing the one before: full, set "
        f"and away but not yet held {g(M.HOLD_S)} s 0.8; full {M.LADDER[3]:g}; half full ({mm(M.FULL / 2)} mm) "
        f"{M.LADDER[2]:g}; any solder melted in {M.LADDER[1]:g}; else 0. Scored 0 from then on: the flux burnt (the "
        f"joint over {g(M.BURN_T)} F at any time), the solder set round the wire (the wire's tip still on the face "
        f"after solder went in, as the joint cooled through {g(M.SOLIDUS)} F), the flame on the joint's edge or the "
        f"solder's end (our rule for the CDA caution: the inner cone within {mm(M.FLAME_CLEAR)} mm of the coupling's "
        f"mouth ring or the wire's tip, even when sweeping), the "
        f"torch or the wire "
        f"{mm(M.TOP - M.LOST_Z)} mm below the table top, or the heat model not running (an error, or not stepping "
        f"with the run)."),
    "run_limit": f"{M.RUN_LIMIT_STEPS:,} steps ({M.RUN_LIMIT_STEPS / 20:g} s at 20 Hz)",
    "reset_notes": ("The torch on its rest and the wire in its holder; the joint at room temperature with no solder "
                    "in it, the face at its top and both rings hidden; the heat rate by seed. Nothing else moves."),
    "fidelity": (
        "Approximate. PhysX has no heat or melting: the joint's temperature, the flame's heat and the solder are our "
        "model. The flame is drawn only; the wire does not shorten, its tip sinks as a spring face at the joint's edge "
        "is lowered (the top cup is drawn only). The flame is held on one mark and solder fed at one point (CDA sweeps "
        "the flame and makes overlapping passes); no capillary flow or fillet. Heat rates, falloff and cooling are "
        "estimates, set so the seeds' melt and burn times do not overlap."),
    "scores": old.get("scores", "pending"),
}
p.write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n")
for k in ("title", "instruction", "context", "fidelity", "scores"):
    print(k, len(d[k]))
