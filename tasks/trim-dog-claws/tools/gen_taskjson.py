"""gen_taskjson.py: write ../task.json from nail_trim.py's constants, so every number in the text comes from the code.
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

M = load(HERE / "nail_trim.py").module
mm = lambda v: f"{v * 1000:.2f}".rstrip("0").rstrip(".")  # noqa: E731
g = lambda v: f"{v:g}"  # noqa: E731
p = HERE / "task.json"
old = json.loads(p.read_text()) if p.exists() else {}
r3 = lambda v: f"{v:.3f}".rstrip("0").rstrip(".")  # noqa: E731
PIV = M.to_world(M.PIVOT[0], 0.0)
PW = [M.to_world(x, 0.0)[1] for x in M.PAW[0]]
CW = [M.to_world(M.CLAW_X, y) for y in M.CLAW_Y]
LENGTH = M.RING_X + M.RING_OUT - M.HANDLE_X[0]

d = {
    "title": "Trim a dog's black claws with a guillotine clipper, a cut at a time, stopping each when the dot before its quick shows",
    "category": "Impact on jobs",
    "keywords": ["dog", "nail trim", "claw", "guillotine clipper", "quick", "pet care", "grooming", "tool use"],
    "instruction": " ".join(M.__doc__.split()),
    "context": (
        "Cutting nails is in a Core O*NET task of three occupations. 39-2021.00 Animal Caretakers (365,300 US workers): "
        "\"Perform animal grooming duties, such as washing, brushing, clipping, and trimming coats, cutting nails, and "
        "cleaning ears.\" 31-9096.00 Veterinary Assistants and Laboratory Animal Caretakers (132,100): \"Perform hygiene-related duties, such as "
        "clipping animals' claws or cleaning and polishing teeth.\" 29-2056.00 Veterinary Technologists and Technicians (131,400): "
        "\"Bathe animals, clip nails or claws, and brush or cut animals' hair.\" A black claw hides its quick, so it is "
        "cut in steps: a clinic \"trimmed incrementally using a conventional guillotine-style nail clipper\" and "
        "\"Each cut was visually inspected for the appearance of a central black dot\" (H\u00e4usler and S\u00f6hnel "
        "2026); WSU: stop when \"a gray to pink oval starts to appear\". The robot seats each claw's end in a 9 mm ring "
        "between claws 14 mm apart, keeps the clipper square, squeezes through the claw, judges each cut face."),
    "setup": (
        f"Coordinates: x forward from the arm bases (at y +-0.305), y left, z up from the table top. Fixed: a paw "
        f"trainer (as the Erler-Zimmer VET4750: a silicone foreleg on a plexiglass stand) on the left arm's side, its "
        f"leg leaving the stand at ({r3(PIV[0])}, {r3(PIV[1])}), {mm(M.PIVOT[2])} mm up, and reaching toward -y, "
        f"{mm(2 * M.LEG_R)} mm thick, clamped to the stand so it does not move; the paw turned sole up "
        f"at y {r3(min(PW))} to {r3(max(PW))}, its four black claws pointing up in a row at y {r3(CW[0][1])}, x "
        f"{', '.join(r3(c[0]) for c in CW)} (claws 1 to 4), {mm(M.Z_TIP - M.Z_TOE)} mm of each "
        f"showing, the ends {mm(M.Z_TIP)} mm up and {mm(M.CLAW_W[0])} by {mm(M.CLAW_W[1])} mm, the roots "
        f"{mm(M.CLAW_ROOT[0])} by {mm(M.CLAW_ROOT[1])} mm. Loose: a guillotine clipper ({g(sum(M.CLIP_KG) * 1000)} "
        f"g, {mm(LENGTH)} mm long) lying on a rubber mat ({mm(2 * M.MAT_HALF[0])} by {mm(2 * M.MAT_HALF[1])} mm, {mm(M.MAT_H)} mm high) at ({M.MAT[0]}, {M.MAT[1]}): two handles {mm(2 * M.HANDLE_Y)} mm "
        f"apart on centres ({mm(2 * M.HANDLE_Y + M.HANDLE_W)} mm across), the ring {mm(2 * M.RING_IN)} mm across "
        f"{mm(M.RING_X - sum(M.HANDLE_X) / 2)} mm ahead of the handles' middle, {mm(M.RING_T)} mm thick; squeezing the handles together "
        f"by q slides the blade over the ring's top face by q (each handle stops after {mm(M.SQUEEZE_EACH)} mm; each "
        f"has a return spring of {g(M.SPRING_PRE)} N preload, {g(M.SPRING_K / 1000)} N/mm). The quick ends {g(M.QUICK_MM[0])} to "
        f"{g(M.QUICK_MM[-1])} mm below a claw's end at the start (in {g(M.QUICK_MM[1] - M.QUICK_MM[0])} mm steps), drawn "
        f"for each claw on its own from the seed; nothing in the scene shows it until "
        f"a cut face comes within {mm(M.DOT)} mm of it, when a pink-grey dot shows in the middle of the cut face."),
    "scoring": (
        f"A cut: the handles squeezed from open (the blade within {mm(M.Q_OPEN)} mm of its rest) until the blade has "
        f"crossed the hole ({mm(M.Q_CUT)} mm); it cuts each claw whose end is in the ring's hole (within "
        f"{mm(M.IN_RING_R)} mm of its centre, in the clipper's frame, the clipper's up axis within "
        f"{g(round(math.degrees(math.acos(M.UPRIGHT))))} deg of vertical) at the ring's top face, "
        f"taking off what stands above it. A claw is done when its dot shows: its cut face within {mm(M.DOT)} mm of "
        f"its quick and not into it. How much each cut takes is not scored. Success (1.0, held {g(M.HOLD_S)} s): all four claws done and the clipper back on "
        f"its mat (its frame within {mm(M.ON_MAT)} mm of its start, lying flat within "
        f"{g(round(math.degrees(math.acos(M.UPRIGHT))))} deg, moving under {g(M.STILL_V * 100)} cm/s) and let go "
        f"(each hand's TCP {mm(M.CLEAR_R)} mm from its frame, past the fingers' reach). Partial "
        f"credit at the last step: all four done and the clipper down but not yet held {g(M.HOLD_S)} s 0.9; else "
        f"{', '.join(g(x) for x in M.LADDER[1:4])} and {g(M.LADDER[4])} for one to four claws done. Scored 0 from then on: a claw cut to or into its quick (it bleeds), the "
        f"clipper {mm(M.TOP - M.LOST_Z)} mm below the table top, or the cut model not running (an error, or not "
        f"stepping with the run)."),
    "run_limit": f"{M.RUN_LIMIT_STEPS:,} steps ({M.RUN_LIMIT_STEPS / 20:g} s at 20 Hz)",
    "reset_notes": ("The clipper on its mat, open; every claw at its full length, no dot or blood showing; the leg clamped to its "
                    "stand; the quick's lengths by seed. Nothing else moves."),
    "fidelity": (
        "Approximate. PhysX does not cut: the claws, quick, dot and bleeding are our model; a cut claw's tip slides "
        "into the toe, its face flat. The blade is drawn only; the handles slide, not pivot; the claw's resistance "
        "is an estimate (8 N per handle while the blade is in it). The leg and paw are rigid (clamped). Sizes and the 2 mm dot zone are "
        "estimates; the quick lengths are our choice. The cue is drawn pink-grey (WSU's oval; the paper's dot is "
        "black). The paw is sole up so the cut faces face up."),
    "scores": old.get("scores", "pending"),
}
p.write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n")
for k in ("title", "instruction", "context", "fidelity", "scores"):
    print(k, len(d[k]))
