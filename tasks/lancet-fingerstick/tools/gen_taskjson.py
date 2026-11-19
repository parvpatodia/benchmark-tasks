"""gen_taskjson.py: write ../task.json from lancet_fingerstick.py's constants, so every number in the text comes from the
code. Keeps an existing "scores" value. Run with yam_arena on PYTHONPATH."""
import json
import math
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
from yam_arena.task_file import load  # noqa: E402

M = load(HERE / "lancet_fingerstick.py").module
CATEGORY = "Impact on jobs"      # WHY one constant: the G15 call (README Closest tasks) may move it to Societal relatability
mm = lambda v: f"{v * 1000:.1f}".rstrip("0").rstrip(".")  # noqa: E731
g = lambda v: f"{v:g}"  # noqa: E731
deg = lambda v: f"{math.degrees(v):.0f}"  # noqa: E731
import importlib.util  # noqa: E402
_spec = importlib.util.spec_from_file_location("tipsweep", HERE / "tools" / "tipsweep.py")
TS = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(TS)
G = M.hold_moment()
K05, _ = TS.sweep(0.5)
p = HERE / "task.json"
old = json.loads(p.read_text()) if p.exists() else {}
LAY = (f"shifted in x and in y each uniform within +-{mm(M.SHIFT_MAX)} mm and turned uniform within "
       f"+-{deg(M.TURN_MAX)} deg about a point {mm(M.TURN_C)} mm behind the ring's tip along the hand (the palm's "
       f"middle), drawn from the trial's seed")
q_fire = (M.F_FIRE - M.F_PRE) / M.K_SHIELD

d = {
    "title": "Fingerstick a trainer hand: twist the lancet's tab off, steady the hand, fire square on the fingertip, "
             "discard in sharps",
    "category": CATEGORY,
    "keywords": ["fingerstick", "capillary puncture", "lancet", "medical assistant", "bimanual", "sharps", "steadying"],
    "instruction": " ".join(M.__doc__.split()),
    "context": (
        "O*NET Core tasks (relevant to at least 67% of incumbents): "
        "Medical Assistants 31-9092.00 (Employment (2025) 833,900), \"Collect blood, tissue, or other laboratory "
        "specimens, log the specimens, and prepare them for testing.\"; Phlebotomists 31-9097.00 (143,900), \"Draw "
        "blood from capillaries by dermal puncture, such as heel or finger stick methods.\"; Licensed Practical and "
        "Licensed Vocational Nurses 29-2061.00 (666,900), \"Collect samples, such as blood, urine, or sputum from "
        "patients, and perform routine laboratory tests on samples.\" The NCCT medical assistant test plan lists \"Perform capillary punctures (i.e., fingerstick, "
        "heelstick).\" BD's guide "
        "gives the steps scored: \"Twist off the tab to break the seal and discard.\"; \"The fifth finger "
        "must not be punctured\"; \"press safety lancet firmly against the puncture site.\"; \"Discard used safety "
        "lancet into a sharps container\". The skill: one hand keeps the hand still while the other presses a sharp "
        "square on a curved fingertip without sliding."),
    "setup": (
        f"Coordinates: x forward from the arm bases (at y +-0.305), y left, z up from the table top. Fixed: a stand "
        f"({mm(2 * M.STAND_HALF)} mm square, {mm(M.STAND_H)} mm high) at ({M.STAND[0]}, {M.STAND[1]}) with a socket on "
        f"top keyed to the lancet's front ({mm(M.CUP_D)} mm deep, the lancet's tab in its floor); the tab breaks off "
        f"when the lancet is turned {deg(M.TAB_BREAK)} deg in the socket, against {g(100 * M.T_TAB)} N-cm. A BD 305635 "
        f"sharps container ({mm(M.SH_L)} x {mm(M.SH_W)} x {mm(M.SH_H)} mm), centred at ({M.SHARPS[0]}, "
        f"{M.SHARPS[1]}), its lid opening {mm(M.SH_OPEN[0])} x {mm(M.SH_OPEN[1])} mm. Loose: a BD Microtainer "
        f"contact-activated lancet 366577 (21 G x 1.8 mm, pink), modelled as a {mm(M.L_W)} x {mm(M.L_T)} x "
        f"{mm(M.L_LEN)} mm polypropylene housing ({g(1000 * sum(M.LANCET_KG))} g) and a {mm(2 * M.SHIELD_R)} mm contact "
        f"shield standing {mm(M.SHIELD_OUT)} mm proud on a spring ({g(M.K_SHIELD / 1000)} N/mm, {g(M.F_PRE)} N "
        f"preload); it fires at {g(M.F_FIRE)} N on the shield ({1000 * q_fire:.2f} mm of travel). It stands tip down in the "
        f"socket. A trainer hand (right hand, rigid, printed PETG, {M.HAND_KG:.3f} kg with its ballast, life size: ring "
        f"finger tip radius {mm(M.FINGERS['ring'][3])} mm, middle {mm(M.FINGERS['middle'][3])} mm, little "
        f"{mm(M.FINGERS['little'][3])} mm) lies palm up on a weighted base ({g(M.BASE_KG)} kg), rolled "
        f"{abs(float(deg(M.ROLL)))} deg about its fingers so the ring and middle fingertips' little-finger sides face up; "
        f"its ring fingertip at ({M.HAND0[0]}, {M.HAND0[1]}) and {mm(M.HZ + M.FINGERS['ring'][3])} mm over the table "
        f"(before its shift), its fingers along +x toward the wrist. The hand turns on a hinge pin across its fingers, "
        f"{mm(M.PIVOT_X)} mm behind the ring's tip at the height of the ring finger's axis "
        f"({mm(M.HZ + M.HINGE_Z)} mm over the table), and rests by its weight on a block under its wrist, a moment of "
        f"{G:.3f} N m about the hinge. Unless the forearm stub (a {mm(2 * M.STUB_R)} mm cylinder, {mm(M.STUB_X)} mm "
        f"behind the ring's tip) is held down, a straight-down press on the site tips the hand fingertips down at "
        f"{TS.vertical_tip_force('ring', M.SITE_S[0]):.1f} N ({mm(M.SITE_S[0])} mm back on the ring) to "
        f"{TS.vertical_tip_force('ring', M.SITE_S[1]):.1f} N ({mm(M.SITE_S[1])} mm back), or "
        f"{TS.vertical_tip_force('middle', M.SITE_S[0]):.1f} to {TS.vertical_tip_force('middle', M.SITE_S[1]):.1f} N "
        f"on the middle; any press the square rule allows on the site, with friction up to 0.5, tips it by "
        f"{G / K05:.1f} N. The hand's base is {LAY}."),
    "scoring": (
        f"The site: the pad of the middle or ring fingertip, {mm(M.SITE_S[0])} to {mm(M.SITE_S[1])} mm back from the "
        f"apex and {deg(M.SITE_ANG[0])} to {deg(M.SITE_ANG[1])} deg round from the pad's middle line on the side "
        f"that faces up; "
        f"the shield's face within {mm(M.ON_SKIN)} mm of the skin is on it. Square: the lancet's axis within "
        f"{deg(M.TILT_MAX)} deg of the skin's normal there. A slide: the face moving more than {mm(M.SLIDE_MAX)} mm on "
        f"the skin between first touch ({g(M.F_TOUCH)} N) and firing; lifting off ends a placement. Steadied: the "
        f"wrist rises at most {mm(M.WRIST_LIFT_MAX)} mm off its block while the lancet presses on anything. Out of the "
        f"stand: the lancet moved more than {mm(M.OUT_XY)} mm across or {mm(M.LIFT_OUT)} mm up from where it stood; "
        f"once out it stays out, so a lancet put back in the stand and let go is a drop. Rungs at the "
        f"last step: 0.2 the tab twisted off and the lancet out of the stand; 0.4 and pressed at {g(M.F_FIRM)} N or "
        f"more square on the site, steadied; 0.6 and fired there, square, steadied, without a slide; 0.8 and the "
        f"used lancet inside the sharps container, let go by both hands; 1.0 that held {g(M.HOLD_S)} s. Scored 0 "
        f"from then on: the lancet lifted out with its tab on; the lancet fired in the stand (tab off, pressed down "
        f"on the post); a press with the hand not steadied; a slide; a fire off the site, on the little finger, or "
        f"tilted; the lancet, once out of the stand, held by neither gripper and outside the container for "
        f"{g(M.DROP_S)} s (dropped; held: a gripper's finger half-gap {mm(M.HELD_GAP[0])} to {mm(M.HELD_GAP[1])} mm, so "
        f"closed on either side of the housing, and its TCP within {mm(M.HELD_R)} mm of the lancet's axis from "
        f"{mm(-M.HELD_AX[0])} mm beyond its face to {mm(M.HELD_AX[1])} mm up it; let go: a gripper's half-gap over "
        f"{mm(M.HELD_GAP[1])} mm or its TCP over {mm(M.RELEASE_R)} mm from that axis); the hand's base dragged "
        f"{mm(M.BASE_MOVED)} mm; the lancet "
        f"{mm(M.TOP - M.LOST_Z)} mm under the table top; the firing model not running."),
    "run_limit": f"{M.RUN_LIMIT_STEPS:,} steps ({M.RUN_LIMIT_STEPS / 20:g} s at 20 Hz)",
    "reset_notes": ("A new lancet in the socket each trial (a used one is spent, and its tab stays in the socket and is "
                    "removed); the hand on its hinge and wrist block, its base placed by seed; the container emptied of the trial's lancet."),
    "fidelity": (
        "Approximate. The lancet's drive is a sprung shield that fires at a set force; needle, puncture and blood are "
        "not simulated. The tab is a sprung turn of its socket that breaks at "
        f"{deg(M.TAB_BREAK)} deg of twist, as BD's guide directs; a straight pull-off is not modelled. The trainer "
        "hand is rigid on a hinge; skin does not deform. The lancet's body size, "
        "its fire force (inside another maker's published range) and the tab torque (under another maker's limit) "
        "are estimates, to be measured before real trials."),
    "scores": old.get("scores", "pending"),
}
if "media" in old:   # WHY: tools/upload_media.py writes it; regenerating the text must not drop it
    d["media"] = old["media"]
p.write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n")
for k in ("title", "instruction", "context", "setup", "scoring", "fidelity", "scores"):
    print(k, len(d[k]))
