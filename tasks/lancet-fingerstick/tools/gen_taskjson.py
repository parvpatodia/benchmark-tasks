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
W = M.HAND_KG * 9.81
tip = lambda s: W * (M.COM_X - M.PIVOT_X) / (M.PIVOT_X - s)  # noqa: E731
p = HERE / "task.json"
old = json.loads(p.read_text()) if p.exists() else {}
LAY = "; ".join(f"layout {i}: shifted ({mm(dx)}, {mm(dy)}) mm and turned {deg(yaw)} deg"
                for i, (dx, dy, yaw) in enumerate(M.LAYOUTS))
q_fire = (M.F_FIRE - M.F_PRE) / M.K_SHIELD

d = {
    "title": "Fingerstick a trainer hand: twist the lancet's tab off, steady the hand, fire square on the fingertip, "
             "discard in sharps",
    "category": CATEGORY,
    "keywords": ["fingerstick", "capillary puncture", "lancet", "medical assistant", "bimanual", "sharps", "steadying"],
    "instruction": " ".join(M.__doc__.split()),
    "context": (
        "O*NET 31-9092.00 Medical Assistants (Employment (2025) 833,900) lists as a Core task \"Collect blood, tissue, "
        "or other laboratory specimens, log the specimens, and prepare them for testing.\" The NCCT Medical Assistant "
        "test plan, built on a national job analysis, lists \"Perform capillary punctures (i.e., fingerstick, "
        "heelstick).\" BD's own fingerstick guide for this lancet gives the steps scored: \"Twist off the tab to break "
        "the seal and discard.\"; \"Select finger: middle or ring finger is preferable. The fifth finger must not be "
        "punctured\"; \"press safety lancet firmly against the puncture site. Do not remove the device from the site "
        "until an audible click is heard.\"; \"Discard used safety lancet into a sharps container\". The skill: two "
        "hands at once, one holding the hand still against the press, the other placing a sharp square on a curved "
        "fingertip and pressing without sliding."),
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
        f"preload); it fires at {g(M.F_FIRE)} N on the shield ({mm(q_fire)} mm of travel). It stands tip down in the "
        f"socket. A trainer hand (right hand, silicone, {M.HAND_KG:.3f} kg, life size: ring finger tip radius "
        f"{mm(M.FINGERS['ring'][3])} mm, middle {mm(M.FINGERS['middle'][3])} mm, little {mm(M.FINGERS['little'][3])} "
        f"mm) lies palm up on a weighted base ({g(M.BASE_KG)} kg) with two foam rests, rolled "
        f"{abs(float(deg(M.ROLL)))} deg about its fingers so the ring and middle fingertips' little-finger sides face up; "
        f"its ring fingertip at ({M.HAND0[0]}, {M.HAND0[1]}) and {mm(M.HZ + M.FINGERS['ring'][3])} mm over the table "
        f"(layout 0), its fingers along +x toward the wrist. The hand turns on the finger rest's edge, {mm(M.PIVOT_X)} mm "
        f"behind the ring's tip, and rests on the wrist rest by its weight: a press of {tip(M.SITE_S[1]):.1f} to "
        f"{tip(M.SITE_S[0]):.1f} N on the fingertip's site tips it up unless the forearm stub (a {mm(2 * M.STUB_R)} mm "
        f"cylinder, {mm(M.STUB_X)} mm behind the ring's tip) is held down. The hand's base by seed ({LAY})."),
    "scoring": (
        f"The site: the pad of the middle or ring fingertip, {mm(M.SITE_S[0])} to {mm(M.SITE_S[1])} mm back from the "
        f"apex and {deg(M.SITE_PHI[0])} to {deg(M.SITE_PHI[1])} deg round from the pad's middle line on either side; "
        f"the shield's face within {mm(M.ON_SKIN)} mm of the skin is on it. Square: the lancet's axis within "
        f"{deg(M.TILT_MAX)} deg of the skin's normal there. A slide: the face moving more than {mm(M.SLIDE_MAX)} mm on "
        f"the skin between first touch ({g(M.F_TOUCH)} N) and firing; lifting off ends a placement. Steadied: the "
        f"wrist rises at most {mm(M.WRIST_LIFT_MAX)} mm off its rest while the lancet presses on anything. Rungs at the "
        f"last step: 0.2 the tab twisted off and the lancet out of the stand; 0.4 and pressed at {g(M.F_FIRM)} N or "
        f"more square on the site, steadied; 0.6 and fired there, square, steadied, without a slide; 0.8 and the "
        f"used lancet inside the sharps container, let go by both hands; 1.0 that held {g(M.HOLD_S)} s. Scored 0 "
        f"from then on: the lancet lifted out with its tab on; a press with the hand not steadied; a slide; a fire off "
        f"the site, on the little finger, or tilted; the lancet loose outside the stand and the container for "
        f"{g(M.DROP_S)} s (dropped); the hand's base dragged {mm(M.BASE_MOVED)} mm; the lancet "
        f"{mm(M.TOP - M.LOST_Z)} mm under the table top; the firing model not running."),
    "run_limit": f"{M.RUN_LIMIT_STEPS:,} steps ({M.RUN_LIMIT_STEPS / 20:g} s at 20 Hz)",
    "reset_notes": ("A new lancet in the socket each trial (a used one is spent, and its tab stays in the socket and is "
                    "removed); the hand on its rests by seed; the container emptied of the trial's lancet."),
    "fidelity": (
        "Approximate. The lancet's drive is a sprung shield that fires at a set force; the needle, the puncture and "
        "blood are not simulated. The tab is a sprung turn of its socket that counts as broken at "
        f"{deg(M.TAB_BREAK)} deg. The trainer hand is rigid on a pivot; skin does not deform. Lancet body sizes, "
        "the fire force within its source range and the tab torque are estimates, measured before real trials."),
    "scores": old.get("scores", "pending"),
}
p.write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n")
for k in ("title", "instruction", "context", "setup", "scoring", "fidelity", "scores"):
    print(k, len(d[k]))
