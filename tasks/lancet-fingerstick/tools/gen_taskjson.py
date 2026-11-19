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
_spec = importlib.util.spec_from_file_location("yieldsweep", HERE / "tools" / "yieldsweep.py")
YS = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(YS)
GIVE = YS.give_force()
p = HERE / "task.json"
old = json.loads(p.read_text()) if p.exists() else {}
LAY = (f"shifted in x and in y each uniform within +-{mm(M.SHIFT_MAX)} mm and turned uniform within "
       f"+-{deg(M.TURN_MAX)} deg about a point {mm(M.TURN_C)} mm behind the ring's tip along the hand (the palm's "
       f"middle), drawn from the trial's seed")
q_fire = (M.F_FIRE - M.F_PRE) / M.K_SHIELD

d = {
    "title": "Fingerstick a trainer hand: twist the lancet's tab off, hold the ring finger still, fire square, discard "
             "in sharps",
    "category": CATEGORY,
    "keywords": ["fingerstick", "capillary puncture", "lancet", "phlebotomy", "medical assistant", "bimanual", "sharps"],
    "instruction": " ".join(M.__doc__.split()),
    "context": (
        "O*NET Core tasks (relevant to at least 67% of incumbents): Phlebotomists 31-9097.00 (Employment (2025) "
        "143,900), \"Draw blood from capillaries by dermal puncture, such as heel or finger stick methods.\"; Medical "
        "Assistants 31-9092.00 (833,900), \"Collect blood, tissue, or other laboratory specimens, log the specimens, "
        "and prepare them for testing.\" The CMA (AAMA) exam outline lists \"Capillary/Dermal puncture (finger, "
        "heel, ear)\"; NCCT's "
        "medical assistant plan, \"Perform capillary punctures (i.e., fingerstick, heelstick).\" BD's guide: \"Twist "
        "off the tab\"; \"press safety lancet firmly against the puncture site.\" A hospital laboratory procedure "
        "for this lancet's family: \"Grasp the finger, and using a sterile lancet, press firmly against the finger to "
        "make a puncture.\" The skill: one hand holds the punctured finger still while the other fires a sharp square on its "
        "curved tip."),
    "setup": (
        f"Coordinates: x forward from the arm bases (at y +-0.305), y left, z up from the table top. Fixed: a stand "
        f"({mm(2 * M.STAND_HALF)} mm square, {mm(M.STAND_H)} mm high, printed) at ({M.STAND[0]}, {M.STAND[1]}): a "
        f"fixed post holds the lancet's tab cap and a cup on it, keyed to the lancet's front ({mm(M.L_W + 2 * M.CUP_CLEAR)} "
        f"x {mm(M.L_T + 2 * M.CUP_CLEAR)} mm pocket, {mm(M.CUP_D)} mm deep), turns up to {deg(M.TAB_STOP)} deg either "
        f"way; the tab breaks off when the lancet is turned {deg(M.TAB_BREAK)} deg either way, against "
        f"{g(100 * M.T_TAB)} N-cm (rig.scad, parts stand and cup). A BD 305635 sharps container ({mm(M.SH_L)} x "
        f"{mm(M.SH_W)} x {mm(M.SH_H)} mm), centred at ({M.SHARPS[0]}, {M.SHARPS[1]}), its lid opening "
        f"{mm(M.SH_OPEN[0])} x {mm(M.SH_OPEN[1])} mm. Loose: a BD Microtainer contact-activated lancet 366577 (21 G x "
        f"1.8 mm, pink), modelled as a {mm(M.L_W)} x {mm(M.L_T)} x {mm(M.L_LEN)} mm polypropylene housing "
        f"({g(1000 * sum(M.LANCET_KG))} g) and a {mm(2 * M.SHIELD_R)} mm contact shield standing {mm(M.SHIELD_OUT)} mm "
        f"proud on a spring ({g(M.K_SHIELD / 1000)} N/mm, {g(M.F_PRE)} N preload); it fires at {g(M.F_FIRE)} N on the "
        f"shield ({1000 * q_fire:.2f} mm of travel). It stands tip down in the cup. A fingerstick trainer (printed "
        f"PETG, rig.scad) lies palm up on a {g(M.BASE_KG)} kg base plate taped down: a palm block with an index finger "
        f"and a thumb, fixed, and a ring finger (life size, tip radius {mm(M.FINGERS['ring'][3])} mm, "
        f"{g(1000 * M.RING_KG)} g) on a vertical linear guide under its knuckle: two springs ({g(M.K_RING / 1000)} N/mm in "
        f"all) hold it up on a top stop with {g(M.RING_PRELOAD)} N to spare over its weight, so a downward push of "
        f"{GIVE:.2f} N moves it {mm(M.DROP_MAX)} mm, and it stops {mm(M.DROP_STOP)} mm down. No middle or little "
        f"finger. The hand is rolled {deg(abs(M.ROLL))} deg about its fingers so the side of the ring "
        f"fingertip faces up; the ring's tip at ({M.HAND0[0]}, {M.HAND0[1]}) and {mm(M.HZ + M.FINGERS['ring'][3])} mm "
        f"over the table, the fingers pointing toward the wrist {deg(abs(M.HAND_YAW))} deg right of +x (before "
        f"the seed's shift). The base is {LAY}."),
    "scoring": (
        f"The site: the side of the ring fingertip's pad that faces up, {mm(M.SITE_S[0])} to {mm(M.SITE_S[1])} mm "
        f"back from the apex and {deg(M.SITE_ANG[0])} to {deg(M.SITE_ANG[1])} deg round from the pad's middle line; "
        f"the shield's face within {mm(M.ON_SKIN)} mm of the skin is on it. Square: the lancet's axis within "
        f"{deg(M.TILT_MAX)} deg of the skin's local normal where it touches. A slide: the face moving more than "
        f"{mm(M.SLIDE_MAX)} mm on the skin between first touch (the shield pushed off its stop, {g(M.F_TOUCH)} N) "
        f"and firing; lifting off ends a placement. Held: a gripper closed on the ring finger, its finger half-gap "
        f"{mm(M.HOLD_GAP[0])} to {mm(M.HOLD_GAP[1])} mm (within {mm(M.HOLD_GAP_ON - M.HOLD_GAP[0])} mm of the "
        f"{mm(M.HOLD_GAP_ON)} mm it reads closed on the finger), its TCP within {mm(M.HOLD_R)} mm of the finger's axis "
        f"between its distal joint and its base and at most {mm(M.HOLD_ACROSS)} mm off the axis along its closing "
        f"direction, its pad tips below the axis; it must hold at every step from the lancet's first touch until "
        f"{g(M.STEADY_AFTER)} s after the click. Still: the ring finger pushed at most {mm(M.DROP_MAX)} mm down, read at "
        f"its slide and at its fingertip, over the same window. Out of the stand: the lancet moved "
        f"more than {mm(M.OUT_XY)} mm across or {mm(M.LIFT_OUT)} mm up from where it stood; once out it stays out, so "
        f"a lancet put back in the stand and let go is a drop. Rungs (0 once any fault has latched; 0.4 and 0.6 count "
        f"once reached, the others are read at the last step): 0.2 the tab twisted off and the lancet out of the "
        f"stand; 0.4 and the lancet has touched the site (contact: the shield off its stop), square, with the ring "
        f"finger held and still; 0.6 and fired there, square, held, still, without a slide; 0.8 and the used lancet "
        f"inside the sharps container, let go by both hands; 1.0 that held {g(M.HOLD_S)} s. Scored 0 from then on: "
        f"the lancet lifted out with its tab on; the lancet fired in the stand (tab off, pressed down on the post); "
        f"the ring finger not still; a step without the hold from first touch to {g(M.STEADY_AFTER)} s after the "
        f"click; a fire with no gripper holding the ring finger; a slide; a fire off the "
        f"site or tilted; the lancet, once out of the stand, held by neither gripper and outside the container for "
        f"{g(M.DROP_S)} s (dropped; held: a gripper's finger half-gap {mm(M.HELD_GAP[0])} to {mm(M.HELD_GAP[1])} mm, "
        f"so closed on either side of the housing, and its TCP within {mm(M.HELD_R)} mm of the lancet's axis from "
        f"{mm(-M.HELD_AX[0])} mm beyond its face to {mm(M.HELD_AX[1])} mm up it; let go: a gripper's half-gap over "
        f"{mm(M.HELD_GAP[1])} mm or its TCP over {mm(M.RELEASE_R)} mm from that axis); the trainer's base dragged, "
        f"lifted or pushed down {mm(M.BASE_MOVED)} mm, or turned or tipped {deg(M.BASE_TURN)} deg; the lancet {mm(M.TOP - M.LOST_Z)} mm under the table top; the firing model not "
        f"running. Real trials (README) read the touch on a kitchen scale under the trainer (a change of "
        f"{round(1000 * M.REAL_TOUCH / M.GRAVITY)} g, {g(M.REAL_TOUCH)} N, a one-way bound on the sim's touch), the "
        f"stillness on steel rules beside the web and the fingertip (side camera, 60 fps), the hold on the cameras "
        f"and the arm's log, the click on audio."),
    "run_limit": f"{M.RUN_LIMIT_STEPS:,} steps ({M.RUN_LIMIT_STEPS / 20:g} s at 20 Hz)",
    "reset_notes": ("A new lancet in the cup each trial (a used one is spent; its tab stays in the post's slot and is "
                    "removed); the ring finger back on its top stop; the base placed by seed; the container emptied "
                    "of the trial's lancet."),
    "fidelity": (
        "Approximate. The lancet's drive is a sprung shield that fires at a set force; needle, puncture and blood are "
        f"not simulated. The tab is a sprung turn that breaks at {deg(M.TAB_BREAK)} deg either way; a pull-off is not "
        "modelled. A real finger's give at its joints, wrist and arm is one sprung slide at its knuckle; skin "
        "does not deform. The lancet's size, fire force, tab torque and the slide's spring are estimates, measured "
        f"before trials; the held rule's on-finger half-gap ({mm(M.HOLD_GAP_ON)} mm) is planned, checked on the "
        "film."),
    "scores": old.get("scores", "pending"),
}
if "media" in old:   # WHY: tools/upload_media.py writes it; regenerating the text must not drop it
    d["media"] = old["media"]
p.write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n")
for k in ("title", "instruction", "context", "setup", "scoring", "fidelity", "scores"):
    print(k, len(d[k]))
