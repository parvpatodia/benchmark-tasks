# Move cookies from a cookie sheet to a cooling rack with a turner, without touching or dropping them

Three cookie stand-ins lie on a silicone mat on a rimless aluminium cookie sheet. The robot takes an offset turner from
its rest, slides the blade under each cookie, carries it level to a wire cooling rack, tips the blade so the cookie
slides off onto the wires in one layer, and puts the turner back. No part of either arm may touch a cookie, and none may
be dropped.

## What it tests

Getting a thin blade under a flat item that lies loose on a surface, carrying it level on the blade, and setting it down
on wires by sliding it off, three times in a row, with the tool held in one hand by a soft handle. The blade must get
under each cookie from the mat (a blade that passes over them carries nothing, mode miss below), and the set-down must
be gentle (tipped off from 40 mm up, every cookie counts as dropped, mode drop). The closest board task, a spatula
sliding tablets on a counting tray, pushes items along a floor; nothing there gets under an item and lifts it.

Why it is familiar (Societal relatability): moving cookies from the sheet to a rack with a spatula is a familiar last
step of baking cookies at home. An oats brand's baking guide (Quaker, `src/quaker_cooling_cookies.txt`) says "Remove
cookies using a metal spatula" and "Cool in single layer on wire cooling rack.", and warns that a plastic turner "may
result in torn or broken cookies". A baking-mix brand's test kitchen (Betty Crocker,
`src/bettycrocker_cookie_tools.txt`) lists a "flat, thin metal spatula for removing hot, fragile cookies from their
baking sheet" among the essential tools, and recommends "shiny, silver-colored aluminum sheets without sides".

## The cookies: a non-perishable stand-in

Real cookies vary batch to batch and spoil, so the trial uses printed stand-ins that keep what the task depends on: a
standard cookie's size and weight, a rigid flat underside with a rounded edge the blade can get under, and a surface
that grips the baking mat but slides on steel.

- **Size and weight, sourced.** USDA FoodData Central (entry 172807, "Cookies, oatmeal, prepared from recipe, without
  raisins", SR Legacy, saved as `src/usda_fdc_172807_oatmeal_cookie.json`) lists the portion "1 cookie (2-5/8" dia)"
  at 15 g. The stand-in is 66.7 mm across (2-5/8 in) and 15 g. Thickness 6 mm [EST: USDA gives none; a thin, crisp
  oatmeal cookie].
- **Make them once.** Print three discs in PLA, 0.2 mm layers, in a tan filament: 66.7 mm across and 6 mm thick, the
  underside edge eased (the flat bottom ends 4 mm inside the rim and the edge rises 1.5 mm, a conical ring), with the
  infill set so each weighs 15 g (the model's effective density is 727 kg/m3 [ARITH], about 60 % of solid PLA). Print
  them upside down (the flat top on the bed) so the eased edge needs no support; lightly sand its steps.
- **Accept** a disc at 66.2 to 67.2 mm across (calipers, two diameters), 5.8 to 6.2 mm thick and 14 to 16 g (kitchen
  scale), lying flat without rocking. They do not wear or change, so reset is putting them back on their marks.
- **Tilt test** (each session, a protractor): a stand-in on the mat must stay put with the mat at 30 degrees, and on
  the turner's blade must slide by 20 degrees. The model has PLA on the mat at 0.75 static friction (slides at 37
  degrees) and on steel at 0.25 (slides at 14 degrees) [EST, checked by this test].
- Nothing is eaten and nothing is heated: the sheet, mat and turner are used cold.

## The other objects

| Object | In the model | Basis |
|---|---|---|
| Cookie sheet | rimless aluminium, 260 x 320 x 1.5 mm, 0.34 kg, loose | [CHOICE size; ARITH mass at 2,700 kg/m3] |
| Baking mat | fibreglass-reinforced silicone, 240 x 300 x 0.75 mm, 0.06 kg | [CHOICE; EST: calipers, scale] |
| Cooling rack | 200 x 360 mm, wire tops 20 mm up, 2 mm wires every 12 mm along y, 4 mm frame, four feet, 0.25 kg, loose | [EST: ruler, scale] |
| Offset turner | stainless blade 90 x 86 mm, 1.0 mm at the neck ground to 0.3 mm at the tip, 35 mm below a 100 x 22 x 14 mm soft-grip handle, 73 g | [CHOICE width: wider than the 66.7 mm cookie, inside the 2 3/4 in and 4 in blade widths of two stocked solid turners (`src/webstaurant_ateco_solid_turner.txt`, `src/webstaurant_oxo_solid_turner.txt`); EST the rest: ruler, calipers, scale] |
| Rest | wood block 30 x 30 x 28 mm, fixed to the table with double-sided tape, under the back of the handle (the handle's back end overhangs it by 5 mm) | [CHOICE] |

Positions are in `task.json`'s setup and the screenshot. The turner's reference point is the middle of its handle, 50 mm
from either end: on the rest it lies at x 0.070, y -0.600, with the handle's front end (where the neck starts) at x
0.120. Mark both on the table with tape. Any rimless sheet, silicone mat, rack and offset turner within these sizes will
do; measure the ones used and keep them for every trial.

## Forces

All small. The cookie is 15 g; the turner 73 g. The hand holds the handle with 20 N a finger [CHOICE: under the YAM's 40
N per finger]. The blade's tip is aimed 1.5 mm into the mat; through the arm's compliance that is a light press, and the
sheet, held only by its own weight and friction, did not move in the demo runs (sheet in the results table). Friction
pairs (PhysX averages the two materials; each value [EST], checked by the tilt test): PLA on mat 0.75 / 0.63, PLA on
steel 0.25 / 0.20, steel on mat 0.70 / 0.58, sheet on table 0.45 / 0.43 (the table's PhysX default of 0.5 assumed),
handle on the fingers rubber-like.

## How it is modelled

Every object is a loose rigid body of boxes, cylinders and convex hulls in PhysX (`mechanism.py`). The blade is a
tapered hull, 1.0 to 0.3 mm thick. A cookie is a cylinder on a conical ring (its eased edge). The blade gets under a
cookie because it is pitched 6 degrees nose-down with its tip on the mat: the tip meets the eased edge below the
cookie's rim, the edge rides up the blade, and the mat holds the cookie back (0.75 against 0.25 on the blade). Over the
rack the blade turns 60 degrees and tips 22 degrees nose-down, 4 mm over the wires, and the cookie slides off.

Simplified: the stand-ins are rigid and cannot break, so the two things that spoil a real cookie during this step, a
robot part on it and a drop, are what the scorer checks. The 86 mm blade is wider than the 66.7 mm cookie, so a cookie
centred on it does not overhang its sides. Touches are a distance from the arm links' collision meshes (3 mm), not a measured
force.

## Scoring

Each cookie that was carried on the turner's blade, then ends lying flat (within 10 degrees) on the rack's wires, in one
layer (its underside within 3 mm of the wire tops), with its whole outline over the rack's outline (2 mm grace), off the
blade, at rest, never touched and never dropped, scores 0.25. Carried: at some step its centre was over the blade and
its underside within 3 mm of the blade's top, 10 mm or more over the mat, so a cookie pushed, tipped or flicked onto the
rack does not count. Off the blade: not lying on it (centre within a radius of the blade, underside within 5 mm of its
top). Touched: any point of either arm's link collision meshes, every link but the base and link 1, to the finger tips, within 3
mm of the cookie at any step (the base and link 1 stay at the arm's foot, far from the cookies). Dropped: falling faster
than 0.6 m/s (a free fall of about 18 mm), or 18 mm lower than 0.1 s before (two control steps, which the 20 Hz speed
samples can miss); the demo lowers a cookie at most 5.9 mm in 0.1 s (results below). A touched or dropped cookie never
counts again. 1.0 when all three count and, held for 1 s: the turner back on its rest (the handle's middle within 20 mm
of x 0.070, y -0.600, pointing within 15 degrees, lying there), the sheet and the rack each within 30 mm of its place,
nothing moving, every gripper point 30 mm from each cookie and from the turner, both arms read at every step, and the
sheet never tilted over 5 degrees nor raised 5 mm at any step. The last rule is for the free arm: tipping or lifting the
sheet to slide the cookies off gets no full score (and those cookies were never carried, so they score nothing).
Otherwise 0.25 per counted cookie (at most 0.75). Doing nothing scores 0.

Real trial: a photo from above for the outline (inside the rack's frame), a ruler or a side photo for flat and single
layer (each stand-in's top 6 mm over the wires, within 3 mm), a ruler against tape marks for the turner, sheet and rack,
a side photo or the side of the top video for the sheet staying flat on the table, and the top and wrist videos for
touches, drops (a visible fall of about 2 cm or more) and the carry on the blade. Every threshold is a [CHOICE] read
with those tools.

## Modes and results

`TASK_DEMO=<mode>` picks a scripted plan. All results are GPU PhysX, `fastrun.py` (stock `yam_scripted` policy and
`task_score` scorer, cameras off), files hash 766db1bd; scores from each run's `result.json`.

| Mode | Seed | Score | Steps | Ends | What it does |
|---|---|---|---|---|---|
| demo | 0, 1, 2 | 1.00 each | 1,435 | success held 1 s | the full plan: take the turner, lift each cookie onto the rack, put the turner back |
| nothing | 0 | 0.00 | 61 | policy stop at 61 (plan ended) | the arms stay in the ready pose |
| touch | 0 | 0.50 | 1,601 | policy stop at 1,601 (plan ended) | the closed right gripper comes down on the middle cookie on the sheet, then the full plan; the trace shows that cookie touched by both finger tips |
| wrist | 0 | 0.50 | 1,639 | policy stop at 1,639 (plan ended) | the left arm lowers its upper arm and wrist (gripper on its side, tips raised) onto the first cookie, then parks; the right arm runs the full plan; the trace shows that cookie touched by link 3 only (offline kinematics put the gripper 55 mm from it) |
| drop | 0 | 0.00 | 1,489 | policy stop at 1,489 (plan ended) | the full plan, but each cookie is tipped off 40 mm over the wires; all three latched dropped |
| miss | 0 | 0.00 | 1,489 | policy stop at 1,489 (plan ended) | the blade passes level 20 mm over the mat, over the cookies: none is ever carried and all three are left on the sheet |

In every run the sheet moved 0.0 mm and stayed inside the 5 degree and 5 mm limits (traces). The largest two-step fall
of a cookie: 5.9 mm in the demos, 7.2 mm in touch, 27.1 mm in drop (limit 18 mm). Dream-1 (`--policy dream`, seed 0,
the same `.py` files, hash 4173e345 after `task.json` text edits): `task_score: 0` after all 2,000 steps; no cookie left the sheet and the turner
never left its rest.

## Real-trial safety

Low forces and no heat or food. The blade is thin but not sharpened, with rounded corners. Nobody reaches into the
workspace while the arms are enabled. The stand-ins are hard plastic and cannot shatter.

## Run it

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=cookies.py -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Move the three cookies from the cookie \
      sheet onto the cooling rack with the turner, in one layer, without touching or dropping them, then put the \
      turner back on its rest." --scorer task_score --max-steps 2000

One fixed layout (`object_jitter=0`); the demo is open loop, so the seed changes nothing in the scene. `fastrun.py` runs
every mode in one simulator session without cameras; `checkkit.py` runs the offline gates (IK, full-link clearance, run
limit), where mode wrist's one clearance miss (a left arm link inside the first cookie's 3 mm floor) is the point of that
mode; `python3 test_cookies.py` asserts it is the only miss, and checks every scoring rule above and the README's cookie
numbers. The demo uses scene constants and the arm's own joint readings only, never object poses.

Runtime: one trial is at most 2,000 control steps (100 s simulated). The demo ends at 1,435 steps (71.75 s simulated). On
the shared GPU box, with other simulations running, a demo trial took 345 to 422 s of wall time with cameras off in the
runs above (each run's `wall_s`); across every demo run of this task so far the range is 345 to 584 s.

## Assets and licences

All geometry is primitive shapes in this folder. `mechanism.py`, `motionkit.py` and `fastrun.py` carry their MIT notices.
Saved in `src/` (HTML gzipped plus text): Quaker "Cooling Cookies", Betty Crocker "The Cookie Baking Tools You Need",
two turner listings (OXO Good Grips, Ateco 1352), and USDA FoodData Central record 172807 as its API returns it (JSON).
