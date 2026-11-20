# Move cookies from a cookie sheet to a cooling rack with a turner, without touching or dropping them

Three cookie stand-ins lie on a silicone mat on a rimless aluminium cookie sheet. The robot takes an offset turner from
its rest, slides the blade under each cookie, carries it level to a wire cooling rack, tips the blade so the cookie
slides off onto the wires in one layer, and puts the turner back. Neither gripper may touch a cookie, and none may be
dropped.

## What it tests

Getting a thin blade under a flat item that lies loose on a surface, carrying it level on the blade, and setting it
down on wires by sliding it off, three times in a row, with the tool held in one hand by a soft handle. The blade must
stay pressed on the mat (held 5 mm up it only pushes the cookies along, mode shove below), and the set-down must be
gentle (tipped off from 40 mm up, every cookie counts as dropped, mode drop). The closest board task, a spatula
sliding tablets on a counting tray, pushes items along a floor; nothing there gets under an item and lifts it.

Why it is familiar (Societal relatability): moving cookies from the sheet to a rack with a spatula is a familiar last
step of baking cookies at home. An oats brand's baking guide (Quaker, `src/quaker_cooling_cookies.txt`) says "Remove cookies
using a metal spatula" and "Cool in single layer on wire cooling rack.", and warns that a plastic turner "may result in
torn or broken cookies". A baking-mix brand's test kitchen (Betty Crocker, `src/bettycrocker_cookie_tools.txt`)
lists a "flat, thin metal spatula for removing hot, fragile cookies from their baking sheet" among the essential
tools, and recommends "shiny, silver-colored aluminum sheets without sides".

## The cookies: a non-perishable stand-in

Real cookies vary batch to batch and spoil, so the trial uses printed stand-ins that keep what the task depends on: a
drop cookie's size and weight, a rigid flat underside with a rounded edge the blade can get under, and a surface that
grips the baking mat but slides on steel.

- **Make them once.** Print three discs in PLA, 100 % infill, 0.2 mm layers, in a tan filament: 76 mm across and
  6 mm thick, the underside edge eased (the flat bottom ends 4 mm inside the rim and the edge rises 1.5 mm, a conical
  ring). Print them upside down (the flat top on the bed) so the eased edge needs no support; lightly sand its steps.
  The model is 33 g each at 1,240 kg/m3 [EST density: weigh]. Sizes [CHOICE]: a 3 in drop cookie.
- **Accept** a disc at 75.5 to 76.5 mm across (calipers, two diameters), 5.8 to 6.2 mm thick and 31 to 35 g (kitchen
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
| Offset turner | stainless blade 90 x 65 mm, 1.0 mm at the neck ground to 0.3 mm at the tip, 35 mm below a 100 x 22 x 14 mm soft-grip handle, 63 g | [EST: ruler, calipers, scale] |
| Rest | wood block 30 x 30 x 28 mm, fixed to the table with double-sided tape, under the handle's back end | [CHOICE] |

Positions are in `task.json`'s setup and the screenshot. Any rimless sheet, silicone mat, rack and offset turner within
these sizes will do; measure the ones used and keep them for every trial.

## Forces

All small. The cookie is 33 g; the turner 63 g. The hand holds the handle with 20 N a finger [CHOICE: under the YAM's
40 N per finger]. The blade's tip is aimed 1.5 mm into the mat; through the arm's compliance that is a light press,
and the sheet, held only by its own weight and friction, did not move in the demo runs (sheet in the results table).
Friction pairs (PhysX averages the two materials; each value [EST], checked by the tilt test): PLA on mat 0.75 / 0.63,
PLA on steel 0.25 / 0.20, steel on mat 0.70 / 0.58, sheet on table 0.45 / 0.43 (the table's
PhysX default of 0.5 assumed), handle on the fingers rubber-like.

## How it is modelled

Every object is a loose rigid body of boxes, cylinders and convex hulls in PhysX (`mechanism.py`). The blade is a
tapered hull, 1.0 to 0.3 mm thick. A cookie is a cylinder on a conical ring (its eased edge). The blade gets under a
cookie because it is pitched 6 degrees nose-down with its tip on the mat: the tip meets the eased edge below the
cookie's rim, the edge rides up the blade, and the mat holds the cookie back (0.75 against 0.25 on the blade). Over the
rack the blade turns 60 degrees and tips 22 degrees nose-down, 4 mm over the wires, and the cookie slides off.

Simplified: the stand-ins are rigid and cannot break, so the two things that spoil a real cookie during this step, a
hand on it and a drop, are what the scorer checks. A cookie carried overhanging the blade, which might crack a real
one, is not modelled. Touches are a distance from the gripper's collision meshes (3 mm), not a measured force.

## Scoring

Each cookie that was carried on the turner's blade, then ends lying flat (within 10 degrees) on the rack's wires, in
one layer (its underside within 3 mm of the wire tops), with its whole outline over the rack's outline (2 mm grace), off
the blade, at rest, never touched and never dropped, scores 0.25. Carried: at some step its centre was over the blade
and its underside within 3 mm of the blade's top, 10 mm or more over the mat, so a cookie pushed, tipped or flicked onto
the rack does not count. Off the blade: not lying on it (centre within a radius of the blade, underside within 5 mm of
its top). Touched: any point of either gripper's collision meshes within 3 mm of the cookie at any step. Dropped:
falling faster than 0.6 m/s (a free fall of about 18 mm), or 18 mm lower than 0.1 s before (two control steps, which
the 20 Hz speed samples can miss); the demo lowers a cookie at most 8.5 mm in 0.1 s. A touched or dropped cookie never
counts again. 1.0 when all three count and, held for 1 s: the turner back on its rest (handle middle within 20 mm,
pointing within 15 degrees, lying there), the sheet and the rack each within 30 mm of its place, nothing moving, every
gripper point 30 mm from each cookie and from the turner, and both grippers read at every step. Otherwise 0.25 per counted cookie (at most 0.75). Doing nothing scores 0.

Real trial: a photo from above for the outline (inside the rack's frame), a ruler or a side photo for flat and single
layer (each stand-in's top 6 mm over the wires, within 3 mm), a ruler against tape marks for the turner, sheet and rack,
and the top and wrist videos for touches, drops (a visible fall of about 2 cm or more) and the carry on the blade.
Every threshold is a [CHOICE] read with those tools.

## Modes and results

`TASK_DEMO=<mode>` picks a scripted plan. All results are GPU PhysX, `fastrun.py` (stock `yam_scripted` policy and
`task_score` scorer, cameras off), files hash 33efb9ca; scores from each run's `result.json`.

| Mode | Seed | Score | Steps | Ends | What it does |
|---|---|---|---|---|---|
| demo | 0 | 1.00 | 1434 | success held 1 s | the full plan: take the turner, lift each cookie onto the rack, put the turner back |
| demo | 1 | 1.00 | 1434 | success held 1 s | same |
| demo | 2 | 1.00 | 1434 | success held 1 s | same |
| nothing | 0 | 0.00 | 61 | plan ended | the arms stay in the ready pose |
| touch | 0 | 0.50 | 1600 | run limit | the closed right gripper presses the middle cookie on the sheet, then the full plan; that cookie is latched touched |
| drop | 0 | 0.00 | 1489 | plan ended | the full plan, but each cookie is tipped off 40 mm over the wires; all three latched dropped |
| shove | 0 | 0.00 | 1489 | plan ended | the blade held level 5 mm over the mat: it pushes the cookies along the sheet and none is ever carried |

In every run the sheet moved 0.0 mm (largest move over the run, from each run's trace).

## Real-trial safety

Low forces and no heat or food. The blade is thin but not sharpened, with rounded corners. Nobody reaches into the
workspace while the arms are enabled. The stand-ins are hard plastic and cannot shatter.

## Run it

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=cookies.py -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Move the three cookies from the cookie \
      sheet onto the cooling rack with the turner, in one layer, without touching or dropping them, then put the \
      turner back on its rest." --scorer task_score --max-steps 1600

One fixed layout (`object_jitter=0`); the demo is open loop, so the seed changes nothing in the scene. `fastrun.py`
runs every mode in one simulator session without cameras; `checkkit.py` runs the offline gates (IK, full-link
clearance against every part, run limit); `python3 test_cookies.py` checks every scoring rule above and the README's
cookie numbers. The demo uses scene constants and the arm's own joint readings only, never object poses.

## Runtime

One trial is at most 1,600 control steps (80 s of simulated time). The demo ends at 1434 steps (71.7 s simulated); on the shared GPU box, with other simulations running, a demo trial took 345 to 441 s of wall time with cameras off (each run's `wall_s`), plus about 5 to 13 s to reset.

## Assets and licences

All geometry is primitive shapes in this folder. `mechanism.py`, `motionkit.py` and `fastrun.py` carry their MIT
notices. Saved sources in `src/` (page HTML gzipped, with a text copy): Quaker "Cooling Cookies" and Betty Crocker
"The Cookie Baking Tools You Need".
