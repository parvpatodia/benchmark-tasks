# Hang a push mower's grass bag under its sprung rear flap

A corded 14 in push mower (Sun Joe MJ401E) stands on the bench, rear toward the arms, unplugged and with no blade. Its
spring-loaded ejector flap hangs shut in front of the two coupling hooks on its rear face. Its empty grass bag stands
on the bench behind it, the steel rod across the bag's mouth toward the mower. The robot does what the manual says:
raise the flap and hold it up, set the bag's rod down into both hooks, release the flap so it closes onto the bag,
and take both hands away.

Files: `mower_bag.py` (scene, measurement, scorer, scripted demo and stems), `mechanism.py` (third party, MIT: sprung
fixed mechanisms, loose multi-shape bodies and the step hooks), `test_mower_bag.py` (offline tests), `tools/` (IK
check, gripper clearance sweep), `sim/` (run scripts and the stem copies' maker).

## Why it matters

A bagging mower's bag comes off to be emptied and has to go back on correctly. The manual's method uses both hands:
the flap is sprung shut over the hooks, so one hand holds it up while the other sets a 10 mm rod into two 21 mm slots
160 mm apart with the bag hanging from the hand, then the flap is let down onto the bag, which holds it in place. The
manual also warns against running the mower without the whole catcher in place (Sources). A bag on one hook, a rod
resting on the hooks' lips, or a flap propped open on a hand are the failures the score separates from a fitted bag.

## Sources

Every number in the code carries a marker: `[SOURCE]` quoted here, `[ARITH]` derived, `[EST]` estimate, `[CHOICE]`
design choice, `[RUN]` computed by a tool. Numbers with none are layout and timing choices of the scripted demo.

Primary source: Snow Joe, "Electric Lawn Mower, 14-inch, 12-amp, Model MJ401E, Operator's Manual", Form No.
SJ-MJ401E-880E-MR7, https://images.homedepot-static.com/catalog/pdfImages/27/27d4e9e2-4909-464a-82fa-1db54b5480a7.pdf
(sha256 ee32fe214d856ebd482b93d350d102be35fe0c0f016de0257e76e28f57df2584). An older revision, Form No. SJ-MJ401E-880E,
https://images.thdstatic.com/catalog/pdfImages/1d/1d3be808-6342-41d3-b7b3-edef0fd61712.pdf
(sha256 b787b7298cd3fe2ed7540b2513e6bb2e14d1bb189105d70007976109124cc574). Both PDFs were read directly.

| What | Source and quote | Used for |
|---|---|---|
| Mower | MR7 p. 5, Technical Data: "Weight 29 lbs (13.2 kg)", "Wheel Size 5.5 in. (14 cm)", "Cutting Width 14 in. (35.6 cm)", "Grass Collection Bag Capacity 10.6 gal (40 l)" | mass, wheels, bag volume |
| Fitting the bag | MR7 p. 7, step 3: "Raise the spring loaded ejector flap and move the grass bag up to the mower body. Fasten it onto the coupling hooks (Fig. 6)."; step 4: "Release the ejector flap to secure the grass bag in position (Fig 7)." | the instruction and every rung |
| Bag frame and flap shape | MR7 p. 7, step 1: "Fit the support rod into the holes in the plastic cover. The bend of the support rod should face the rear of the machine (Fig. 4)."; Figs. 4 to 7 show the rod as a bent frame through the cover and the flap as a wide rear door lying over the front of the bag's top | the fidelity notes (below) |
| Why a fitted bag matters | MR7 p. 1: "Do not operate without the entire grass catcher, discharge guard, rear guard or other safety protective devices in place and in proper working order." (again on p. 3); SJ-MJ401E-880E p. 1: "Risk of injury could occur if the lawn mower is operated without the catcher assembly attached." | Why it matters |
| Spare bag | MR7 p. 14, Optional Accessories: "10.6-gallon (40 l) grass bag MJ401E-10" | real trial |
| Feeler gauge | Starrett 66MA: "THICKNESS GAGE, 20 LEAVES, 0.05-1.0mm", leaves "0.05, 0.10, ... 0.50, ... 1.0" | the 1.5 mm and 5 mm checks |

Not sourced, all [EST] and measured on the real mower before trials (below): the hooks' size, height and spacing, the
flap's size, mass, spring and hinge, the bag's shape, rod, handle and masses, all friction values, the mower's rolling
resistance. No household-share figure for bagging mowers was found.

Fidelity against the manual's figures. The real support rod is bent and runs through holes in the bag's plastic cover,
forming its frame (Fig. 4); the sim models only its straight cross-piece, 300 mm long, on two short arms 32 mm long.
The real flap is a wide rear door that lies over the front of the bag's top (Fig. 7); the sim's flap is 240 x 100 mm,
narrower than the 300 mm bag, its size an estimate from Fig. 7. A wider flap was tried and dropped: at 280 mm and up
the left hand's pinch beside its edge is out of reach (IK error 1.0 to 4.6 mm on contact waypoints, RUN).

## The scene (world: x forward from the arm bases, y left, z up from the bench)

- Mower: rear face at x 0.53. Rear housing 60 mm deep, y +-130 mm, up to 400 mm; deck 320 mm long to the bench's far
  edge (x 0.91), y +-180 mm; motor cover; four 140 mm wheels, the front axle at x 0.83. It rides a fore-aft slide with
  its wheels 0.5 mm off the bench: 3.9 N of friction [EST: rolling resistance 0.03 x 129 N] and 10 N s/m of damping
  [CHOICE], from 150 mm back to 80 mm forward, where the front wheels reach the bench edge [ARITH]. The arms can roll a
  13.2 kg mower but not slide it sideways (about 84 N at a rubber-on-bench friction of 0.65 [EST]) or tip it, so only
  the roll is modelled.
- Hooks: two steel hooks at y +-80 mm, 16 mm wide. Each is a floor plate 25 mm deep standing out of the rear face, top
  at 285 mm, with a 20 mm lip at its back: a 21 mm slot, so a 10 mm rod has 5.5 mm of play a side [ARITH].
- Flap: 240 x 100 x 4 mm plastic, 20 mm lip along its lower edge, hinged at x 0.49, 355 mm up. Shut, it stands 15 mm in
  front of the hooks' lips and reaches 30 mm below their floor (test). Spring 0.15 N m/rad wound 1.33 rad when shut
  (0.20 N m [ARITH]); spring and weight close it from every angle to 125 deg (test): no detent, nothing holds it open.
- Bag (rigid): 500 x 300 x 267 mm body (40.05 l [ARITH] for the manual's 40 l), 8 mm hard cover, 10 mm x 300 mm steel
  rod 32 mm in front of the mouth, carry bar over the centre of mass (221 mm behind the rod [ARITH], test), 1.0 kg.
  Hung in both hooks, it turns 3.2 deg tail-down about the rod until its tail rests on the bench [ARITH, test], and
  the flap closes to 50 deg with its edge on the bag's cover [ARITH, test].
- Start: the bag stands 85 mm short of the slots; by seed moved 0 to 10 mm back, +-15 mm sideways and turned +-3 deg
  about the rod's centre. At 3 deg the rod's ends move 7.8 mm, past the slot's 5.5 mm play, so one fixed open-loop
  plan cannot fit every seed (test).

## Two hands, and the one-hand route

The rod alone is not blocked: between the shut plate and the hooks' lips is a 15 mm channel that takes the 10 mm rod
from below, and the rod's side arms pass outside the 240 mm flap (test). What blocks the fit is the bag's front: shut,
the flap's lip lies inside the space the hung bag's front fills (test), and pushed from the robot's side the flap only
presses on its shut stop. Carried straight in with the flap shut, the bag's front meets the flap (stem `s_one_arm`, 0).

The manual's method holds the flap up with one hand. A one-hand route also exists in the geometry (stem `s_lift`):
tip the bag 20 deg nose-down, take the rod in under the lip into the channel (1 mm off the hooks' lips, 4 mm off the
plate), and rise, so the bag's cover lifts the lip while the bag levels, then lower the rod into the hooks. A 2D
sweep of that path finds the flap riding up on the bag and resting on it at the end, never past its stop [RUN]; it
needs a lift of roughly 10 to 50 N through the bag [EST: the flap's 0.20 to 0.37 N m closing torque over levers of
7.5 to 23 mm read from the sweep's angle steps]. Whether PhysX lets it
happen is what the stem's run shows. If it reaches 1.0, it is a correct fit, not a cheat: the score reads the end
state only, and the task is then a manual-method task that one hand can also do.

## Scoring (`score()`, the last step only; `success()` must hold 1 s)

Everything is measured in the mower's own frame, so rolling the mower never changes a fit.

Rod seated in both hooks. For each hook, three stations across it: its two sides, each widened 5 mm, and its middle.
Each station must lie on the rod's actual 300 mm length (the rod is a segment, never its axis extended past an end),
and the rod's centre there must be inside that hook's slot (between the lip and the rear face, 2 mm allowed on each
wall for the colliders' contact offset) and resting on the floor: from 0.5 mm below to 1.5 mm above where a rod lying
on the floor has its centre. A rod whose end stops inside or short of a hook fails that hook (stem `s_one_hook` and a
test that shows the extended axis would have passed). A rod turned more than 60 deg from the hooks' line fails both,
and the bag must be upright (its cover within 30 deg of level), so an upside-down bag never counts (test).

- 0: not seated in both hooks.
- 0.25: seated.
- 0.5: and no point of either gripper (finger slices and housing, the same point model as the clearance sweep) within
  30 mm of the bag, and the bag still: under 5 mm/s and under 0.05 rad/s.
- 0.75: and the flap's free edge resting on the bag: three points along it, at y -100, 0 and 100 mm, over the bag's
  cover, each from 3 mm below to 5 mm above it.
- 0.9: and no point of either gripper within 30 mm of the flap, the flap under 0.05 rad/s, and the mower safely on the
  bench: rolled no more than 60 mm forward from its start, so its front wheels stay 20 mm or more inside the bench
  edge [CHOICE: a safety margin; the slide's stop is at the edge itself].
- 1.0: 0.9 held 1 s (yam_arena ends the episode with `success`).

Small rolls cost nothing: any roll backward, and up to 60 mm forward, leaves the score as it is (a nudge while fitting
the bag is normal). There is no history: a hand or a roll earlier in the run does not lower the end score (test). The
only memory is the 1 s hold. Hands that cannot be read count as touching (at most 0.25). Nothing in the scene can
break.

Real trial measurement:

- Seated: at each hook, a 1.5 mm feeler stack (1.0 + 0.5) must not pass under the rod on the hook's floor, and the
  rod must sit between the lip and the rear face; a steel rule shows the rod running at least 5 mm past each side of
  each hook.
- Flap on the bag: a 5 mm stack (1.0 + 0.95 + 0.9 + 0.85 + 0.8 + 0.5) must not pass under the flap's edge at its
  middle and 100 mm either side.
- Hands: a tape for the 30 mm, from any part of each gripper.
- Mower on the bench: tape on the bench at the front wheels' start and a second tape 60 mm ahead of it; the front
  wheels must not cross the second tape.
- Stillness: a 60 fps video over a 1 mm grid.

## Building the real trial

- An MJ401E (or the same 14 in corded model), unplugged, its blade removed, standing on a bench with its wheels free
  to roll, its handle folded forward, its front wheels 80 mm from the bench's far edge. Its OEM grass bag (spare part
  MJ401E-10, Sources). Before any trial, measure and record: the hooks' height, spacing, width and slot; the flap's
  size and the torque to hold it at 90 deg (a force gauge at its edge); the bag's size, rod, handle and mass; the
  force to start the mower rolling. The sim's [EST] values are replaced by these before results are compared.
- Lower-cost stand-in, if no MJ401E is at hand: an OEM MJ401E-10 bag on its own support rod, and a rear deck of 18 mm
  plywood built to the sim's dimensions: a 260 x 380 mm rear face, two 16 mm wide steel hooks (2 mm steel strip bent
  to a 25 mm floor and a 20 mm lip, 21 mm slot) at 285 mm and 160 mm apart, a 240 x 100 x 4 mm plastic plate on a piano
  hinge at 355 mm with a torsion spring set to 0.20 N m shut (checked with a force gauge: 2.0 N at 100 mm), the deck
  weighted to 13.2 kg on four 140 mm wheels. Prices not verified: the product pages could not be fetched from this
  machine, so no cost is claimed.
- Safety: no power cord connected, no blade fitted. The only stored energy is the flap's small spring. The front
  wheels start 80 mm from the bench edge and the score's limit keeps them 20 mm inside it.

## Run

    PYTHONPATH=<yam-isaac-lab-arena checkout> python3 test_mower_bag.py
    PYTHONPATH=<yam-isaac-lab-arena checkout> python3 tools/ikcheck.py      # ends TOTAL BAD 0
    PYTHONPATH=<yam-isaac-lab-arena checkout> python3 tools/clearance.py    # ends CLEARANCE BAD 0
    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=mower_bag.py -E object_jitter=0 \
        -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 --seed 0 \
        --instruction "$(python3 -c 'import json;print(json.load(open("task.json"))["instruction"])')"

The seed is the one the reset receives; it sets the bag's start (numpy `default_rng(seed)`). The sim scripts read
`ARENA_ENV` (the simulator's environment file) and `LOCKS`/`LOCK_BASE` (the GPU lock files) from the environment.

The scripted demo is privileged: it reads the seed and the bag's and mower's true poses. The score never uses them.
Its grips are pitched, not straight down: each waypoint's yaw carries a full TCP rotation, which the task file hands
to yam_scripted's planner (it changes only the demo policy, never the scene or the score).

1. Left: beside the flap's left edge, approach along -y, pinch the plate 50 mm from the hinge, swing it up to 110 deg
   in 10 deg steps, hold; then move the later waypoints by the mower's measured roll (at most 25 mm).
2. Right: from behind, round the bag's bar (approach 70 deg from vertical), grip, lift 10 mm, wait for the flap,
   slide in under it, lift over the lips (the demo's margin: the rod's underside 12 mm over them), carry over the
   slots, lower 20 mm past the seat so the hooks take the bag, set the tail down on the bench, let go, slide off the
   bar, draw back. Four line-up passes (behind the hooks, again behind them before the carry, over the slots, again)
   correct the rod's pose, each at most 25 mm and 6 deg (the two behind the hooks up to 40 mm in height); one more
   aims the hand where it is before it opens.

Load compensation. The arm holds each joint on a PD spring (80 N m/rad at joints 1 to 3, 10 at 4 to 6, from
yam_arena's constants) and its gravity compensation covers the arm alone, so the 1 kg bag bends the hand away from
the plan. The traced PhysX run of commit 704ea7f measured the TCP 6.6 mm back, 11.9 mm left and 45.4 mm low at the
line-up behind the hooks [RUN]; the spring model (`tools/sagcheck.py`, static balance with the full kinematics)
gives 6.5, 11.4 and 45.4 mm [ARITH]. Without compensation the rod met the lips from the side and never went over
them. The demo therefore commands each loaded waypoint (lift to the line-ups) at q - K^-1 J^T w, the joint angles
the weight bends onto the planned pose; the model puts the hand within 1.22 mm and 0.68 deg of the plan on every
loaded stretch (5 Hz) of seeds 0 to 9 and every stem. The lowering and what follows are commanded as planned, so the
hooks take the bag. Plans, tests and the clearance sweep keep the planned poses; `tools/ikcheck.py` checks the
commanded ones.
3. Left: lower the flap to 30 deg over its resting angle, let go (it closes the rest at about 4 rad/s), draw out
   sideways, back off.

Demo length 35.7 s; success expected about 1 s later, inside the 60 s run limit (test).

Offline checks [RUN]: IK error at most 0.10 mm on every waypoint near a part and 1.7 mm in free air (the left wrist
sits on its stop beside the flap), for seeds 0 to 9 and every stem. Gripper sweep (the scorer's finger and housing
points along the planned paths; for `s_nudge` and `s_shove` the mower rolls with the push): least clearance 3.5 mm
(right fingers sliding off the bar over the cover), 3.7 mm (left fingers over the bag's rod end while raising the
flap), 9 mm (left fingertips to the hook), 7 mm (left hand to the flap after letting go); the bag clears the opening
flap by 31.7 mm and the lips by 32.7 mm, also with the pinched flap 10 deg under its plan (the traced run held it
at 101 to 103 deg of 110). At the demo's end the whole left gripper is 79 mm from the bag and 42 mm from
the flap, the right 73 mm from the bag. Let go, the flap falls onto the left's lower finger and stays on it until the
hand slides out sideways (a contact the sweep leaves out).

Stems: `bash sim/mk.sh` writes traced copies to `sim/arm/` (traces to `sim/out/`); `bash sim/run.sh <stem> <steps>
<lock> <seed>` runs one.

| Stem | What it does | Expected |
|---|---|---|
| `t00` | the demo | 1.0 |
| `s_nothing` | both arms stay put | 0 |
| `s_one_arm` | the right hand alone carries the bag in, flap shut | 0 |
| `s_lift` | the right hand alone tips the bag under the flap's lip, lifts the flap with it and hangs it | the sim decides (1.0 if it works) |
| `s_one_hook` | the bag 65 mm to the right: the rod's left end stops inside the left hook | 0 |
| `s_tips` | the rod set on the lips' tops and held there | 0 |
| `s_ground` | the bag let go short of the hooks, onto the bench | 0 |
| `s_hold` | the demo, but the right hand keeps the bag | 0.25 |
| `s_prop` | the demo, but the left hand keeps the flap up | 0.5 |
| `s_pinch` | the right hand parks under the flap's path and the flap is let go onto it | 0.5 |
| `s_slam` | the flap let go fully open, straight onto the bag | 1.0 if the rod stays seated [EST] |
| `s_nudge` | before letting go, the bag rolls the mower 8 mm (the left hand moves with the flap) | 1.0 |
| `s_shove` | before letting go, the bag rolls the mower 70 mm (the left hand moves with the flap) | 0.75 |

## Closest tasks

Checked on the repository (state on 2026-11-19):

| Task | State | What it shares | What differs here |
|---|---|---|---|
| #2282, hand a drink out a self-closing drive-thru window | merged | hold a self-closing part open with one hand, place an object with the other, let it shut | here the closing part must come to rest on the placed object and hold it; the object is hung by a rod in two slotted hooks, not set on a shelf |
| #2385, hang a dish-rack cutlery caddy by its two hooks | not accepted, final (overlap with existing hanging tasks) | an object hung by two hooks | here the hooks sit behind a sprung flap that must be held up, and the flap must then close onto the bag |
| hang-cable-between-hooks | accepted | laying a long part across two hooks | a rigid 1 kg bag's rod into two slots, behind a flap |
| hang-a-blind-headrail-in-gated-brackets | accepted | a rail seated in two brackets | no flap to hold; the brackets' gates are part of the brackets |
| refit-drawer-past-rear-hooks | accepted | carrying a load past hooks | a drawer on slides; no sprung part to hold |
| #1277 (self-closing gate), #1699 (piano lid and prop), #1676 (trailer tongue), #1403 (spring flange) | not accepted, final | hold a sprung or heavy part open while the other hand works | none of them seats a part in hooks under the held part or has it close onto the part |
| #2489, hang a mower grass bag under its sprung flap | not accepted, its hold lapses 2026-11-21 | the same idea | the reviewer's two points are answered by construction: the finite rod is checked at each hook, and the mower's roll is a final, stated safety limit, not a permanent zero |

Risk, stated: #2385 was closed for overlap with hanging tasks, and #2282 already covers hold-open, place, let shut.
This task's case rests on the combination, the flap closing onto the hung bag as the manual's fitting step requires.

## Sim results

Not yet run in the sim. The first traced run of the demo decides the open PhysX questions: the pinch on the 4 mm
plate while the wrist rolls it to 110 deg, the bar grip under 1 kg, the rod settling into the slots, and the roll
joint's friction (Isaac Lab takes it as an effort; if it acts as a coefficient instead, the mower will not roll in
`s_nudge` and `s_shove`).

## Reset

Nothing consumed: lift the bag off the hooks, close the flap, roll the mower back to its mark (front wheels 80 mm from
the bench edge), stand the bag on its taped outline. Then set the trial's seeded start: 0 to 10 mm back and up to 15 mm
sideways from the outline (a steel rule), turned up to 3 deg about the rod's centre (a protractor on the outline's
front edge); `bag_layout(seed)` in `mower_bag.py` gives the three values.

## Licence

MIT. `mechanism.py` is adapted from robocurve/benchmark-tasks #223 under its MIT licence (header in the file).
