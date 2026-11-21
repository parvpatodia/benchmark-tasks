# Return a shopping trolley: push the chain key of the trolley in front home in your coin lock, then take the quarter back

Two nested shopping trolleys, shown by their push handles and the side tubes under them: the returned (near) trolley's
handle, and 178 mm further on the handle of the trolley in front. Each handle carries a coin lock. The front lock's key
hangs free on its 220 mm chain between the two handles. The robot takes the hanging key, turns it level, pushes it into
the near lock's key slot until the lock lets its coin slide out, takes the quarter from the slide, lays it on a mat, and
takes its hands away.

## What it tests

A part that hangs free on a chain: it swings, twists and moves away when touched, so it has to be gripped from the side
(the palm cannot come down on the chain) and turned 90 degrees before it lines up with a 21 x 4.5 mm slot. An access
problem: the key hangs between two handles, two locks and four inclined side tubes, and the slot faces the front
trolley, so the hand works in a gap. An insertion with a threshold: the lock only releases the coin when the key is
pushed 12.5 mm in, past the plunger's spring, the latch and the retaining pin; a key held short scores 0.25 (mode
partway). Then a small coin taken from a slide and laid flat on a mat.

Why it matters (Societal relatability): coin-lock trolleys are returned this way at supermarkets; the coin is the
deposit the shopper gets back. The mechanism is that of EP0427819B1 (`src/ep0427819.txt`): the coupling member goes in
through an opening in a side wall, the coin slide runs at right angles to it, and on full insertion the coin slide is
released and a compression spring pushes it out "so that the coin can be removed".

## The parts (every number in `trolley.py` carries its basis)

| Part | In the model | Basis |
|---|---|---|
| Nesting gap | 178 mm handle to handle | [SOURCE] `src/unarco_M60TN.txt`: "Nesting Distance 7"" |
| Handles | steel tube 25 mm across, 300 mm along y, axis 0.34 m over the bench, side tubes 80 mm forward to the bench | lock fits "25-27mm Trolley Handle" [SOURCE] `src/supermarket_fixture_coinlock.txt`; length, height and slope [CHOICE] for the arms' reach (real handle "373/4"" over the floor [SOURCE] `src/unarco_M60TN.txt`) |
| Coin lock | 90 x 70 x 68 mm plastic, red top | [SOURCE] `src/supermarket_fixture_coinlock.txt`: "Size:90*70*68mm", "Material:Plastic" |
| Chain and key | 11 steel links of 20 mm (220 mm); key 62 x 15 x 3 mm steel, 22 g, a 9 mm wide hole 6 to 14 mm behind its tip | listing "260mm stainless steel chain" [SOURCE]; 220 mm and key size [CHOICE] so the key's tip hangs 66.5 mm over the bench |
| Key slot | 21 x 4.5 mm in the near lock's front face | [CHOICE] 3 and 0.75 mm play a side on the key |
| Retaining pin | 5 x 2 mm, sprung down (50 N/m), 4.5 mm travel, its low edge 2.5 mm into the key | stands in for the patent's "ball of e.g. 7 mm in diameter" [SOURCE] `src/ep0427819.txt`; size [CHOICE] |
| Basket rear wall | near trolley only: steel wire grid under the handle, 30 mm ahead of its axis, rails 70, 185 and 300 mm under it, 8 upright wires 40 mm apart | [CHOICE] and [EST]; the child seat and the front basket are left out |
| Quarter | 24.26 mm, 1.75 mm thick, 5.67 g | [SOURCE] `src/usmint_quarter.txt` |
| Mat | rubber, 100 x 100 x 3 mm | [CHOICE] |

## How it is modelled

Three articulations in PhysX (`mechanism.py`) and two loose bodies. The front trolley (frame and lock as base) carries
its chain: 11 capsule links on hinges whose axes alternate across and along the handle, then the key on a hinge. The
near trolley (frame, basket wall and lock casing as base) has three moving links in the lock: a plunger behind the slot
(spring, 6 mm travel) with a tooth T, the coin slide (spring toward -y, 40 mm travel) carrying the quarter's pocket and
a lug L, and the retaining pin. The slide's spring holds L against T. The key's tip meets the plunger 8 mm in; 4.5 mm
further takes T past L, the slide runs out of the casing's -y end, and a rail behind L lies across T's way back, so the
plunger stays in and the slide out. The near lock's own chain and key are a third articulation (hanging behind, unused).
The quarter stands on edge in its pocket (walls on both faces, end stops under its rim); the mat is a box.

Key retention follows the patent: the ball "drops downwards under the action of gravity, so that it occupies the
apertures 19 and 21" (one in the casing over the key, one an "elongate hole" in the key) and "the coupling member 8 is
firmly locked". Here a pin over the key's path, sprung down, has a +x face that slopes (the incoming key lifts it) and an
upright -x face; at 12.5 mm the key's hole comes under it, it drops in, and a pull on the key stops on that face. The
9 mm hole catches the 2 mm pin for any key within 3.5 mm of the slot's centre line, more than the key's 3 mm play. The
near basket's rear wall hangs under its handle, between it and the hanging key: the hand reaches over it.

Simplified: the tooth and rail replace the patent's coin-slide locking member and the pin its ball; the pin's release by
the re-inserted slide is not modelled. The chain has no twist joint. Handles shortened and lowered (see the table).
Forces are small. Release takes the plunger spring (150 N/m, about 0.8 N at release [ARITH]) plus friction of L on T
under the slide spring (30 N/m, 2.1 N held in [ARITH]). The key is held at 6 N a finger, the quarter at 4 N.

Closed shortcuts: a finger goes at most 10 mm into the slot, 2 mm short of release (tested on the open finger's
cross-section); the lug stops the slide being pried out, and nothing scores without the key in line in the slot; 0.5 and
up need the pin down in the key's hole, so a key held short by the hand does not count. The near lock's own key can
reach round to its own slot (a real lock prevents this), but only the front lock's key scores, so that scores 0. A key
held in line behind the near lock (tip more than 20 mm past the mouth; the plunger stops it at 14 mm) scores 0.

## Scoring

From the end state. `score()` tops out at 0.75; 1.0 comes only from `success()` held `SUCCESS_HOLD_S` (1 s).

- 0.25: the front lock's key in the near lock's slot, in line (tip within 4 mm across and 2.5 mm up or down of the slot
  centre, key within 15 degrees of the slot axis, tip first) and 5 to 20 mm past the mouth.
- 0.5: also the coin slide out 35 mm or more of its 40 mm travel, and the key held by the lock (tip 11.5 mm or more in,
  retaining pin within 1.5 mm of its rest: in the key's hole, 1 mm or more into its thickness). The slide comes out only
  at 12.5 mm of key depth.
- 0.75: also the quarter lying flat on the mat (faces within 15 degrees of level, centre 5 mm or more inside its edges,
  lowest point within 3 mm of its top). A quarter on edge does not count.
- 1.0: also quarter, key, plunger and slide at rest (under 0.01 m/s, 0.5 rad/s) and every gripper point 30 mm or more
  from the key and the quarter, held 1 s.

Real trial, on the printed lock: a ruler on the slide's travel and the key's exposed length (49.5 mm at 12.5 mm in),
a 1 N pull on the chain for the retention, a photo of the mat and hands. The lock (`cad/coinlock.scad`, STLs in
`cad/stl/`, key `cad/key.dxf`, `cad/drawing.svg`) uses the sim's numbers under the sim's names; class `Cad` in
`test_trolley.py` checks all 69 against `trolley.py` and the moving parts against the sim's shapes. Bought parts
`src/hardware_parts.txt`; parts list, print, assembly, checks and the differences (spring rates, a reset release
hole): `cad/BUILD.md`.

## Modes

`TASK_DEMO=<mode>` picks a scripted plan (motionkit waypoints; scene constants and the arms' joint readings only).

| Mode | Expected | What it does |
|---|---|---|
| demo | 1.0 | right hand grips the hanging key from the side, lifts and rolls it level, pushes it home, lets go, takes the quarter, tips it 55 degrees and lets it fall flat on the mat |
| nothing | 0 | arms stay in the ready pose |
| partway | 0.25 | the key pushed only 10 mm in and held there: the slide stays in |
| key_only | 0.5 | key home, the quarter left in the slide |
| drop_coin | 0.5 | the quarter let go 40 mm up beside the mat, on the bench |
| hold_coin | 0.5 | the quarter held 30 mm over the mat to the end |
| pry_coin | 0 | no key: closed fingers push and drag at the slide's exposed end |
| no_key_coin | 0 | no key: the hand reaches for where the quarter would come out |

## Results

Run `result.json` files on hash 527c77b5 (sha256 of the sorted `*.py` plus `task.json`, first 8 hex), runs
fast_527c77b5_1121_125145 and _130816, cameras off, end states from the last trace row. Since then: a 20 mm depth cap
(the plunger stops the key at 14 mm, so the scores stand [ARITH, not re-run]), CAD tests and text; hash 09f8f91a.

| Mode | Seed | Score | Steps | End |
|---|---|---|---|---|
| demo | 0 | 1.0 | 682 | success held 1 s; key 12.7 mm in, pin 0.6 mm up, slide out 40 mm, quarter flat on the mat |
| demo | 1, 2 | 1.0 each | 682 | the same |
| key_only | 0 | 0.5 | 501 | key 12.7 mm in, pin 1.0 mm up, slide out 40 mm, quarter left in the slide |
| drop_coin | 0 | 0.5 | 736 | quarter on the bench beside the mat |
| hold_coin | 0 | 0.5 | 760 | quarter still in the hand, its centre 37 mm over the mat |
| partway | 0 | 0.25 | 439 | key 9.7 mm in, held; slide in, pin riding on the key (3.0 mm up) |
| pry_coin | 0 | 0 | 191 | slide in, key hanging |
| no_key_coin | 0 | 0 | 173 | slide in, key hanging |
| nothing | 0 | 0 | 61 | |

In the demo's last second the key's tip and back end move at most 4.6 mm/s and it turns at most 0.14 rad/s (the 1.0
check is 10 mm/s and 0.5 rad/s). The pin holds it at 12.7 mm against the chain's pull.

### Dream-1

Dream-1 ran once on hash eba907ea: 0 at the 1,600-step limit; it never reached the lock. That start scene had no
basket wall, a 6 mm key hole and a 4 mm pin, so the score is for eba907ea, not this scene.

`checkkit.py` (IK with 3 degree joint margins, full-link clearance, run limit) and `test_trolley.py` (33 tests: every
scoring rule, each mode's end state, latch and retention geometry, the finger, the text, the CAD against the scene) pass.

## Run it

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=trolley.py -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \
      --scorer task_score --max-steps 1600

One fixed layout; the demo is open loop. `fastrun.py` runs every mode without cameras; `PYTHONPATH=<yam_arena
source> python3 test_trolley.py` runs the offline tests; `cad/make.sh` re-exports the CAD.
`sim/film.sh` runs the stock filmed demo and, via env `TROLLEY_HD` (never set when scoring), the same run's cameras at
1280 x 720; demo.mp4 is that top view with the right wrist camera inset, `setup.png` its first frame.

## Assets and licences

Task scene: primitive shapes and hulls made in code; `mechanism.py`, `motionkit.py`, `checkkit.py`, `fastrun.py` carry
MIT notices. Robot (not bundled): the YAM arm and gripper (URDF, meshes, hulls) in the SDK's `yam_arena/assets/yam/`,
from i2rt (https://github.com/i2rt-robotics/i2rt, commit 120c3c8), MIT, Copyright (c) I2RT Robotics (`LICENSE.i2rt`),
and its `dream/` link2/link3 colour split (SDK `dream/SOURCE`: "MIT, see ../LICENSE.i2rt"). The SDK's room, lights and
cameras are code; its README says it has no license yet (ported from YAM-inspect-Isaac). `cad/`: made for this task,
MIT. `src/`: the patent text, the coin lock listing (supermarket-fixture.com), the cart sheet (text plus gzipped PDF),
the US Mint coin specifications and the bought parts' catalogue values.
