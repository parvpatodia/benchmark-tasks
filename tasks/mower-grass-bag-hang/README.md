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

Bagging mowers are fitted and refitted every time the bag is emptied. The step is two-handed by design: the flap is
sprung shut and hangs in front of the hooks, so one hand holds it open while the other sets a 10 mm rod into two
21 mm slots 160 mm apart, with the bag's 1 kg hanging from the hand. Then the flap is let down onto the bag, which is
what holds the bag in place while mowing.

## Sources

Every number in the code carries a marker: `[SOURCE]` quoted here, `[ARITH]` derived, `[EST]` estimate, `[CHOICE]`
design choice, `[RUN]` computed by a tool. Numbers with none are layout and timing choices of the scripted demo.

| What | Source and quote | Used for |
|---|---|---|
| Mower | Sun Joe MJ401E operator's manual, Technical Data: "Weight: 29 lbs (13.2 kg)", "Wheel Size: 5.5 in. (14 cm)", "Cutting Width: 14 in. (35.6 cm)", "Grass Collection Bag Capacity: 10.6 gal (40 l)"; parts "11. Ejector flap", "12. Grass bag" | mass, wheels, bag volume |
| Fitting the bag | Same manual, step 3: "Raise the spring-loaded ejector flap and move the grass bag up to the mower body. Fasten it onto the coupling hooks (Fig. 6)."; step 4: "Release the ejector flap to secure the grass bag in position (Fig. 7)." | the instruction and every rung |
| Feeler gauge | Starrett 66MA: "THICKNESS GAGE, 20 LEAVES, 0.05-1.0mm", leaves "0.05, 0.10, ... 0.50, ... 1.0" | the 1.5 mm and 5 mm checks |

The manual's lines were read through a fetch tool from manualslib's page for the manual (its raw pages refuse
scripted downloads), so they are second-hand until the PDF is read directly. Not sourced, all [EST] and measured on
the real mower before trials (below): the hooks' size, height and spacing, the flap's size, spring and hinge, the bag's
shape, rod, handle and masses, all friction values, the mower's rolling resistance.

## The scene (world: x forward from the arm bases, y left, z up from the bench)

- Mower: rear face at x 0.53. Rear housing 60 mm deep, y +-130 mm, up to 400 mm; deck 320 mm long, y +-180 mm; motor
  cover; four 140 mm wheels. It rides a damped fore-aft slide (40 N s/m [EST], +-150 mm) with its wheels 0.5 mm off
  the bench: the arms can roll a 13.2 kg mower but not slide it sideways (about 65 N at 0.5 friction [ARITH]) or tip
  it, so only the roll is modelled.
- Hooks: two steel hooks at y +-80 mm, 16 mm wide. Each is a floor plate 25 mm deep standing out of the rear face, top
  at 285 mm, with a 20 mm lip at its back: a 21 mm slot, so a 10 mm rod has 5.5 mm of play a side [ARITH]. The rod
  must lift 12 mm over the lips to go in [ARITH].
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

## Why two arms

The shut flap stands between the robot and the hooks, across both hooks and over the whole height the rod must pass
(test), and it closes from any angle. Pushed from the robot's side it only presses on its shut stop. So one hand
holds it open while the other carries the bag in: stem `s_one_arm` carries the bag straight in with the flap shut, and
the rod stops on the flap (0). Not ruled out, and untested: one hand tipping the bag nose-down so its rod passes under
the flap's lip and lifts it. If that ends in the scored state it is a correct fit, not a cheat.

## Scoring (`score()`, the last step only; `success()` must hold 1 s)

Everything is measured in the mower's own frame, so rolling the mower never changes a fit.

Rod seated in both hooks. For each hook, three stations across it: its two sides, each widened 5 mm, and its middle.
Each station must lie on the rod's actual 300 mm length (the rod is a segment, never its axis extended past an end),
and the rod's centre there must be inside that hook's slot (between the lip and the rear face) and resting on the
floor: from 0.5 mm below to 1.5 mm above where a rod lying on the floor has its centre. A rod whose end stops inside or
short of a hook fails that hook (stem `s_one_hook` and a test that shows the extended axis would have passed).

- 0: not seated in both hooks.
- 0.25: seated.
- 0.5: and no gripper pad within 30 mm of the bag, the bag under 5 mm/s.
- 0.75: and the flap's free edge resting on the bag: three points along it, at y -100, 0 and 100 mm, over the bag's
  cover and each within 5 mm above it.
- 0.9: and no pad within 30 mm of the flap, the flap under 0.05 rad/s, the mower within 25 mm of its start mark.
- 1.0: 0.9 held 1 s (yam_arena ends the episode with `success`).

The mower is free to roll up to 25 mm without any cost (a nudge while fitting the bag is normal); a roll of more than
25 mm caps the score at 0.75. There is no history: a hand or a roll earlier in the run does not lower the end score
(test). The only memory is the 1 s hold. Hands that cannot be read count as touching. Nothing in the scene can break.

Real trial measurement:

- Seated: at each hook, a 1.5 mm feeler stack (1.0 + 0.5) must not pass under the rod on the hook's floor, and the
  rod must sit between the lip and the rear face; a steel rule shows the rod running at least 5 mm past each side of
  each hook.
- Flap on the bag: a 5 mm stack (1.0 + 0.95 + 0.9 + 0.85 + 0.8 + 0.5) must not pass under the flap's edge at its
  middle and 100 mm either side.
- Hands: a tape for the 30 mm. Mower mark: tape on the bench at the rear wheels; a steel rule for the 25 mm.
- Stillness: a 60 fps video over a 1 mm grid.

## Building the real trial

- An MJ401E (or the same 14 in corded model), unplugged, its blade removed, standing on a bench with its wheels free
  to roll, its handle folded forward. Its OEM grass bag. Before any trial, measure and record: the hooks' height,
  spacing, width and slot; the flap's size and the torque to hold it at 90 deg (a force gauge at its edge); the bag's
  size, rod, handle and mass. The sim's [EST] values are replaced by these before results are compared.
- If no MJ401E is at hand, a fixture with the sim's dimensions: two 16 mm steel hooks with a 25 mm floor and a 20 mm
  lip on a 13.2 kg wheeled base, a 240 x 100 mm plate on a sprung hinge (0.20 N m shut), and a 40 l bag on a 10 mm
  steel rod with a bar handle.
- Safety: no power cord connected, no blade fitted. The only stored energy is the flap's small spring.

## Run

    PYTHONPATH=<yam-isaac-lab-arena checkout> python3 test_mower_bag.py
    PYTHONPATH=<yam-isaac-lab-arena checkout> python3 tools/ikcheck.py      # ends TOTAL BAD 0
    PYTHONPATH=<yam-isaac-lab-arena checkout> python3 tools/clearance.py    # ends CLEARANCE BAD 0
    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=mower_bag.py -E object_jitter=0 \
        -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 --seed 0 \
        --instruction "$(python3 -c 'import json;print(json.load(open("task.json"))["instruction"])')"

The seed is the one the reset receives; it sets the bag's start (numpy `default_rng(seed)`).

The scripted demo is privileged: it reads the seed and the bag's and mower's true poses. The score never uses them.
Its grips are pitched, not straight down: each waypoint's yaw carries a full TCP rotation, which the task file hands
to yam_scripted's planner (it changes only the demo policy, never the scene or the score).

1. Left: beside the flap's left edge, approach along -y, pinch the plate 50 mm from the hinge, swing it up to 110 deg
   in 10 deg steps, hold.
2. Right: from behind, round the bag's bar (approach 70 deg from vertical), grip, lift 10 mm, wait for the flap,
   slide in under it, lift 12 mm over the lips, carry over the slots, lower 20 mm past the seat (more than the arm's
   sag under the bag), set the tail down on the bench, let go, slide off the bar, draw back. Three line-up passes
   (behind the hooks, over the slots, again) correct the rod's pose, each at most 25 mm and 6 deg; one more aims the
   hand where it is before it opens.
3. Left: lower the flap to 30 deg over its resting angle, let go (it closes the rest at about 4 rad/s), draw out
   sideways, back off.

Demo length 34.5 s; success expected about 1 s later, inside the 60 s run limit (test).

Offline checks [RUN]: IK error at most 0.10 mm on every waypoint near a part and 1.7 mm in free air (the left wrist
sits on its stop beside the flap), for seeds 0 to 9 and every stem. Gripper sweep (finger slices and housing as
points along the planned paths): least clearance 3.5 mm (right fingers sliding off the bar over the cover), 3.7 mm (left
fingers over the bag's rod end while raising the flap), 9 mm (left fingertips to the hook), 7 mm (left hand to the
flap after letting go); the bag clears the opening flap by 31.7 mm and the lips by 32.7 mm. Let go, the flap falls
onto the left's lower finger and stays on it until the hand slides out sideways (a contact the sweep leaves out).

Stems: `bash sim/mk.sh` writes traced copies to `sim/arm/` (traces to `sim/out/`); `bash sim/run.sh <stem> <steps>
<lock> <seed>` runs one.

| Stem | What it does | Expected |
|---|---|---|
| `t00` | the demo | 1.0 |
| `s_nothing` | both arms stay put | 0 |
| `s_one_arm` | the right hand alone carries the bag in, flap shut | 0 |
| `s_one_hook` | the bag 65 mm to the right: the rod's left end stops inside the left hook | 0 |
| `s_tips` | the rod set on the lips' tops and held there | 0 |
| `s_ground` | the bag let go short of the hooks, onto the bench | 0 |
| `s_hold` | the demo, but the right hand keeps the bag | 0.25 |
| `s_prop` | the demo, but the left hand keeps the flap up | 0.5 |
| `s_pinch` | the right hand parks under the flap's path and the flap is let go onto it | 0.5 |
| `s_slam` | the flap let go fully open, straight onto the bag | 1.0 if the rod stays seated [EST] |
| `s_nudge` | before letting go, the bag rolls the mower 8 mm | 1.0 |
| `s_shove` | before letting go, the bag rolls the mower 60 mm | 0.75 |

## Sim results

Not yet run in the sim. The first traced run of the demo decides the open PhysX questions: the pinch on the 4 mm
plate, the bar grip under 1 kg, and the rod settling into the slots.

## Reset

Nothing consumed: lift the bag off the hooks, close the flap, roll the mower back to its mark, stand the bag on its
taped outline.

## Licence

MIT. `mechanism.py` is adapted from robocurve/benchmark-tasks #223 under its MIT licence (header in the file).
