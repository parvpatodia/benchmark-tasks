# Load a film pack into an instant camera

An instax mini 12 instant camera stands upright on the table, empty, its back towards the right arm and its lens
towards the left. A new film pack lies flat beside it. The instruction asks for the pack to be loaded and the camera
made ready to shoot. That is the camera's own loading sequence, each step a different kind of contact:
1. Press the back cover lock down: the back cover springs ajar.
2. Swing the cover open round its hinge.
3. Turn the pack so its yellow mark is at the camera's yellow mark (yellow end up, the back with the two holes facing
   out), set its bottom edge into the chamber tilted out at the top, let it drop in and press it flat, never touching
   the two holes.
4. Swing the cover shut and press it until it locks.
5. Grip the lens ring and turn it to ON.
6. Press the shutter button once: the black film cover comes out of the slot on top.

A pack in upside down or back to front, a cover that isn't locked, or a press on the pack's holes spoils the film; a
shutter press on a camera that isn't loaded right ejects nothing.

## The real step
Fujifilm's instax mini 12 User's Guide (FI020-EN-WW-01, pages 3-4, drawn steps; part names on page 6: "Back cover
lock", "Back cover", "Lens ring / Power", "Shutter button", "Film ejection slot"):
1. Slide the back cover lock down to open the back cover.
2. Insert the film pack, aligning the yellow marks.
3. Close the back cover.
4. Turn the camera on by rotating the lens ring to ON.
5. Press the shutter button: the film cover is ejected.

Fujifilm's instax printer manuals add, for the same mini film pack: "When you load the film pack, never press the two
rectangular holes on the back of the film pack."

Done by: anyone who owns an instant camera, at a party, a wedding or at home; the same step comes with every new
pack (10 shots). What goes wrong: fogged or jammed film, a wasted pack. A photo of the camera shows whether the film
cover came out.

## The camera, the pack, and what was simplified
| Size | Value used | Source |
|---|---|---|
| camera body | 104 x 66.6 x 122 mm (W x D x H) | Fujifilm's instax mini 12 specifications page (306 g without batteries, strap and film) |
| film pack | 92 x 61 x 20 mm, 45 g | size: Wikipedia's Instax article, mini specifications table, "Film pack size (W×H×D) 61 mm × 92 mm × 20 mm" (fetched 2026-11-23; a secondary source); mass: **estimate** |
| film inside the pack | 86 x 54 mm per sheet | Fujifilm's instax mini Link specifications page, "Film size: 86 mm x 54 mm" (fetched 2026-11-23); shown only as a check that a 92 x 61 mm pack is plausible |
| film chamber | 67 x 98 mm, 20.5 mm deep, floor 11 mm up; solid walls on all four sides | **estimate**: the pack plus 3 mm of play each side (width and height) and 0.5 mm of depth; no camera was measured |
| back cover | 3 mm plate, 30 g, hinged on its far edge, a stop at 95 degrees, springing 20 degrees ajar when unlocked | **estimate** from the guide's drawings (which warn against opening it further) |
| back cover lock | 12 x 12 x 7 mm tab, 3 mm travel, unlocks at 2 mm | **estimate** |
| lens ring | 62 mm across, 22 mm deep; ON at 30 degrees or more | **estimate** |
| shutter button | 18 mm across, 4 mm proud, 2.5 mm travel | **estimate** |
| pack's holes | two 20 x 10 mm holes, either side of the back's middle | **estimate** |

How the sim models it:
- The camera is a fixed articulation, built with `mechanism.py` (MIT, from robocurve#223; changes listed at its top):
  - fixed boxes and cylinders for the body, the rear frame round the chamber, the chamber's floor and the marks;
  - the back cover on a revolute joint with a spring whose target the task file sets every control step: locked, the
    target is 0 (held shut, 20 N m/rad); once the lock tab is 2 mm down, the target is 20 degrees (it springs ajar),
    then follows the cover wherever it is put; pushed back within 2 degrees with the tab up, it relocks;
  - the lock tab and the shutter button on sprung prismatic joints; the lens ring on a revolute joint with friction;
  - the film cover, a thin black plate inside the body on a prismatic joint, driven 45 mm up out of the slot by a
    shutter press made while the camera is ready.
- The pack is one loose rigid box at its true size; its holes and yellow mark are drawn on its back, not cut.

What we simplified:
- The camera is fixed to the table; a person holds it in one hand. Real trials: fix it with double-sided tape.
- The film cover is ejected by a joint, not a motor and rollers; nothing is exposed or printed.
- The pack is a plain box: the real pack's shape keys it, so in a real camera a wrong-way pack sits proud and the cover
  won't shut; here it fits and is scored 0.
- No camera was measured for this version (we have none to hand), so the chamber's size is not a measurement. It is the
  pack's size plus a small stated clearance, with solid walls and no finger notches: the reviewed version's 10 x 72 mm
  side notches, which let the gripper carry the pack home by its long sides, are gone. What matters for the task is
  kept: the pack drops into a pocket only a little larger than itself, so no finger fits between the pack and the
  chamber's sides, and the pack has to be set in from outside and pushed home, as a person does it. The 3 mm play is
  if anything generous to the robot; a tighter real chamber would make the bottom-edge-first insertion harder, not
  different. Measuring a camera's chamber would replace these numbers.
- The sizes marked estimate above, until a camera is measured; the batteries are left out.

## Setup
Positions in cm from the midpoint between the arm bases: x forward, y to the robot's left, heights above the table.

| Object | Description | Position |
|---|---|---|
| camera | instax mini 12, mint green, upright; fixed | centre (38, 0), back towards -y (right arm), lens towards +y (left arm) |
| back cover | mint, 3 mm, hinged along its far edge (x 43.2) | covers the back; opens towards -y |
| film chamber | dark floor, yellow mark at its top far corner, solid sides | x 34.65-41.35, 1.1-10.9 cm up, behind the cover |
| back cover lock | grey tab | x 34.0-35.2, y -4.5 to -3.3, 11.2-11.9 cm up |
| lens ring | silver, black front, index mark on top | centre (38.6, y 3.3-5.9), 4.8 cm up |
| shutter button | silver | (34.6, y 3.3), 9.2 cm up |
| film pack | black; two grey holes and a yellow corner mark on its back | flat, back up, centre (24, -18), length along y, yellow end away from the arms |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`. For real trials: the camera
empty, cover locked, ring at OFF, a fresh pack placed as in the setup photo.

## Scoring
- **Seated:** the pack in the chamber with its yellow end up and its back out (each axis within 10 degrees), standing
  on the chamber's floor and centred across it (each within 3.5 mm), its back within 1.5 mm of flush with the mouth.
- **Locked:** the cover pushed shut within 2 degrees with the lock tab up.
- **ON:** the lens ring turned 30 degrees or more.
- **Holes touched:** in a real trial, a 30 x 20 mm square of household aluminium foil is taped over each hole before
  the trial (a spent pack can be used). After the trial, a close-up photo of each square shows whether the foil is
  dented, creased or torn inside the hole's 20 x 10 mm outline. Over the hole the foil is unsupported, so any press on
  the hole dents it; over the pack's plastic around it, a touch leaves it flat. In the sim: any part of either gripper
  finger, pads or body (both finger links, as three boxes each, from the YAM's own finger mesh), comes within 0.5 mm of
  a hole's 20 x 10 mm area on the pack's back, at any step.
- **Hands clear:** no point along either gripper's pads, and neither fingertip's end, within 2 cm of the camera.

| Score | When |
|---|---|
| 1.0 | the film cover ejected by a shutter press made with the pack seated, the cover locked and the ring at ON; still so, hands clear, the pack still; held for 1 s, which ends the trial |
| 0.75 | seated, locked and ON, but not yet ejected (or a hand near the camera) |
| 0.5 | seated and locked |
| 0.25 | seated |
| 0 | anything else, including a pack in the chamber the wrong way round, the holes touched (as above), and the cover ever pulled 5 degrees open while locked |

In a real trial, the film cover in the slot and the camera's film counter (S, then 10) show the result; a ruler gives
the pack's seating.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder. `mechanism.py` must sit beside the task file, which
imports it:

```bash
INSTR="$(python3 -c 'import json; print(json.load(open("task.json"))["instruction"])')"
COMMON=(--embodiment yam_arena -E task=load_instant_film.py -E object_jitter=0 -E control_hz=20 -E cam_width=320
        -E cam_height=176 --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames)

# Dream-1
inspect-robots run "${COMMON[@]}" --policy dream --log-dir ~/yam-eval/load-instant-film-dream

# The scripted demo, which reaches full score
inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/load-instant-film-demo

# Wrong ways (probes of the scoring rules)
FILM_PACK_TURNED=1 inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/lif-turned   # yellow end down
FILM_DEMO=noring inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/lif-noring     # ring left OFF
FILM_DEMO=presshole inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/lif-hole  # press made on a hole

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/load-instant-film-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. With `FILM_DEBUG=<file>` set, each control step adds a line to that file: the
pack's place and turn, whether it is in the chamber and seated, the cover's angle, the lock's travel, the ring's angle,
the shutter's travel, the film cover's height, the lock state, presses, ejection, the holes rule, and where the pad
tips are.

## The scripted demo
It plays a fixed plan from the fixed layout with the arm's own inverse kinematics, and reads nothing from the
simulator.

1. The right hand, pointing down, presses the lock tab 3 mm; the cover springs 20 degrees ajar.
2. Turned so its two closed fingertips lie along the cover, it comes down into the gap behind it and pushes it round
   its hinge, in 5-degree steps, to its stop at 95 degrees.
3. It pinches the pack across its width, holding 16 mm of its 20 mm sides, lifts it, turns it upright (yellow end up,
   back towards the arm) and stands it on the table.
4. It regrips the standing pack by the top of its long sides, near its back only (6 to 12 mm deep), with the hand
   tipped 9 degrees down, lifts it and brings it behind the chamber tilted 9 degrees out at the top, the hand now
   pointing straight at the camera. It sets the pack's bottom edge on the chamber floor, every finger point 2.5 mm or
   more outside the chamber's mouth, and lets go. Below 12.3 degrees off upright the pack's centre of mass is inside
   its bottom outer edge, so it falls in upright. The closed fingertips then press its back flat, 84 mm up the pack,
   20 mm above the top of the holes.
5. It goes round the open cover's free edge, low, and swings the cover shut from outside to about 45 degrees, then
   turns to point at it and pushes it straight shut until it locks, and parks.
6. The left hand, pointing at the lens, grips the ring and turns it (to its 80-degree stop), lets go, then presses
   the shutter button 2.2 mm and parks. The film cover comes out of the top.

## Runtime and seeds
- **Official runs of this version: not yet run.** The scripted demo and the probes (`FILM_PACK_TURNED=1`,
  `FILM_DEMO=noring`, `FILM_DEMO=presshole`) still have to be run in the official simulator on this geometry; no score
  for this version is claimed here.
- **The reviewed version** (with side notches) scored 1.0 at step 991; that run used the notched chamber and does not
  carry over.
- **Offline checks:** the scoring, lock logic, reach, the solid chamber, the hole rule, and the planned finger geometry
  (from the YAM's URDF kinematics: outside the chamber while loading, never on a hole, on a hole in the
  `presshole` probe) run without the simulator:
  `python -m pytest -q test_load_instant_film.py` (needs `yam_arena` importable).
- **Seeds:** one fixed layout (`-E object_jitter=0`), so the seed doesn't change the start.
