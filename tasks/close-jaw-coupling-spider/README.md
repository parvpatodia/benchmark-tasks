# Close a Lovejoy L100 jaw coupling: seat the spider, turn the driven shaft to mesh, slide the loose hub to its line

A small drive on a bench: a worm reducer whose output shaft carries the fixed hub of an L100 jaw coupling, and in line
with it a 5/8 in driven shaft turning freely in two pillow blocks, its loose hub slid 50 mm back on the key and turned
40 degrees out of mesh. The robot takes the open-centre spider from its holder and seats it in the fixed hub's jaws,
turns the driven shaft until the loose hub's jaws face the spider's free gaps, slides the loose hub along until its back
face is on the line scribed round the shaft (faces G apart), and takes both hands away.

## What it tests

Three steps that only score in order. A part seated against a face. A phase found by turning one shaft about its own
axis while the other is held by the drive: the loose hub only goes on when its three jaws face the three gaps the
fixed jaws leave between the spider's six legs, and a hub pushed on without the turn stops with its jaw tips on the
leg ends (mode no_turn). A slide to a line, through a firm push: the loose jaws must lift the seated spider out of its play on their chamfers
as they enter, and a push without enough follow-through stops there (mode weak_push). The slide joint's travel ends at home, faces exactly G apart, so
the hub cannot be pushed past the line or press the spider; the stage tests getting the hub all the way on. The turn is closed fingers
pushing the loose hub's top jaw sideways (the hub turns the shaft through its key); the slide is a closed hand behind
the hub's back face.

Why it matters (Impact on jobs): closing a coupling between a drive and a driven machine is routine reassembly for
industrial machinery mechanics (O*NET 49-9041.00, "446,900 employees", task "Reassemble equipment after completion of
inspections, testing, or repairs.", `src/onet_49-9041.00.txt`) and general maintenance and repair workers (49-9071.00,
"1,621,800 employees", `src/onet_49-9071.00.txt`). The maker's L-Line install guide (`src/lovejoy_install_2012.txt`,
PDF gzipped beside it) gives the order used here: "Slide the appropriate hub on each of the shafts over the keys"
(6.5), "insert the spider into one of the hubs" (6.6), then move the equipment "to achieve the" G gap "between hub
faces" (6.7), with the hubs "touching the side of the spider or cushions without any excess pressure".

## The parts (every number in `jaw.py` carries its basis)

| Part | In the model | Basis |
|---|---|---|
| Hubs | L100, 2.54 in OD (12-sided), 1.38 in long, 3 jaws each, 0.69 kg each | [SOURCE] `src/lovejoy_jaw_2025.txt`, L and C Type dimensional data, row L100: OD 2.54, LTB 1.38, G 0.71, OAL 3.48 in, 3.20 lb complete at min bore; mass [ARITH] from that |
| Spider | L099/L100 open centre: 6 legs 0.43 in wide, 0.61 in along the axis, 1.03 in centre hole, 28 g | [SOURCE] same file, elastomer dimensional data, row L099/L100: A 2.54, H 1.03, W 0.61, B 0.43 in; ring radius and density [EST] |
| Gap G | 0.71 in, 18.0 mm face to face | [SOURCE] row L100 |
| Shafts | 5/8 in steel; driven 206 mm long, turns freely (damping 0.02 N m s) | [CHOICE] a bore inside the L100's 1.375 in max [SOURCE]; drag [EST] |
| Loose hub on its key | prismatic joint along the shaft, 50 mm travel, viscous drag 6 N s/m | [EST] for a slip fit |
| Drive | reducer housing 82 x 110 x 120 mm; its output hub held by a stiff joint with 0.002 rad play | [CHOICE]; a worm reducer does not back-drive by hand |
| Holder | two wood blocks 12 x 50 x 14 mm, either side of the spider | [CHOICE] |

Engagement: the loose hub sits 14 mm on its shaft when home [CHOICE]. The guide says "Ideally" engagement equals the
shaft's diameter (15.9 mm); 14 mm leaves the spider 2.8 mm a side to come down past the shaft end. The spider starts
with two legs level, which is already in phase with the fixed jaws.

## How it is modelled

Two articulations in PhysX (`mechanism.py`): the drive (housing as base, fixed hub as a moving link) and the driven
side (frame and pillow blocks as base, shaft and loose hub as links). The fixed hub is a moving link, not part of a
base, because PhysX filters a mechanism's links against its own base, and the spider and the loose hub must touch its
jaws. The spider is a loose rigid body of 12 convex pieces. Each jaw's flanks run parallel to the neighbouring legs.

Simplified: the elastomer spider is rigid. Its give is replaced by 1.0 mm play from each leg to each jaw flank
[CHOICE] and a 1.2 mm chamfer on each jaw tip [EST], so the loose jaws enter at a phase within the play, as a real
spider's legs flex. Set screws, keys and torques are out of scope (the hub slides on a joint).

## Forces

Small. The hub slide works against 6 N s/m of viscous drag on the key [EST], about 0.1 N at the plan's 14 to 18 mm/s [ARITH]. The turn pushes one jaw against only the shaft's 0.02
N m s drag: about 0.01 N m at the plan's 0.4 rad/s [ARITH]. The spider is held
at 15 N a finger on two leg ends.

## Scoring

From the end state. The 1.0 stage must hold for 1 s; `score()` itself tops out at 0.75 and 1.0 comes only from
`success()` held `SUCCESS_HOLD_S` (1 s), so a state reached on the last step cannot score 1.0.


- 0.25: spider seated (centre within 1.5 mm of the axis, its axis within 5 degrees, far face within 1.0 mm of the
  fixed hub's face) with every gripper point 30 mm or more from it.
- 0.5: also the loose jaw tips 3 mm or more past the spider's near face (in its gaps; tips resting on legs read 0 mm).
- 0.75: also the hub faces G apart within 1.0 mm (back face on the scribed line).
- 1.0: also spider, shaft and loose hub at rest (points under 0.01 m/s, spins under 0.5 rad/s), every gripper point 30 mm or more from all three.

No seated spider scores 0, whatever the hub does. Real trial: a feeler gauge for G and the spider face, a straightedge
across the hub ODs for the axis, and the video for the hands. Every threshold is a [CHOICE] read with those tools.

## Modes

`TASK_DEMO=<mode>` picks a scripted plan (motionkit waypoints; scene constants and the arms' own joint readings only).

| Mode | Expected | What it does |
|---|---|---|
| demo | 1.0 | right pushes the top jaw across, turning hub and shaft 40 degrees, and parks; left seats the spider, then slides the hub to the line |
| nothing | 0 | arms stay in the ready pose |
| no_spider | 0 | the turn and the slide, spider left in its holder (the lazy route of skipping the spider) |
| no_turn | 0.25 | spider seated, hub pushed on without the turn: jaw tips stop on the leg ends |
| part_turn | 0.25 | only half the turn (20 degrees): out of the phase window |
| weak_push | 0.25 | full turn and spider, but the slide aimed 8 mm short of the line without the demo's follow-through: the jaw tips jam where they meet the spider |
| knock | 0 | left shoves the fixed hub sideways (the drive holds it); no spider, so 0 |

## Results

All from run `result.json` files on hash 57d5b0f3 (sha256 of the sorted `*.py` plus `task.json`, first 8 hex), cameras
off, seed 0, unless noted. Hub travel is the slide joint at the end (50 mm = home, faces G apart).

| Mode | Seed | Score | Steps | End |
|---|---|---|---|---|
| demo | 0 | 1.0 | 867 | success held 1 s; shaft 40.8 degrees, hub home |
| demo | 1, 2 | 1.0 each | 867 | the same (run on hash 63668945, which differs only in the weak_push mode's name and comments) |
| no_turn | 0 | 0.25 | 653 | jaw tips stop on the leg ends (hub at 32.0 mm, so the depth measure reads -2.5 mm, not the nominal 0 mm; under 3 mm either way) |
| part_turn | 0 | 0.25 | 908 | out of the phase window (shaft 23.6 degrees, hub at 34.1 mm) |
| weak_push | 0 | 0.25 | 909 | tips jam where they meet the spider (hub at 35.6 mm, 1.1 mm in; also on seeds 1 and 2) |
| no_spider | 0 | 0 | 618 | no seated spider |
| knock | 0 | 0 | 226 | no seated spider |
| nothing | 0 | 0 | 61 | |

The filmed demo and the Dream-1 run below ran on hash 1340836f. Since then the `task.json` text changed (setup,
scoring, fidelity, media and scores wording; no scene numbers), and the code changed in two places: `score()` now tops
out at 0.75 (1.0 only from `success()` held 1 s, as the scoring text already said; the logged `distance()`, which is 1
minus `score()`, so reads 0.25 at the goal), and the `short` mode became `weak_push`. The demo plan and the scene code
are unchanged, and the demo scores 1.0 in 867 steps on both hashes.

Filmed demo (stock kit cameras, 320 x 176, hash 1340836f): `task_score: 1`, 867 steps. Dream-1 (`--policy dream`,
seed 0, hash 1340836f): `task_score: 0` after all 1,200 steps; it never took the spider from its holder. Its cameras rendered
washed-out grey frames for about 13 s of that run (steps 575 to 802 and 1,087 to 1,118, all three cameras at once); the
filmed demo and the HD recording of the same scene have none, so this reads as a renderer fault on a busy GPU.

In the demo the push leaves the shaft 38.9 degrees round of its 40; the jaw-tip chamfer takes up the rest as the hub
slides on. `checkkit.py` (IK with 3 degree joint margins, full-link clearance, arm gap, run limit) and `test_jaw.py` (13
tests, every scoring rule above against the code) pass.

## Demo video

`sim/film.sh` runs the stock filmed demo and, through env `JAW_HD`, records the same run's cameras at 1280 x 720
(`jaw.py` HD recorder; scoring and Dream runs never set it). `demo.mp4` (1920 x 900, H.264, ffmpeg overlays) has one
large view and three small ones: the large view is the wrist camera of the working hand (right for the turn, left for
the spider and the slide) and a close-up of the coupling for the end state; the small views are the top camera, a
close-up crop of it and the other wrist camera. A caption bar names each step with its time from the plan's labels
(turn 0 to 12.2 s, take the spider to 23.0 s, seat to 31.5 s, slide to 41.8 s, hands away), and the last frame is held
2 s. `setup.png` is the first HD top frame.

## Run it

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=jaw.py -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \
      --scorer task_score --max-steps 1200

One fixed layout; the demo is open loop. `fastrun.py` runs every mode in one session without cameras;
`PYTHONPATH=<yam_arena source> python3 test_jaw.py` runs the offline tests.

## Assets and licences

All geometry is primitive shapes and hulls in this folder. `mechanism.py`, `motionkit.py`, `checkkit.py` and
`fastrun.py` carry their MIT notices. Saved in `src/`: the Lovejoy jaw coupling catalogue section (2025) and L-Line
install guide (2012), text plus gzipped PDF, and the two O*NET summaries as text.
