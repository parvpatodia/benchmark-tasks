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

Why it matters (Societal relatability): coin-lock trolleys are returned this way at supermarkets in much of Europe and
elsewhere; the coin is the deposit the shopper gets back. The mechanism is that of EP0427819B1 (`src/ep0427819.txt`):
the coupling member goes in through an opening in a side wall, the coin slide runs at right angles to it, and on full
insertion the coin slide is released and a compression spring pushes it out "so that the coin can be removed".

## The parts (every number in `trolley.py` carries its basis)

| Part | In the model | Basis |
|---|---|---|
| Nesting gap | 178 mm handle to handle | [SOURCE] `src/unarco_M60TN.txt`: "Nesting Distance 7"" |
| Handles | steel tube 25 mm across, 300 mm along y, axis 0.34 m over the bench, side tubes 80 mm forward to the bench | lock fits "25-27mm Trolley Handle" [SOURCE] `src/malltek_coinlock.txt`; length, height and slope [CHOICE] for the arms' reach (real handle "373/4"" over the floor [SOURCE] `src/unarco_M60TN.txt`) |
| Coin lock | 90 x 70 x 68 mm plastic, red top | [SOURCE] `src/malltek_coinlock.txt`: "Size:90*70*68mm", "Material:Plastic" |
| Chain and key | 11 steel links of 20 mm (220 mm); key 62 x 15 x 3 mm steel, 22 g, a 6 mm wide hole 6 to 14 mm behind its tip | listing "260mm stainless steel chain" [SOURCE]; 220 mm and key size [CHOICE] so the key hangs 67 mm over the bench |
| Key slot | 21 x 4.5 mm in the near lock's front face | [CHOICE] 3 and 0.75 mm play a side on the key |
| Retaining pin | 5 x 4 mm, sprung down (50 N/m), 3.5 mm travel | stands in for the patent's "ball of e.g. 7 mm in diameter" [SOURCE] `src/ep0427819.txt`; size [CHOICE] |
| Quarter | 24.26 mm, 1.75 mm thick, 5.67 g | [SOURCE] `src/usmint_quarter.txt` |
| Mat | rubber, 100 x 100 x 3 mm | [CHOICE] |

## How it is modelled

Three articulations in PhysX (`mechanism.py`) and two loose bodies. The front trolley (frame and lock as base) carries
its chain: 11 capsule links on hinges whose axes alternate across and along the handle, then the key on a hinge. The
near trolley (frame and lock casing as base) has two moving links inside the lock: a plunger behind the slot (spring,
6 mm travel) with a tooth T, and the coin slide (spring toward -y, 40 mm travel) carrying the quarter's pocket and a lug
L. The slide's spring holds L against T. The key's tip meets the plunger 8 mm in; pushing 4.5 mm further takes T past L,
the slide runs out through the casing's -y end, and a rail behind L then lies across T's way back, so the plunger stays
in and the slide stays out. A third articulation is the near lock's own chain (it hangs behind, not used). The quarter
stands on edge in its pocket (walls on both faces, end stops under its rim); the mat is a box.

Key retention follows the patent: the ball "drops downwards under the action of gravity, so that it occupies the
apertures 19 and 21" (one in the casing over the key, one an "elongate hole" in the key) and "the coupling member 8 is
firmly locked". Here a pin over the key's path, sprung down, has a +x face that slopes (the incoming key lifts it) and an
upright -x face; at 12.5 mm the key's hole comes under it, it drops in, and a pull on the key stops on that face.

Simplified: the patent's coin-slide locking member is replaced by the tooth and rail, and its ball by the pin. The pin's
release by the re-inserted coin slide (the reset) is not modelled.
The chain has no twist joint. The handles are shortened and lowered for the arms (see the table).

Closed shortcuts: a finger is too wide to go more than 10 mm into the slot (2 mm short of release; `test_trolley.py`
checks the open finger's cross-section against the slot); the slide cannot be pried out (the lug holds it until the
plunger moves), and nothing scores without the key in line in the slot; 0.5 and up need the pin down in the key's hole,
so a key held short by the hand does not count; the key cannot reach its own lock.

## Forces

Small. Release takes the plunger spring (150 N/m, about 0.8 N at release [ARITH]) plus friction of L on T under the
slide spring (30 N/m, 2.1 N held in [ARITH]). The key is held at 6 N a finger, the quarter at 4 N.

## Scoring

From the end state. `score()` tops out at 0.75; 1.0 comes only from `success()` held `SUCCESS_HOLD_S` (1 s).

- 0.25: the front lock's key in the near lock's slot, in line (tip within 4 mm across and 2.5 mm up or down of the slot
  centre, key within 15 degrees of the slot axis, tip first) and 5 mm or more past the mouth.
- 0.5: also the coin slide out 35 mm or more of its 40 mm travel, and the key held by the lock (tip 11.5 mm or more in,
  retaining pin within 1 mm of its rest, in the key's hole). The slide comes out only at 12.5 mm of key depth.
- 0.75: also the quarter resting on the mat (centre 5 mm or more inside its edges, lowest point within 3 mm of its top).
- 1.0: also quarter, key, plunger and slide at rest (under 0.01 m/s, 0.5 rad/s) and every gripper point 30 mm or more
  from the key and the quarter, held 1 s.

Real trial: a ruler on the slide's travel and the key's exposed length, a 1 N pull on the chain for the retention, a
photo of the mat and hands.

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

All from run `result.json` files on hash eba907ea (runs fast_eba907ea_1121_112154 and fast_eba907ea_1121_113149; sha256 of the sorted `*.py` plus `task.json`, first 8 hex), cameras
off. End state from the run's last trace row: key tip depth past the mouth, slide travel, the quarter's height.

| Mode | Seed | Score | Steps | End |
|---|---|---|---|---|
| demo | 0 | 1.0 | 682 | success held 1 s; key 12.7 mm in, pin down, slide out 40 mm, quarter flat on the mat |
| demo | 1, 2 | 1.0 each | 682 | the same |
| key_only | 0 | 0.5 | 501 | key held by the pin, slide out 40 mm, quarter left in the slide |
| drop_coin | 0 | 0.5 | 736 | quarter on the bench beside the mat |
| hold_coin | 0 | 0.5 | 760 | quarter still in the hand, its centre 37 mm over the mat |
| partway | 0 | 0.25 | 439 | key 9.8 mm in, held; slide in, pin riding on the key (1.8 mm up) |
| pry_coin | 0 | 0 | 191 | slide in, key hanging |
| no_key_coin | 0 | 0 | 173 | slide in, key hanging |
| nothing | 0 | 0 | 61 | |

In the demo the key's tip and back end move under 2 mm/s once it is let go (the 1.0 check is 10 mm/s). The pin holds it
at 12.7 mm against the chain's pull. Dream-1 has not been run on this task yet.

`checkkit.py` (IK with 3 degree joint margins, full-link clearance, run limit) and `test_trolley.py` (24 tests:
every scoring rule above against the code, each mode's end state, the latch geometry, the finger, the text) pass.

## Demo video

`sim/film.sh` runs the stock filmed demo and, through env `TROLLEY_HD`, records the same run's cameras at 1280 x 720
(scoring and Dream runs never set it). `setup.png` is the first top frame.

## Run it

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=trolley.py -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \
      --scorer task_score --max-steps 1600

One fixed layout; the demo is open loop. `fastrun.py` runs every mode in one session without cameras;
`PYTHONPATH=<yam_arena source> python3 test_trolley.py` runs the offline tests.

## Assets and licences

All geometry is primitive shapes and hulls in this folder. `mechanism.py`, `motionkit.py`, `checkkit.py` and
`fastrun.py` carry their MIT notices. Saved in `src/`: the patent text, the coin lock listing, the cart sheet (text plus
gzipped PDF) and the US Mint coin specifications.
