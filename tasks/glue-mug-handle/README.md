# Glue a broken mug handle back on

A stoneware mug lies on its side in a foam cradle, the two broken stubs where its handle was facing up. The handle, in
one piece, stands on its broken ends on a felt pad. A tube of no-drip gel glue stands nozzle down in a holder, cap off.
One arm does the repair in the order the glue's data sheet gives (masking, dampening the faces and closing the tube are
left out and not scored): dry-fit the handle on both stubs at once and stand it back on its ends on the pad; pick up the tube without
squeezing glue out, put a thin bead on each broken face by squeezing the tube's walls, and put the tube back; then,
within 5 minutes of the first glue, seat both ends of the handle on their stubs at once, press it down firmly on both
and let go. Glue anywhere but the two faces, too much on a face (it squeezes out once that end is seated), glue before
the dry fit or before the handle is back on the pad, the handle seated on its glue before the tube is back, the mug
moved or turned in its cradle, or a handle that moves once let go on its glue scores 0. Joined, pressed, let go, the
tube back and both hands clear scores 1.0.

What it asks of a robot is physical: lift a soft tube by its walls with a grip under the gel's yield; meter a bead of
10 to 30 uL by squeeze force and time with the nozzle within 4 mm over a 12 by 8 mm face; and mate two ends 55 mm apart on two faces at
once, each within 1.5 mm, and press each with 3 N, without dragging glue onto the glaze. The handle and the mug move by seed, so a replayed path
misses (`test_open_loop_plan_fails_other_layouts`).

This revives the idea of benchmark-tasks #1610 (a mug handle reglue, still open but not revised within 48 hours of
its review, which says "Your idea stays reserved for 48 hours from this review; if no revision arrives by then, the
hold lapses and another team can claim the idea"). That review asked for a real adhesive with the maker's hold time, real
dispensing, and glue scored on both faces with both ends in contact. This task scores those: the glue is LePage Extreme
Glue No Drip Gel, whose data sheet allows this use and gives the open time; the bead comes out only when the tube is squeezed;
each face has its own glue count; both ends must be seated, and each stub measures the press on it.

Nearest tasks on the board: #2003 (dowels) and #1450 (clamp-up) press glued joints but never dispense; #461 and #1909
dose a liquid by weight, watching a scale; #2177 squeezes a bulb flat but meters nothing; #1454 lays a caulk bead with
a trigger gun along one joint. Here the amount on each of two small faces is metered by squeeze force and time, from a
soft tube held with a grip under the gel's yield, and the faces are then mated at once.

Files: `mug_handle.py` (the task), `mechanism.py` (articulated objects, adapted from benchmark-tasks #223, MIT; here it
also builds loose articulated objects and places them by seed), `test_mug_handle.py` (offline tests), `task.json`,
`tools/` (task.json generator, reach and clearance checks), `sim/` (run scripts).

## Sources

Saved copies were read in full; every quote here is from them.

- Henkel, LePage Extreme Glue No Drip Gel, technical data sheet: "Pre-fit all materials before applying adhesive.",
  "Mask off the adjacent areas with tape before gluing if necessary.", "Apply thin layer of adhesive to one surface.
  Immediately position and press parts firmly together within 5 minutes.", "Do not move or use parts for at least 2
  hours."; its tables give Open Time "3-5 minutes", the package as "Squeeze Tube", "20 g", Color "Colorless, Clear" and
  specific gravity 1.17. Its limitations include "Should not be considered for direct or indirect food contact"; the handle's joint is on the mug's outside, away from the drink.
- LePage, guide to gluing ceramics: "If the mug has simply cracked in half or the handle has fallen off, you can follow
  the steps above", and of Extreme Glue No Drip Gel, "it is rated safe for items that hold hot liquids".
- IKEA DINERA mug, product page: Height "9 cm", "30 cl", "Stoneware, Coloured glaze". Its package measures 9 by 9 by
  12 cm; that bounds the mug with its handle but is not the mug's size.
- Loctite (Australia), page on fixing a broken mug: it recommends Loctite Super Glue Control Liquid.
- Henkel, Loctite Super Glue Control Liquid, technical data sheet (Denmark, ref. NRLO15000TDS): "Not suitable for
  bonding silicone rubber, glass, glazed surfaces", "Never use for bonding assemblies that will hold hot liquids".
- Henkel, Loctite Super Glue Gel Control, technical data sheet (US): "Not for assemblies which will hold hot liquids",
  "Do not use on glazed surfaces".

The guide's steps say "Apply the glue to each of the pieces"; the data sheet says one surface, and it governs here. The
guide links the product page of item 2764485, the data sheet's item number, so both name the same glue. The data sheet
also says "Bonding two non-porous surfaces can result in a slower cure time and is generally not recommended", and to
dampen non-porous surfaces. The faces glued are the broken body, not the glaze; whether that body counts as porous is
not sourced, and if it does not, the sheet's dampening step applies (left out and not scored). The maker's own ceramics
guide names this glue for a fallen-off handle.

Why not super glue: Loctite's broken-mug page recommends Super Glue Control Liquid, but that product's own sheet and
the Gel Control sheet (quoted above) rule out glazed surfaces and items that hold hot liquids. We follow the sheets.

Not sourced (estimates, marked in the code): the mug's radius (39 mm, from 30 cl in 9 cm with an estimated 7 mm base
and 5 mm wall), the handle's section (12 by 8 mm), span (55 mm) and reach (30 mm), the stubs' 3 mm, the tube's size and
wall stiffness (20 g of gel, about 30 g in all; no maker drawing), the gel's yield squeeze (4 N) and flow (2.5 uL/s per
N over it), the thin-bead window (10 to 30 uL on a 96 mm2 face, a layer 0.10 to 0.31 mm thick), and "firmly" as 3 N on
each stub. The data sheet gives no volume for a "thin layer" and no force for "firmly".

## The model

- The tube is one loose articulated body: the tube and its nozzle, and two walls 22 mm apart that slide in up to 6 mm
  each against a spring (2 N/mm, 1 N preload). The squeeze is read from the walls' travel.
- The gel: none comes out while the two walls' mean squeeze is 4 N or less (a no-drip gel holds under a small
  squeeze); over that it flows at 2.5 uL/s per N. Glue lands on a broken face when the nozzle's tip is over it (1.5 mm inside its
  edges, up to 4 mm above, or resting on it) and points down within 20 deg; otherwise it is astray. A bead drawn on
  each face rises with its glue (1.4 mm proud at 10 uL, full at 30 uL), drawn pale so it shows; the real gel is clear,
  and a real layer of 10 to 30 uL on the face is 0.10 to 0.31 mm thick, so the drawn bead is 8 to 14 times thicker.
- Seated: a broken end's centre within 1.5 mm of its face across and along the mug, every corner of the end 0.3 mm into
  to 0.6 mm over the face in height (a feeler gauge finds a gap at an edge), the handle square to the mug within 4 deg. The modelled handle is symmetric, so it fits either way round.
- Pressed: each stub rides a stiff spring (2.5 N/mm, 1 N preload, 2.5 mm travel), only to measure the press on it; the
  handle's weight never moves it. Both stubs held at 3 N or more for 0.5 s while seated is the press, and proves each
  end is in contact. The real stubs are rigid. The arm's joints are PD springs, about 0.5 N/mm at the TCP in z at the
  press [EST: from yam_arena's joint stiffnesses], so the demo aims the TCP 20 mm under contact (about 4.6 N a stub [ARITH from that EST]). Under the press
  the handle slips 0.7 to 3.8 mm up in the grip (run 10), so the demo eases over 1.5 s to a light push (6 mm under
  contact) and lets go there; eased back to contact, the hand lifted the handle up to 2.5 mm off its glue (run 10).
  Its dry fit pushes lightly too (6 mm under contact): held only, the handle sat 0.7 deg tilted (run 9).
- The order: glue before the handle is seated on both stubs dry, or before it is back standing on the pad with both hands
  clear, is a fault, and so is seating the handle on its glue before the tube is back in its holder. Lifted off its
  stubs before let go (an end over 2 mm up, or its centre over 1.5 mm from its face's centre), the joint is broken and
  must be seated and pressed again. WHY 2 mm, not seated's 0.6 mm: released in 0.4 s from a press, the stubs' springs
  threw the handle up to 1.3 mm (run 9), which a rigid real stub would not.
- Both hands are watched: let go means each hand open and its TCP 30 mm from the handle's grip, or 100 mm away.
- The glue has no strength in the sim. The data sheet's "Do not move or use parts" is scored instead: once let go, the
  handle moving 1 mm in the mug's frame is a fault. The run is 150 s, inside the open time, so the open time never
  binds in a sim run, and the 2-hour handling time is not simulated: yam_arena ends the trial 1 s after success. The mug
  lies on its side, so the handle rests on its stubs by its weight.
- The mug may slide 8 mm along its cradle and turn 5 deg; more is a fault. Its body collides as a 32-sided prism (64 vertices,
  PhysX's default hull limit; its flats 0.19 mm inside the true radius): as a PhysX cylinder, the untouched mug spun in place on the rails' edges,
  about 40 deg by the run's end on every seed (run 7). Its centre of mass is set 3 mm below its axis, so the whole mug,
  stubs included, rests stubs up; the offset stands in for the grip of a real foam cradle, which the rails do not model.

## Seeds

The handle's place on its pad (shift from the pad's centre, turn) and the mug's shift along its cradle:

| `--seed` | layout | handle shift, mm | handle turn, deg | mug shift, mm |
|---|---|---|---|---|
| 0 | 1 | 12, -10 | 20 | 20 |
| 2 | 0 | 0, 0 | 0 | 0 |
| 4 | 2 | -10, 12 | -17 | -18 |

The scripted demo is not a policy: it reads the seed's layout and plans from it, and it plans its squeeze time from the
flow model (20 uL a face). It shows the task can be done.

## Real trials

Hardware: IKEA DINERA mugs (stoneware), one tube of LePage Extreme Glue No Drip Gel per session, a balance (0.01 g
readability, 2 kg or more capacity, for the mug, the cradle and the press) with the foam cradle fixed to its pan and
the balance fixed to the table, a felt pad and a tube holder on blocks so their tops stand at the heights the sim's
have over the cradle, a 0.001 g scale, and at each joint a side camera with a backlight behind it and a 0.6 mm shim
fixed in view beside the joint as a reference.

- Consumables: each trial needs a mug with its handle off. Score the handle's root with a tile saw 1 mm deep at both
  ends and snap it off, so the breaks are near the sim's stubs; clean and dry the faces (out of scope). A handle that
  breaks elsewhere is discarded. Before the trial, ink dots on the handle and on the mug beside each joint.
- Seeds: the handle is put on the pad and the mug in the cradle by printed templates, one per layout.
- Glue: the tube is weighed on the 0.001 g scale before and after the run (never during it): the total, at specific
  gravity 1.17 (10 uL is 11.7 mg), must be 20 to 60 uL. The balance, tared before the run, gives each face's bead
  from the video, read once steady with nothing touching the mug or the cradle (the second face's is the change in the
  reading). One count (0.01 g) is 8.5 uL, so the real check is coarser than the sim's: each face must read 0.01 to
  0.03 g, which is 4 to 30 uL of true glue, against the sim's 10 to 30 uL.
- Astray glue: after the run, photograph every surface the nozzle passed over (the glaze, the cradle, the pad, the
  table and the holder's rim) under raking light with a scale in view, then wipe them. Outside a band 1 mm wide along
  each joint line (the fillet below), the spots' summed area over 3 mm2 (about 1 uL at 0.3 mm thick, the sim's
  allowance) scores 0.
- Squeezed out: a real thin bead pressed down leaves a small fillet at the joint's edge, which the sim does not model.
  [CHOICE] Before the trials, a 20 uL and a 40 uL bead are each pressed at 6 N on a spare mug and their fillets
  measured; glue reaching beyond either joint's edge further than midway between the two (side cameras, then
  calipers) scores 0, standing in for the sim's over-30-uL rule.
- Order and time: from the video, the dry fit seated on both stubs at once (no line of backlight through either joint
  as wide as the 0.6 mm shim), then the handle standing on the pad and both hands off it, before the first squeeze;
  the tube in its holder before the handle is seated on its glue; both ends seated within 5 minutes of the first
  glue. The run is limited to 150 s, as in the sim.
- Seated, on the glue (clear gel lets backlight through, so light is not used here): each joint's gap measured in the
  side camera against the 0.6 mm shim, then after the run a 0.6 mm feeler gauge must not enter any side of either
  joint; neither end overhangs its face by 1.5 mm (calipers).
- Pressed: the balance reads at least 612 g (6 N) over the rest weight for 0.5 s with both ends seated (the video). A
  real trial cannot split it per end; the sim does.
- Lifted: from the press to let go, no end rises more than 2 mm off its face in the side cameras (against the shim),
  as in the sim.
- Moved: the ink dots, tracked in the side cameras from let-go to the end, move under 1 mm relative to each other.
- At the end, as in the sim: the tube upright in its holder, both hands open and away from the handle, the handle still,
  and the mug within 8 mm of its template mark and turned under 5 deg (a mark on its end against a line on the cradle).
- The mug is then left 24 h (full cure, per the sheet) and the joint tested by hand; that is reported, not scored.

## Run

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=mug_handle.py -E object_jitter=0 \
  -E control_hz=20 --instruction "$(python3 -c "import json;print(json.load(open('task.json'))['instruction'])")" \
  --scorer task_score --seed 0 --max-steps 3000
```

## Sim results

yam_arena, scripted demo, `object_jitter=0`, 20 Hz, 3,000-step limit. The film and the Dream-1 run hashed
`mug_handle.py`, `mechanism.py` and `task.json` before starting: `mug_handle.py` and `mechanism.py` match the files
here, and `task.json` differs from the runs' copy only in `scores`, written after the runs (they read only its
instruction).

- Filmed demo (`--seed 0`): 1.0; the run ended on success at step 1,484 (74.2 s), 667 s of wall time on a shared GPU.
- Traced runs of the same demo (run 11, logging copies made by `sim/mk.sh`): 1.0 on `--seed 0`, `2` and `4`. Glue on
  the two faces 20.42 and 20.41, 20.44 and 20.43, 20.46 and 20.45 uL; none astray; the weaker stub's peak press 4.35,
  4.91 and 5.17 N; from the press to let go both ends stayed seated, none rising off its face (so the 2 mm lift-off
  allowance is never used) and each centre within 1.2 mm of its face's; the mug turned at most 0.16 deg.
- The Dream-1 run (`--seed 0`): 0 over 3,000 steps. Dream-1 also ran on the revision before this one (0 over 3,000
  steps) and once crashed after about 4 minutes with no score, when the shared instance's disk filled.
- Earlier runs, each fixed in the files here: run 7 (0), the mug spun in its cylinder collider; run 8 (0.6), the press,
  aimed 1.6 mm under contact, never lifted the stubs off their preload; run 9 (0 and 0.6), the dry fit sat tilted on
  seed 0, and easing the press in 0.4 s let the stubs' springs throw the handle up and break the joint on seeds 2 and 4;
  run 10 (1.0) eased slowly back to contact and allowed lifts up to 3 mm, but there the hand, with the handle slipped
  up in its grip, lifted it 2.5 mm off its glue on seed 0 before let go, so the demo now eases to a light push and lets
  go there, and the allowance is 2 mm, from the springs' 1.3 mm.

## Sim against real

- PhysX has no fluids or soft bodies. The tube is two sprung walls; the gel's flow, where it lands and the bead are our
  model. A real bead spreads and a real tube's output depends on how full it is.
- The glue has no tack or strength: the handle stays on the stubs by its weight and the press. A real joint grips within
  the open time.
- The handle's ends and the stubs are flat, matching faces; real breaks are rough, mate one way only, and locate the
  handle by their shape.

## Tests

`python3 test_mug_handle.py` (with yam_arena on PYTHONPATH) runs the offline tests: the glue model (no flow under the
yield; glue lands on the face under the nozzle; off the face or tilted it is astray; glue before the dry fit; too much
squeezes out; the open time), the seating rules (a tilted end gaping at one edge), the order (glue before the handle is back on the pad; the handle
seated on glue before the tube is back), a lifted joint, a press rebound that is not a lift, the press on both stubs, the left hand, the scorer (the ladder
along the demo, a handle moved after let-go in the mug's frame, the mug turned or slid in its cradle, a model that
stops stepping, the tube left out), the demo replayed through the model by a kinematic stand-in on every film
seed, a plan replayed on another layout failing, and the geometry. `tools/ikcheck.py` and `tools/clearance.py` check the
demo's reach and clearances offline.

## Third-party code

`mechanism.py` is adapted from benchmark-tasks pull request #223 (MIT License; the licence is kept in its header).
