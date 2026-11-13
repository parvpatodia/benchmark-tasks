# Seal a loaded container door with an ISO 17712 bolt seal

The robot seals the right door of a loaded shipping container the way the CBP CTPAT seal procedure asks: an ISO 17712
bolt seal goes on the cam keeper at the bottom of the centre-most locking bar, not on the door handle. The bolt goes
through both ears of the keeper hasp, then the barrel is pushed onto the bolt's tip from the far side until it seats.
A seal closes for good, so a barrel closed on a bolt that is through nothing, or a seal closed on the handle hasp, is
latched as a 0.

Files: `container_bolt_seal.py` (the task), `mechanism.py` (the scene builder), `task.json`.

## Sources

| Number | Value | Source |
|---|---|---|
| Seal placement | "placed on the Secure Cam position, if available, instead of the right door handle ... at the bottom of the center most vertical bar of the right container door" | CBP CTPAT Job Aid, Seal Security Procedures, last updated October 19, 2021 (copy hosted at cummins.com/sites/default/files/2024-07/CTPAT-Job-Aid-Seal-Security-Procedures-October-2021.pdf) |
| Seal standard | ISO 17712 high security seals; "qualifying cable or bolt seals are acceptable" | same job aid |
| Pin | 87.0 mm long, 22.2 mm across (head) | J. J. Keller HBSCP8 coated pin bolt seal spec sheet (cdn.jjkeller.com, Coated-Bolt-Seal-Spec-Sheet_430272.pdf): 3.43 x .87 x .88 in |
| Pin diameter | 8.0 mm steel, 11.8 mm over the plastic coat | same sheet: .31 in / .46 in |
| Operational length | 80.6 mm (read here as the pin below the head) | same sheet: 3.17 in |
| Barrel | 31.6 mm long, 21.6 mm across | same sheet: 1.24 x .85 x .85 in |
| Assembled length | 95.9 mm, so 22.7 mm of pin inside the barrel when closed | same sheet: 3.78 in (the 22.7 is our arithmetic) |
| Uses | trailer door latch, intermodal container door latch | same sheet |
| Jobs | heavy and tractor-trailer truck drivers held about 2.2 million jobs in 2025 | US BLS Occupational Outlook Handbook |

Our choices (not from a source): the hasp ears (5 mm plate, 8 mm gap, 16 mm hole, so 2.1 mm radial play on the coated
pin), the two hasp positions, the 22 mm locking bar stub, the bolt's tip and shoulder shape (the sheet does not draw them),
the barrel's 9 mm bore (0.5 mm radial play) and 2 mm mouth chamfer, the masses (40 g bolt and 25 g barrel, estimated
from the sizes; the sheet gives none), and the frictions.

## Sim results

yam_arena, scripted demo, one fixed start (`object_jitter=0`), 20 Hz, 2,400-step limit. The filmed demo scored 1.0 with success at step 639 (31.95 s) and took 689 s of wall time on a shared GPU. The one Dream-1 run scored 0 over 2,400 steps. Both runs hashed `container_bolt_seal.py`, `mechanism.py` and `task.json` before starting; the hashes match the files here (`task.json` as it was before its `scores` line was filled in).

## Sim against real

- A real barrel snaps onto a groove in the tip and cannot be pulled off. Here both parts stay free rigid bodies; the
  scorer judges the seal closed when the barrel's mouth sits at the shoulder, coaxial, and latches a close at the wrong
  place or through nothing as 0 (that is what a real seal does: it has to be cut off).
- The seal hole and the barrel are 12-sided, faces at the stated radii. The bolt's seal number is not drawn.
- Real hasps vary in size and the keeper's hole is often in a cam retainer; here both hasps are plain ear pairs.
- The scorer reads poses and gripper hulls, not forces.

## Third-party code

`mechanism.py` is adapted from the MIT-licensed `mechanism.py` of Robocurve PR #223, as used by earlier board tasks.
No meshes or other assets: every shape is a primitive or convex hull built in the task file.

## Checks

    PYTHONPATH=<yam-isaac-lab-arena checkout> python -m pytest -q -p no:cacheprovider -W error::DeprecationWarning
