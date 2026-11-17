

# -- appended by sim/mk.sh to the one-arm copies only (o00, j00): the left arm stays parked, so nothing holds the
# forearm stub down while the right arm presses. o00 presses as the demo does (20 mm under contact over 1.5 s); j00
# jabs, the same depth in 0.15 s. WHY: the README claims only what a quasi-static replay shows (the unheld hand tips
# before the lancet fires); these runs test that claim in PhysX, where the hand's inertia might let a fast press fire
# before the wrist rises 2 mm. Expected: 0, "pressed with the hand not steadied". The task file itself is unchanged.
_JAB = False
_JAB_S = 0.15


def demo(delay=0.0):  # noqa: F811
    right, _ = plan(delay, run_seed())
    if _JAB:
        right = [_w(_JAB_S, w.pos, w.finger, w.label, w.yaw) if w.label == "press until it fires" else w for w in right]
    total = sum(w.duration for w in right)
    return {"right": right, "left": [_w(2.0, PARK_L, OPEN_F, "park, gripper down", 0.0),
                                     _w(total - 2.0, PARK_L, OPEN_F, "stay parked", 0.0)]}
