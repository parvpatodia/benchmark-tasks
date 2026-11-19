

# -- appended by sim/mk.sh to the stem copies only (o00, j00, w00, f00, k00); the task file itself is unchanged. Each
# stem is the demo's own plan with one change, and its expected score (README Sim results; test_sim_stems replays each
# offline):
#   o00  one arm: the left stays parked, the right presses as the demo does (20 mm under contact over 1.5 s).
#        Expected 0, "pressed with the hand not steadied".
#   j00  one arm, the same press as a 0.15 s jab. WHY: the hand's inertia might let a fast press fire before the
#        wrist rises 2 mm. Expected 0, "not steadied".
#   w00  one arm, a press near the apex (8.5 mm back) driven down and 20 deg toward the wrist, so friction tilts the
#        contact force toward the wrist. WHY: with a hinge low under the fingers that force held the wrist down; the
#        hinge now sits at the contact's height. Expected 0, "not steadied" (or "slid", also 0).
#   f00  both arms; after the twist the right presses the lancet down in the stand before lifting it. WHY: the real
#        lancet, its tab off, fires into the stand's post. Expected 0, "fired in the stand".
#   k00  both arms; after firing the right lowers the lancet into the container's opening and keeps holding it.
#        Expected 0.6: fired well, never let go.
_STEM = "none"
_JAB_S = 0.15
_NEAR_S = 0.0085
_TILT = math.radians(20.0)
_KEEP_Z = TOP + 0.181            # the TCP lowered over the opening: the lancet's face 150 mm up, inside (lid 162 mm)


def _one_arm(right):
    total = sum(w.duration for w in right)
    return [_w(2.0, PARK_L, OPEN_F, "park, gripper down", 0.0), _w(total - 2.0, PARK_L, OPEN_F, "stay parked", 0.0)]


def _stem(right, left, seed):
    if _STEM == "j":
        right = [_w(_JAB_S, w.pos, w.finger, w.label, w.yaw) if w.label == "press until it fires" else w for w in right]
    if _STEM == "w":
        near, yaw = site_world(seed, _NEAR_S)
        dp = near - site_world(seed)[0]
        side = np.array([math.cos(yaw), math.sin(yaw), 0.0]) * PRESS_DZ * math.tan(_TILT)   # toward the wrist
        moved = {"over the fingertip", "steady over the fingertip", "just over the site", "wait for the wrist to be held",
                 "tip on the site", "press until it fires", "hold the press", "lift off", "up"}
        right = [_w(w.duration, np.asarray(w.pos) + dp + (side if w.label in ("press until it fires", "hold the press")
                                                          else 0.0), w.finger, w.label, w.yaw)
                 if w.label in moved else w for w in right]
    if _STEM in ("o", "j", "w"):
        left = _one_arm(right)
    if _STEM == "f":
        out = []
        for w in right:
            out.append(w)
            if w.label == "hold":    # the twist is done: the tab is off, the lancet still in the stand
                out.append(_w(1.5, np.asarray(w.pos) - [0, 0, PRESS_DZ], w.finger, "press it down in the stand", w.yaw))
                out.append(_w(0.6, w.pos, w.finger, "back up in the stand", w.yaw))   # 2.1 s in all
        right = out
        # WHY: the left waits as long again, so it still holds the wrist through the right's press
        left = [_w(w.duration + 2.1, w.pos, w.finger, w.label, w.yaw) if w.label == "wait parked" else w for w in left]
    if _STEM == "k":
        i = [w.label for w in right].index("let go")
        over = right[i - 1]
        right = right[:i] + [_w(1.5, (over.pos[0], over.pos[1], _KEEP_Z), GRIP_L, "into the opening, held", over.yaw),
                             _w(4.0, (over.pos[0], over.pos[1], _KEEP_Z), GRIP_L, "keep holding it there", over.yaw)]
    return right, left


def demo(delay=0.0):  # noqa: F811
    seed = run_seed()
    right, left = _stem(*plan(delay, seed), seed)
    return {"right": right, "left": left}
