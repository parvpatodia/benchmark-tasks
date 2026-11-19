

# -- appended by sim/mk.sh to the stem copies only (o00 j00 x00 w00 f00 k00 c00 i00); the task file itself is unchanged.
# Each stem is the demo's own plan with one change, and its expected score (README Sim results; test_sim_stems replays
# each offline):
#   o00  one arm: the left stays parked, the right presses as the demo does (20 mm under contact over 1.5 s).
#        Expected 0, "not held still" (the unheld finger gives way down its slide).
#   j00  one arm, the same press as a 0.15 s jab. Expected 0, "not held still".
#   x00  one arm, a jab 30 mm under contact in one control step (0.05 s). WHY: a press fast enough may fire before the
#        light finger moves; the finger then gives way within 0.5 s of the click, and the window counts it.
#        Expected 0, "not held still" (or "no gripper holding the ring finger", also 0).
#   w00  one arm, a press near the apex (8.5 mm back) driven 10 deg off vertical toward the knuckle. WHY: friction then
#        tilts the contact force off vertical, the press the yield sweep finds hardest. Expected 0, "not held still"
#        (or "slid", also 0).
#   f00  both arms; after the twist the right presses the lancet down in the stand before lifting it. WHY: the real
#        lancet, its tab off, fires into the stand's post. Expected 0, "fired in the stand".
#   k00  both arms; after firing the right lowers the lancet into the container's opening and keeps holding it.
#        Expected 0.6: fired well, never let go.
#   c00  both arms; the tab twisted the other way (clockwise seen from above). Expected 1.0: either way breaks it.
#   i00  both arms; the left closes on the index finger instead of the ring. Expected 0, "not held still" (or "no
#        gripper holding the ring finger").
_STEM = "none"
_JAB_S = 0.15
_X_S, _X_DZ = 0.05, 0.030
_NEAR_S = 0.0085
_I_FWD = 0.002                   # the index hold this far nearer the apex than the ring's, and its pads open this far
_I_OPEN = 0.005                  # past the index's skin. WHY: at the ring's distance and 20 mm open, the left's pads
#                                  came 0.3 mm from the thumb and 1.1 mm from the palm; here 4.7 and 3.9 mm, the arms
#                                  11.5 mm apart [RUN: tools/grippers.py sweep, 2026-11-19; floors 3 and 10 mm]
_TILT = math.radians(10.0)
_KEEP_Z = TOP + 0.181            # the TCP lowered over the opening: the lancet's face 150 mm up, inside (lid 162 mm)
_PRESS = {"over the fingertip", "steady over the fingertip", "just over the site", "steady just over the site",
          "tip on the site", "press until it fires", "hold the press", "lift off"}
_HOLD = {"over the ring finger", "steady over the ring finger", "down round the ring finger", "grip the ring finger",
         "hold it up on its stop", "keep holding through the press", "ease off", "let go of the ring finger", "up"}


def _one_arm(right):
    total = sum(w.duration for w in right)
    return [_w(2.0, PARK_L, OPEN_F, "park, gripper down", 0.0), _w(total - 2.0, PARK_L, OPEN_F, "stay parked", 0.0)]


def _index_hold(seed):
    """The left's hold moved from the ring finger to the index (its axis and radius, _I_FWD nearer the apex)."""
    g, yaw = ring_world(seed, HOLD_AT)
    iy, iz = _axis_yz(FINGERS["index"][0])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([-s * iy - c * _I_FWD, c * iy - s * _I_FWD, iz]), -(FINGERS["index"][4] - 0.005)


def _stem(right, left, seed):
    if _STEM == "j":
        right = [_w(_JAB_S, w.pos, w.finger, w.label, w.yaw) if w.label == "press until it fires" else w for w in right]
    if _STEM == "x":
        right = [_w(_X_S, np.asarray(w.pos) - [0, 0, _X_DZ - PRESS_DZ], w.finger, w.label, w.yaw)
                 if w.label == "press until it fires" else
                 (_w(w.duration, np.asarray(w.pos) - [0, 0, _X_DZ - PRESS_DZ], w.finger, w.label, w.yaw)
                  if w.label == "hold the press" else w) for w in right]
    if _STEM == "w":
        near, yaw = site_world(seed, _NEAR_S)
        dp = near - site_world(seed)[0]
        side = np.array([math.cos(yaw), math.sin(yaw), 0.0]) * PRESS_DZ * math.tan(_TILT)   # toward the knuckle
        right = [_w(w.duration, np.asarray(w.pos) + dp + (side if w.label in ("press until it fires", "hold the press")
                                                          else 0.0), w.finger, w.label, w.yaw)
                 if w.label in _PRESS else w for w in right]
    if _STEM in ("o", "j", "x", "w"):
        left = _one_arm(right)
    if _STEM == "f":
        out = []
        for w in right:
            out.append(w)
            if w.label == "hold":    # the twist is done: the tab is off, the lancet still in the stand
                out.append(_w(1.5, np.asarray(w.pos) - [0, 0, PRESS_DZ], w.finger, "press it down in the stand", w.yaw))
                out.append(_w(0.6, w.pos, w.finger, "back up in the stand", w.yaw))   # 2.1 s in all
        right = out
        # WHY: the left waits as long again, so it still holds the finger through the right's press
        left = [_w(w.duration + 2.1, w.pos, w.finger, w.label, w.yaw) if w.label == "wait parked" else w for w in left]
    if _STEM == "k":
        i = [w.label for w in right].index("let go")
        over = right[i - 1]
        right = right[:i] + [_w(1.5, (over.pos[0], over.pos[1], _KEEP_Z), GRIP_L, "into the opening, held", over.yaw),
                             _w(4.0, (over.pos[0], over.pos[1], _KEEP_Z), GRIP_L, "keep holding it there", over.yaw)]
    if _STEM == "i":
        d, grip = _index_hold(seed)
        opn = -(FINGERS["index"][4] + _I_OPEN)
        left = [_w(w.duration, np.asarray(w.pos) + d, {GRIP_R: grip, OPEN_R: opn}.get(w.finger, w.finger), w.label,
                   w.yaw) if w.label in _HOLD else w for w in left]
    return right, left


def demo(delay=0.0):  # noqa: F811
    seed = run_seed()
    right, left = _stem(*plan(delay, seed, -TWIST if _STEM == "c" else TWIST), seed)
    return {"right": right, "left": left}
