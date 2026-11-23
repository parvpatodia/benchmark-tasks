"""Scoring and demo checks for load-instant-film that run without the simulator: python -m pytest -q"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import load_instant_film as M
from mechanism import MechanismState

HERE = Path(__file__).resolve().parent
Z3 = np.zeros(3)


def pads(x, y, z) -> M.Pads:
    return M.Pads(back=np.array([x, y, z + 0.04]), tips=np.array([x, y, z]))


AWAY = {"left_gripper": pads(*M.PARK_LEFT), "right_gripper": pads(*M.PARK_RIGHT)}


def quat_of(r: np.ndarray) -> np.ndarray:
    """A rotation matrix's quaternion (x, y, z, w)."""
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    if w > 1e-6:
        return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    q = np.zeros(4)
    q[i] = math.sqrt(max(0.0, 1 + r[i, i] - r[j, j] - r[k, k])) / 2
    q[j] = (r[j, i] + r[i, j]) / (4 * q[i])
    q[k] = (r[k, i] + r[i, k]) / (4 * q[i])
    q[3] = (r[k, j] - r[j, k]) / (4 * q[i])
    return q


LOADED = np.column_stack([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]).astype(float)  # length up, width to -x... back out
SEATED_POS = np.array([M.CH_X, M.CH_WALL - M.PACK[2] / 2, M.TOP + M.CH_Z0 + M.PACK[0] / 2])


def pack(pos=SEATED_POS, r=LOADED, still=True) -> ObjectState:
    v = Z3 if still else np.array([0.05, 0, 0])
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=quat_of(r), lin_vel=v, ang_vel=Z3, low=p - 0.05, high=p + 0.05)


def camera(cover=0.0, lock=0.0, ring=0.0, shutter=0.0, sheet=0.0) -> MechanismState:
    return MechanismState(pos=np.array([0, 0, M.TOP]), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=Z3, ang_vel=Z3,
                          low=Z3, high=Z3, joints={"back_cover": cover, "lock": lock, "lens_ring": ring,
                                                   "shutter": shutter, "film_cover": sheet})


def objs(p=None, cam=None, rec=None, hands=None):
    return {"film_pack": p or pack(), "camera": cam or camera(), "film_record": rec or M.FilmRecord(),
            **(hands or AWAY)}


ON = -math.radians(40)


def test_task_file_loads():
    load(HERE / "load_instant_film.py")


def test_loaded_frame_is_a_rotation_and_right_way():
    assert np.allclose(LOADED @ LOADED.T, np.eye(3)) and np.isclose(np.linalg.det(LOADED), 1.0)
    assert M.right_way(pack()) and M.seated(pack())


def test_carry_brings_yellow_end_up():
    r = M._carry_turn() @ np.array([[math.cos(M.PACK_YAW), -math.sin(M.PACK_YAW), 0],
                                    [math.sin(M.PACK_YAW), math.cos(M.PACK_YAW), 0], [0, 0, 1]])
    assert r[2, 0] > 0.99 and -r[1, 2] > 0.99


def test_scores_in_order():
    done = M.FilmRecord(locked=True, ejected=True, presses=1)
    assert M.grade(objs(cam=camera(ring=ON, sheet=M.EJECT), rec=done)) == 1.0
    assert M.score(objs(cam=camera(ring=ON, sheet=M.EJECT), rec=done)) == M.COMPLETE_SCORE
    assert M.grade(objs(cam=camera(ring=ON))) == 0.75
    assert M.grade(objs(cam=camera())) == 0.5
    assert M.grade(objs(cam=camera(cover=math.radians(40)), rec=M.FilmRecord(locked=False))) == 0.25
    assert M.grade(objs(p=pack(pos=M.PACK_START, r=np.eye(3)))) == 0.0


def test_hand_near_camera_holds_full_score_back():
    done = M.FilmRecord(locked=True, ejected=True, presses=1)
    near = {**AWAY, "left_gripper": pads(M.CX, M.YF + 0.03, M.TOP + 0.09)}
    assert M.grade(objs(cam=camera(ring=ON, sheet=M.EJECT), rec=done, hands=near)) == 0.75


def test_wrong_way_round_scores_zero():
    upside_down = np.column_stack([[0, 0, -1], [1, 0, 0], [0, -1, 0]]).astype(float)  # yellow end down
    back_in = np.column_stack([[0, 0, 1], [1, 0, 0], [0, 1, 0]]).astype(float)  # holes towards the lens
    for r in (upside_down, back_in):
        p = pack(r=r)
        assert M.in_chamber(p) and not M.right_way(p)
        assert M.grade(objs(p=p, cam=camera(ring=ON))) == 0.0


def test_not_pushed_home_scores_zero():
    proud = SEATED_POS + np.array([0, -0.004, 0])
    assert M.grade(objs(p=pack(pos=proud))) == 0.0


def test_chamber_has_no_finger_notches():
    """The frame round the chamber is solid: every point of both side walls, at every height, is inside a body box."""
    boxes = [sh.bounds()
             for sh in M.CAMERA.base.shapes if sh.kind == "box"]
    cx0, cx1 = M.CH_X - M.CH_W / 2, M.CH_X + M.CH_W / 2
    for x in np.r_[np.linspace(M.X0 + 1e-4, cx0 - 1e-4, 6), np.linspace(cx1 + 1e-4, M.X1 - 1e-4, 6)]:
        for y in np.linspace(M.YB + 1e-4, M.CH_WALL - 1e-4, 5):
            for z in np.linspace(1e-4, M.H - 1e-4, 60):
                q = np.array([x, y, z])
                assert any(np.all(q >= lo) and np.all(q <= hi) for lo, hi in boxes), q
    assert M.CH_W - M.PACK[1] == pytest.approx(0.006)  # 3 mm play each side: no room for an 8.5 mm finger


def test_finger_beside_seated_pack_hits_the_frame():
    """A fingertip box (8.5 mm thick) against a seated pack's long side, reaching any depth past the mouth, is inside
    the frame: the gripper cannot carry the pack home by its long sides."""
    for depth in (0.001, 0.005, 0.016):
        y = M.YB + depth
        x = M.CH_X + M.PACK[1] / 2 + 0.0085 / 2  # the finger's middle, against the pack's side
        q = np.array([x, y, M.CH_Z])
        boxes = [sh.bounds()
                 for sh in M.CAMERA.base.shapes if sh.kind == "box"]
        assert any(np.all(q + np.array([0.004, 0, 0]) >= lo) and np.all(q + np.array([0.004, 0, 0]) <= hi)
                   for lo, hi in boxes)


def _box_pads(local_box, pack_r=LOADED, pack_pos=SEATED_POS) -> M.Pads:
    """Pads whose finger points cover a box given in the seated pack's frame (m)."""
    pts = M._box_surface(local_box)
    world = pts @ pack_r.T + pack_pos
    return M.Pads(back=world[0], tips=world[0], finger_points=world)


def test_finger_touching_a_hole_scores_zero_and_press_away_is_fine():
    top = M.PACK[2] / 2
    (hx, hy), (sx, sy) = M.HOLES[0]
    # an 8.5 x 36 mm fingertip face resting on the back over a hole, and one just covering the hole's edge
    on = _box_pads(((hx - 0.004, hx + 0.004), (hy - 0.018, hy + 0.018), (top, top + 0.03)))
    edge = _box_pads(((hx + sx / 2 - 0.0002, hx + sx / 2 + 0.008), (hy - 0.018, hy + 0.018), (top, top + 0.03)))
    # hovering 0.4 mm above a hole still counts; 1 mm above does not
    hover = _box_pads(((hx - 0.004, hx + 0.004), (hy - 0.003, hy + 0.003), (top + 0.0004, top + 0.03)))
    above = _box_pads(((hx - 0.004, hx + 0.004), (hy - 0.003, hy + 0.003), (top + 0.001, top + 0.03)))
    # the demo's press: the closed fingertips on the back 28-45 mm from the middle along the length, i.e. well clear
    press = _box_pads(((0.028, 0.045), (-0.018, 0.018), (top, top + 0.03)))
    p = pack()
    assert M.presses_holes(on, p) and M.presses_holes(edge, p) and M.presses_holes(hover, p)
    assert not M.presses_holes(above, p) and not M.presses_holes(press, p)
    # a finger beside the pack's long side, below its back (the regrip's place) does not touch a hole
    side = _box_pads(((0.010, 0.046), (M.PACK[1] / 2, M.PACK[1] / 2 + 0.0085), (top - 0.012, top + 0.03)))
    assert not M.presses_holes(side, p)
    rec = M.FilmRecord(holes_pressed=True, locked=True, ejected=True, presses=1)
    assert M.grade(objs(cam=camera(ring=ON, sheet=M.EJECT), rec=rec)) == 0.0


def test_holes_and_forced_lock_score_zero():
    for rec in (M.FilmRecord(holes_pressed=True), M.FilmRecord(forced=True)):
        assert M.grade(objs(cam=camera(ring=ON), rec=rec)) == 0.0


def test_fingertip_point_on_a_hole_is_seen():
    p = pack()
    (hx, hy), _ = M.HOLES[0]
    local = np.array([hx, hy, M.PACK[2] / 2 + 0.0003])
    q = LOADED @ local + SEATED_POS
    on_hole = M.Pads(back=q, tips=q, fingertips=(q,))
    margin = LOADED @ np.array([0.030, 0, M.PACK[2] / 2 + 0.0003]) + SEATED_POS
    on_margin = M.Pads(back=margin, tips=margin, fingertips=(margin,))
    assert M.presses_holes(on_hole, p) and not M.presses_holes(on_margin, p)


def test_lock_logic():
    rec = {"locked": True, "popping": False, "forced": False}
    assert M.lock_step(rec, 0.0, 0.0) == 0.0 and rec["locked"]
    assert M.lock_step(rec, math.radians(8), 0.0) == 0.0 and rec["forced"]
    rec = {"locked": True, "popping": False, "forced": False}
    assert M.lock_step(rec, 0.0, -0.0025) == pytest.approx(math.radians(M.POP_DEG)) and not rec["locked"]
    popped = math.radians(M.POP_DEG - 1)
    assert M.lock_step(rec, popped, 0.0) == pytest.approx(popped)  # popped: now free
    assert M.lock_step(rec, math.radians(60), 0.0) == pytest.approx(math.radians(60))
    assert M.lock_step(rec, math.radians(1.5), -0.0025) == pytest.approx(math.radians(1.5))  # tab held down: no lock
    assert M.lock_step(rec, math.radians(1.5), 0.0) == 0.0 and rec["locked"]


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_reach(arm):
    plan = ArmPlan(M.demo()[arm], LEFT_BASE_POS if arm == "left" else RIGHT_BASE_POS)
    worst_mm, worst_deg, worst_at = 0.0, 0.0, ""
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
        if plan.worst_ik_mm > worst_mm or plan.worst_ik_deg > worst_deg:
            worst_mm, worst_deg = plan.worst_ik_mm, plan.worst_ik_deg
            worst_at = plan.target(t)[3]
    print(f"{arm}: worst IK {worst_mm:.1f} mm, {worst_deg:.1f} deg at {worst_at!r}")
    assert worst_mm < 3.0 and worst_deg < 3.0, (worst_mm, worst_deg, worst_at)


# -- the planned demo against the planned pack: finger link geometry from the URDF's kinematics -------------------------
def _finger_clouds(plan_waypoints, base_pos, labels, held_width=None):
    """For each control step whose label is in ``labels``: (label, both fingers' surface points in the world). With
    ``held_width``, the fingers stop that far apart, as on the pack they hold (the plan commands them fully shut)."""
    from yam_arena import YAM_URDF
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.urdf import forward_kinematics

    plan = ArmPlan(plan_waypoints, base_pos)
    out = []
    for t in np.arange(0.0, plan.duration, 0.05):
        _, _, finger, label = plan.target(t)
        q = plan.action7(t)[0][:6]
        if label not in labels:
            continue
        if held_width is not None:
            finger = min(finger, -held_width / 2)
        poses = forward_kinematics(YAM_URDF, {**{n: float(v) for n, v in zip(ARM_JOINTS, q)}, "joint7": float(finger),
                                              "joint8": float(finger)})
        cloud = np.concatenate([pts @ poses[link][:3, :3].T + poses[link][:3, 3] + np.asarray(base_pos)
                                for link, pts in M.FINGER_POINTS.items()])
        out.append((label, cloud))
    return out


def _set_pack(tilt_deg=None) -> ObjectState:
    """The pack where the demo sets it down (tilted, its bottom outer edge on the chamber floor)."""
    tilt = M.LOAD_TILT_DEG if tilt_deg is None else tilt_deg
    r = M.axis_angle_matrix(np.eye(3)[0], math.radians(tilt)) @ LOADED
    centre = M._bo_set() + M.axis_angle_matrix(np.eye(3)[0], math.radians(tilt)) @ np.array([0.0, M.PACK[2] / 2, M.PACK[0] / 2])
    return pack(pos=centre, r=r)


LOAD_LABELS = {"bottom edge in, over the floor", "set it down on the floor", "let go: it falls in upright"}
PRESS_LABELS = {"press it flat", "hold the press"}


def test_set_pack_pose_is_on_the_floor_and_right_way():
    p = _set_pack()
    assert M.right_way(p) and M.in_chamber(p)
    upright = _set_pack(0.0)
    assert M.seated(upright)


def test_demo_fingers_stay_outside_the_chamber_while_loading():
    """Holding the tilted pack, every point of both fingers is outside the camera's body boxes and behind its back."""
    boxes = [(lo + [0, 0, M.TOP], hi + [0, 0, M.TOP]) for lo, hi in
             (sh.bounds() for sh in M.CAMERA.base.shapes if sh.kind == "box")]
    clouds = _finger_clouds(M.demo()["right"], RIGHT_BASE_POS, LOAD_LABELS, held_width=M.PACK[1])
    assert clouds
    worst = -1.0
    for label, cloud in clouds:
        near = cloud[np.abs(cloud[:, 0] - M.CX) <= M.W / 2 + 0.001]
        worst = max(worst, float(near[:, 1].max()) - M.YB) if len(near) else worst
        for lo, hi in boxes:
            inside = np.all((cloud > lo + 0.0005) & (cloud < hi - 0.0005), axis=1)
            assert not inside.any(), (label, cloud[inside][:3])
    print(f"deepest finger point over the camera's width: {worst * 1000:.1f} mm past the back")
    assert worst < 0.0


def test_demo_fingers_never_touch_the_holes_and_hole_probe_does(monkeypatch):
    seated, tilted = pack(), _set_pack()
    for label, cloud in (_finger_clouds(M.demo()["right"], RIGHT_BASE_POS, LOAD_LABELS, held_width=M.PACK[1])
                         + _finger_clouds(M.demo()["right"], RIGHT_BASE_POS, PRESS_LABELS)):
        p = tilted if label in LOAD_LABELS else seated
        assert not M.presses_holes(M.Pads(back=cloud[0], tips=cloud[0], finger_points=cloud), p), label
    monkeypatch.setenv("FILM_DEMO", "presshole")
    hits = [M.presses_holes(M.Pads(back=c[0], tips=c[0], finger_points=c), seated)
            for _, c in _finger_clouds(M.demo()["right"], RIGHT_BASE_POS, PRESS_LABELS)]
    assert any(hits)


def test_press_is_clear_of_holes_by_margin():
    """The press's fingertips (36 mm tall, closed: two 8.5 mm fingers stacked) end above the holes' top."""
    holes_top = M.PACK[0] / 2 + max(c[0] + s[0] / 2 for c, s in M.HOLES)  # up the pack from its bottom
    assert M.PRESS_H - 0.0085 - 0.0005 > holes_top + 0.005
