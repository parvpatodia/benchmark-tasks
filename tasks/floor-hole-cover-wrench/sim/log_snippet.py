

# -- appended by sim/mk.sh to the sim copies only: a per-step trace of the cover, the wrench, both hands and the score's
# quantities (every one the rung reads) --------------------------------------------------------------------------------
import os as _os  # noqa: E402

_TRACE = {"f": None, "n": 0}
_TRACE_COLS = ("step,cx,cy,cz,cyaw,gap_max,seat_off,flat,placed,cleats,seated,dropped,latched,"
               "wx,wy,wz,wyaw,low,in_tote,away,in_hole,d_cover,d_wrench,still,"
               "lx,ly,lz,lf,rx,ry,rz,rf,rung,held,lineups,err\n")


@on_step
def _trace(env):
    try:
        _TRACE["n"] += 1
        base = env.unwrapped
        origin = _np(base.scene.env_origins)[0]
        pc = _np(base.scene["cover"].data.root_link_pose_w)[0]
        pw = _np(base.scene["wrench"].data.root_link_pose_w)[0]
        c_p, c_q, w_p, w_q = pc[:3] - origin, pc[3:7], pw[:3] - origin, pw[3:7]
        c, w = cover_check(c_p, c_q), wrench_check(w_p, w_q)
        hands = []
        for side in ("left", "right"):
            rob = base.scene[f"{side}_robot"]
            g = _np(rob.data.body_link_pose_w)[0][list(rob.body_names).index("gripper")]
            qq = _np(rob.data.joint_pos)[0]
            tcp = g[:3] - origin + _rot3(g[3:7]) @ np.array(TCP_OFFSET)
            hands += [float(tcp[0]), float(tcp[1]), float(tcp[2] - TOP),
                      float(qq[list(rob.joint_names).index("joint7")])]
        n = _MEM.get("now") or {}
        m = n.get("m") or {}
        if _TRACE["f"] is None:
            # WHY sim/out, not next to the copy in sim/arm: sim/out is the folder collected after a run
            out = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "out")
            _os.makedirs(out, exist_ok=True)
            # WHY the seed in the name: two seeds of one stem must not overwrite each other's trace
            path = _os.path.join(out, f"trace_{_os.path.basename(__file__)[:-3]}_r{run_seed()}.csv")
            _TRACE["f"] = open(path, "w")
            _TRACE["f"].write(_TRACE_COLS)
        mm = lambda v: round(1000 * float(v), 2)  # noqa: E731
        row = [_TRACE["n"], mm(c_p[0]), mm(c_p[1]), mm(c_p[2] - TOP), round(math.degrees(_yaw_of(_rot3(c_q))), 3),
               mm(c["gap_max"]), mm(c["seat_off"]), int(c["flat"]), int(c["placed"]), int(c["cleats"]),
               int(c["seated"]), int(c["dropped"]), int(_MEM["latched"]),
               mm(w_p[0]), mm(w_p[1]), mm(w_p[2] - TOP), round(math.degrees(_yaw_of(_rot3(w_q))), 3),
               mm(w["low"]), int(w["in_tote"]), int(w["away"]), int(w["in_hole"]),
               mm(m.get("d_cover", -1)), mm(m.get("d_wrench", -1)), int(bool(m.get("still", False))),
               *[mm(v) if i % 4 != 3 else round(v, 5) for i, v in enumerate(hands)],
               n.get("rung", -1), round(_MEM["held"], 2), sum(len(s) for s in _SERVO["done"].values()),
               str(_M["error"]).replace(",", ";")]
        _TRACE["f"].write(",".join(str(v) for v in row) + "\n")
        _TRACE["f"].flush()
    except Exception as e:  # noqa: BLE001
        if _TRACE["f"] is not None:
            _TRACE["f"].write(f"# trace error {e!r}\n")
