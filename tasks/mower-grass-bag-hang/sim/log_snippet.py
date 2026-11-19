

# -- appended by sim/mk.sh to the sim copies only: a per-step trace of the bag, the rod in the mower's frame, the flap,
# the mower's roll, both hands and the score's rung --------------------------------------------------------------------
import os as _os  # noqa: E402

_TRACE = {"f": None, "n": 0}


@on_step
def _trace(env):
    try:
        _TRACE["n"] += 1
        if _TRACE["n"] % 2:
            return
        base = env.unwrapped
        origin = _np(base.scene.env_origins)[0]
        pb = _np(base.scene["bag"].data.root_link_pose_w)[0]
        bd = base.scene["bag"].data
        vb = _np(bd.root_link_vel_w)[0] if hasattr(bd, "root_link_vel_w") else _np(bd.root_vel_w)[0]
        mw = base.scene["mower"]
        q = _np(mw.data.joint_pos)[0]
        names = list(mw.joint_names)
        roll, theta = float(q[names.index("roll")]), float(q[names.index("flap")])
        Rm, pm = np.eye(3), np.array([X_M + roll, 0.0, TOP])
        bag_p, bag_q = pb[:3] - origin, pb[3:7]
        c, u = rod_in_mower(bag_p, bag_q, Rm, pm)
        hk = {s: hook_check(c, u, s)["ok"] for s in (1, -1)}
        gap, _, on = flap_edge_gap(theta, bag_p, bag_q, Rm, pm)
        Rb = _rot3(bag_q)
        tilt = math.degrees(math.asin(max(-1.0, min(1.0, float(Rb[2, 0])))))
        yaw = math.degrees(math.atan2(Rb[1, 0], Rb[0, 0]))
        hands = []
        for side in ("left", "right"):
            rob = base.scene[f"{side}_robot"]
            g = _np(rob.data.body_link_pose_w)[0][list(rob.body_names).index("gripper")]
            qq = _np(rob.data.joint_pos)[0]
            tcp = g[:3] - origin + _rot3(g[3:7]) @ np.array(TCP_OFFSET)
            hands += [*tcp, float(qq[list(rob.joint_names).index("joint7")])]
        n = _MEM.get("now") or {}
        m = n.get("m") or {}
        if _TRACE["f"] is None:
            # WHY sim/out, not next to the copy in sim/arm: sim/out is the folder collected after a run
            out = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "out")
            _os.makedirs(out, exist_ok=True)
            # WHY the seed in the name: two seeds of one stem must not overwrite each other's trace
            path = _os.path.join(out, f"trace_{_os.path.basename(__file__)[:-3]}_r{run_seed()}.csv")
            _TRACE["f"] = open(path, "w")
            _TRACE["f"].write("step,bx,by,bz,byaw,btilt,cx,cy,cz,hookL,hookR,theta,gap,on,roll,vb,dbag,dflap,"
                              "lx,ly,lz,lf,rx,ry,rz,rf,rung,held,lineup,err\n")
        mm = lambda v: round(1000 * float(v), 2)  # noqa: E731
        row = [_TRACE["n"], mm(bag_p[0]), mm(bag_p[1]), mm(bag_p[2] - TOP), round(yaw, 3), round(tilt, 3),
               mm(c[0]), mm(c[1]), mm(c[2]), int(hk[1]), int(hk[-1]), round(math.degrees(theta), 2), mm(gap),
               int(on), mm(roll), mm(np.linalg.norm(vb[:3])), mm(m.get("d_bag", -1)), mm(m.get("d_flap", -1)),
               *[mm(hands[i] - (TOP if i % 4 == 2 else 0.0)) if i % 4 != 3 else round(hands[i], 5) for i in range(8)],
               n.get("rung", -1), round(_MEM["held"], 2), len(_SERVO["done"]),
               str(_M["error"]).replace(",", ";")]
        _TRACE["f"].write(",".join(str(v) for v in row) + "\n")
        _TRACE["f"].flush()
    except Exception as e:  # noqa: BLE001
        if _TRACE["f"] is not None:
            _TRACE["f"].write(f"# trace error {e!r}\n")
