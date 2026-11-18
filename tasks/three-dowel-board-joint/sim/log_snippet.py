

# -- appended by sim/mk.sh to the sim copies only: a per-step trace of both boards, the joint and both hands ------------
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
        a, b = (base.scene[n] for n in ("dowel_board", "hole_board"))
        pa, pb = (_np(o.data.root_link_pose_w)[0] for o in (a, b))
        va, vb = (_np(o.data.root_link_vel_w)[0] if hasattr(o.data, "root_link_vel_w") else _np(o.data.root_vel_w)[0]
                  for o in (a, b))
        m = measure(pa[:3] - origin, pa[3:7], pb[:3] - origin, pb[3:7])
        hands = []
        for side in ("left", "right"):
            rob = base.scene[f"{side}_robot"]
            g = _np(rob.data.body_link_pose_w)[0][list(rob.body_names).index("gripper")]
            q = _np(rob.data.joint_pos)[0]
            tcp = g[:3] - origin + _rot3(g[3:7]) @ np.array(TCP_OFFSET)
            hands += [*tcp, float(q[list(rob.joint_names).index("joint7")])]
        n = _MEM.get("now") or {}
        if _TRACE["f"] is None:
            # WHY sim/out, not next to the copy in sim/arm: sim/out is the folder rr.sh collects after a run (the
            # 2026-11-18 probe's traces were left behind in sim/arm)
            out = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "out")
            _os.makedirs(out, exist_ok=True)
            # WHY the layout and seed in the name: two seeds of one stem must not overwrite each other's trace
            path = _os.path.join(out, f"trace_{_os.path.basename(__file__)[:-3]}_L{layout_index(run_seed())}"
                                      f"_r{run_seed()}.csv")
            _TRACE["f"] = open(path, "w")
            _TRACE["f"].write("step,ax,ay,az,ayaw,bx,by,bz,byaw,g0,g1,dx,dz,relyaw,tilta,tiltb,d0,d1,d2,o0,o1,o2,"
                              "va,vb,lx,ly,lz,lf,rx,ry,rz,rf,rung,clear,still,held,lineup,err\n")
        mm = lambda v: round(1000 * float(v), 3)  # noqa: E731
        yaw = lambda q: round(math.degrees(math.atan2(_rot3(q)[1, 0], _rot3(q)[0, 0])), 3)  # noqa: E731
        row = [_TRACE["n"], mm(pa[0] - origin[0]), mm(pa[1] - origin[1]), mm(pa[2] - origin[2] - TOP), yaw(pa[3:7]),
               mm(pb[0] - origin[0]), mm(pb[1] - origin[1]), mm(pb[2] - origin[2] - TOP), yaw(pb[3:7]),
               mm(m["gaps"][0]), mm(m["gaps"][1]), mm(m["dx"]), mm(m["dz"]), round(math.degrees(m["yaw"]), 3),
               round(m["tilt"][0], 3), round(m["tilt"][1], 3), *[mm(v) for v in m["depth"]], *[mm(v) for v in m["off"]],
               mm(np.linalg.norm(va[:3])), mm(np.linalg.norm(vb[:3])),
               *[mm(hands[i] - (TOP if i % 4 == 2 else 0.0)) if i % 4 != 3 else round(hands[i], 5) for i in range(8)],
               n.get("rung", -1), int(bool(n.get("clear"))), int(bool(n.get("still"))), round(_MEM["held"], 2),
               len(_SERVO["done"]),
               str(_M["error"]).replace(",", ";")]
        _TRACE["f"].write(",".join(str(v) for v in row) + "\n")
        _TRACE["f"].flush()
    except Exception as e:  # noqa: BLE001
        if _TRACE["f"] is not None:
            _TRACE["f"].write(f"# trace error {e!r}\n")
