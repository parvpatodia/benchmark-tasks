

# -- appended by sim/mk.sh to the sim copies only: a per-step trace of the tube, the handle and the model -------------
import os as _os  # noqa: E402

_TRACE = {"f": None, "n": 0}


@on_step
def _trace(env):
    try:
        _TRACE["n"] += 1
        if _TRACE["n"] % 2:
            return
        base = env.unwrapped
        tube, mug, han = base.scene["tube"], base.scene["mug"], base.scene["handle"]
        origin = _np(base.scene.env_origins)[0]
        pt, pm, ph = (_np(a.data.root_link_pose_w)[0] for a in (tube, mug, han))
        qt = _np(tube.data.joint_pos)[0]
        tid = _ids(tube, "tube", ("wall_a", "wall_b"))
        qa, qb = float(qt[tid["wall_a"]]), float(qt[tid["wall_b"]])
        tp, mp, hp = pt[:3] - origin, pm[:3] - origin, ph[:3] - origin
        tip = nozzle_w(tp, pt[3:7])
        faces = faces_w(mp, pm[3:7], _M["stubs"])
        ends = ends_w(hp, ph[3:7])
        Rm = _rot3(pm[3:7])
        de = [Rm.T @ (e - f) for e, f in zip(ends, faces)]
        k = int(np.argmin([np.linalg.norm(tip[:2] - f[:2]) for f in faces]))
        dt_ = Rm.T @ (tip - faces[k])
        tz = float(_rot3(pt[3:7])[2, 2])
        hz = float(_rot3(ph[3:7])[2, 2])
        if _TRACE["f"] is None:
            path = _os.path.join(_os.path.dirname(__file__), "trace_" + _os.path.basename(__file__)[:-3] + ".csv")
            _TRACE["f"] = open(path, "w")
            _TRACE["f"].write("step,tx,ty,tz,t_up,qa,qb,force,face,tip_dx,tip_dy,tip_dz,glue0,glue1,stray,"
                              "e0x,e0y,e0z,e1x,e1y,e1z,h_up,mx,my,mz,tcpx,tcpy,tcpz,finger,fitted,joined,let_go,"
                              "stage,fail,model_err,mroll,s0,s1,f0,f1,parked,pressed,lx,ly,lz,lfinger\n")
        m, mem = _M, _MEM
        st = mem["now"]["stage"] if mem.get("now") else -1
        tcp = m["tcp"] if m["tcp"] is not None else np.full(3, np.nan)
        r4 = lambda v: round(float(v), 4)  # noqa: E731
        row = [_TRACE["n"], r4(tp[0]), r4(tp[1]), r4(tp[2] - TOP), round(tz, 4), round(qa, 5), round(qb, 5),
               round(squeeze_force(qa, qb), 2), k, r4(dt_[0]), r4(dt_[1]), r4(dt_[2]),
               round(m["glue"][0], 2), round(m["glue"][1], 2), round(m["stray"], 2),
               *[r4(v) for v in de[0]], *[r4(v) for v in de[1]], round(hz, 4),
               r4(mp[0]), r4(mp[1]), r4(mp[2] - TOP), r4(tcp[0]), r4(tcp[1]), r4(tcp[2] - TOP),
               round(m["finger"] if m["finger"] is not None else float("nan"), 5),
               int(m["fitted"]), int(m["joined"]), int(m["let_go"]), st,
               str(mem.get("fail")).replace(",", ";"), str(m["error"]).replace(",", ";"),
               round(math.degrees(math.atan2(Rm[0, 2], Rm[2, 2])), 2), round(1000 * m["stubs"][0], 3), round(1000 * m["stubs"][1], 3),
               round(stub_force(m["stubs"][0]), 2), round(stub_force(m["stubs"][1]), 2),
               int(m["parked"]), int(m["pressed"]),
               *([r4(v) for v in m["left"][0]] + [round(m["left"][1], 5)] if m["left"] else [""] * 4)]
        _TRACE["f"].write(",".join(str(v) for v in row) + "\n")
        _TRACE["f"].flush()
    except Exception as e:  # noqa: BLE001
        if _TRACE["f"] is not None:
            _TRACE["f"].write(f"# trace error {e!r}\n")
