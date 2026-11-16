

# -- appended by sim/mk.sh to the sim copies only: a per-step trace of the clipper, the stand, both hands and the model -------------
import os as _os  # noqa: E402

_TRACE = {"f": None, "n": 0}


@on_step
def _trace(env):
    try:
        _TRACE["n"] += 1
        if _TRACE["n"] % 2:
            return
        base = env.unwrapped
        tr, cl = base.scene["trainer"], base.scene["clipper"]
        origin = _np(base.scene.env_origins[0])
        names = list(tr.body_names)
        pp = _np(tr.data.body_link_pose_w)[0][names.index("stand")]
        pc = _np(cl.data.root_link_pose_w)[0]
        qc = _np(cl.data.joint_pos)[0]
        bq = float(qc[_ids(cl, "clipper", ("blade",))["blade"]])
        tips = claw_tips(pp[:3] - origin, pp[3:7], _M["removed"])
        cpos = pc[:3] - origin
        Rc = _rot3(pc[3:7])
        ring = cpos + Rc @ np.array([RING_X, 0.0, 0.0])
        k = int(np.argmin([math.hypot(t[0] - ring[0], t[1] - ring[1]) for t in tips]))
        inside, prot = in_ring(tips[k], cpos, pc[3:7])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(Rc[2, 2])))))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(Rc[2, 0])))))
        if _TRACE["f"] is None:
            path = _os.path.join(_os.path.dirname(__file__), "trace_" + _os.path.basename(__file__)[:-3] + ".csv")
            _TRACE["f"] = open(path, "w")
            _TRACE["f"].write("step,cx,cy,cz,tilt,pitch,blade,near,dx,dy,prot,inside,stand_dx,stand_dy,stand_dz,"
                              "r0,r1,r2,r3,cuts,stage,fail,model_err,dR,fR,dL,fL,clear,resist\n")
        m, mem = _M, _MEM
        st = mem["now"]["stage"] if mem.get("now") else -1
        dp = pp[:3] - origin - np.array([0.0, 0.0, TOP])
        row = [_TRACE["n"], round(float(cpos[0]), 4), round(float(cpos[1]), 4), round(float(cpos[2]) - TOP, 4),
               round(tilt, 2), round(pitch, 2), round(bq, 4), k + 1, round(float(ring[0] - tips[k][0]), 4),
               round(float(ring[1] - tips[k][1]), 4), round(prot, 4), int(inside),
               round(float(dp[0]), 4), round(float(dp[1]), 4), round(float(dp[2]), 4)]
        row += [round(r * 1000, 2) for r in m["removed"]] + [sum(m["cuts"]), st,
                str(mem.get("fail")).replace(",", ";"), str(m["error"]).replace(",", ";")]
        hs = m.get("hands") or []
        for tcp, f in hs:
            row += [round(float(np.linalg.norm(np.asarray(tcp) - cpos)) * 1000, 1), round(f * 1000, 2)]
        row += ["", ""] * (2 - len(hs)) + [int(hands_clear(m, cpos)), m.get("resist", 0.0)]
        _TRACE["f"].write(",".join(str(v) for v in row) + "\n")
        _TRACE["f"].flush()
    except Exception as e:  # noqa: BLE001
        if _TRACE["f"] is not None:
            _TRACE["f"].write(f"# trace error {e!r}\n")
