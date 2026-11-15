

# -- appended by sim/mk.sh to the sim copies only: a per-step trace of the tools, the joint's face and the model --------
import os as _os  # noqa: E402

_TRACE = {"f": None, "n": 0}


@on_step
def _trace(env):
    try:
        _TRACE["n"] += 1
        if _TRACE["n"] % 5:
            return
        base = env.unwrapped
        bench, tor, wir = base.scene["bench"], base.scene["torch"], base.scene["wire"]
        q = _np(bench.data.joint_pos)[0]
        ids = _ids(bench)
        origin = _np(base.scene.env_origins[0])
        pt = _np(tor.data.root_link_pose_w)[0]
        pw = _np(wir.data.root_link_pose_w)[0]
        ft = flame_tip(pt[:3] - origin, pt[3:7])
        wt = wire_tip(pw[:3] - origin, pw[3:7])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, _rot3(pw[3:7])[2, 2]))))
        if _TRACE["f"] is None:
            path = _os.path.join(_os.path.dirname(__file__), "trace_" + _os.path.basename(__file__)[:-3] + ".csv")
            _TRACE["f"] = open(path, "w")
            _TRACE["f"].write("step,face,bead,burn,T,fed,on,pressed,flame_d,wire_r,wire_dz,wire_tilt,torch_z,"
                              "stage,fail,model_err,fx,fy,fz,torch_pitch,wx,wy,fedge\n")
        m, mem = _M, _MEM
        st = mem["now"]["stage"] if mem.get("now") else -1
        row = [_TRACE["n"]] + [round(float(q[ids[j]]), 5) for j in JOINTS] + [
            round(m["T"], 1), round(m["fed"], 5), int(m["on"]), int(m["pressed"]),
            round(float(np.linalg.norm(ft - np.asarray(MARK))), 4), round(math.hypot(wt[0] - JX, wt[1]), 5),
            round(float(wt[2]) - (TOP + STOP_TOP), 5), round(tilt, 2), round(float(pt[2] - origin[2]) - TOP, 4), st,
            str(mem.get("fail")).replace(",", ";"), str(m["error"]).replace(",", ";")]
        e = ft - np.asarray(MARK)
        ax = _rot3(pt[3:7]) @ np.array([0.0, -1.0, 0.0])           # the flame's direction
        row += [round(float(e[0]), 4), round(float(e[1]), 4), round(float(e[2]), 4),
                round(math.degrees(math.asin(max(-1.0, min(1.0, float(ax[2]))))), 2),
                round(float(wt[0]) - JX, 5), round(float(wt[1]), 5), round(edge_dist(ft), 4)]
        _TRACE["f"].write(",".join(str(v) for v in row) + "\n")
        _TRACE["f"].flush()
    except Exception as e:  # noqa: BLE001
        if _TRACE["f"] is not None:
            _TRACE["f"].write(f"# trace error {e!r}\n")
