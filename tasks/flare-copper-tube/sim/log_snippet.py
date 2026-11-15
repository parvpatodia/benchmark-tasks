

# -- appended by sim/mk.sh to the sim copies only: a per-step trace of the tool, the tube and the model ---------------
import os as _os  # noqa: E402

_TRACE = {"f": None, "n": 0}


@on_step
def _trace(env):
    try:
        _TRACE["n"] += 1
        if _TRACE["n"] % 5:
            return
        base = env.unwrapped
        tool, tube_a = base.scene["tool"], base.scene["tube"]
        q = _np(tool.data.joint_pos)[0]
        ids = _ids(tool)
        origin = _np(base.scene.env_origins[0])
        pose = _np(tube_a.data.root_link_pose_w)[0]
        top, xy, up = tube_top(pose[:3] - origin, pose[3:7])
        if _TRACE["f"] is None:
            path = _os.path.join(_os.path.dirname(__file__), "trace_" + _os.path.basename(__file__)[:-3] + ".csv")
            _TRACE["f"] = open(path, "w")
            _TRACE["f"].write("step,hinge,slide,feed,cone_z,clamp_turn,clamp_x,tube_x,tube_y,tube_top,up,tip_z,"
                              "formed,p,centre_err,iron,tight,pushing,stage,fail,model_err,r_zmin,r_x,r_y,l_zmin\n")
        m, mem = _M, _MEM
        st = mem["now"]["stage"] if mem.get("now") else -1
        row = [_TRACE["n"]] + [round(float(q[ids[j]]), 5) for j in JOINTS] + [
            round(xy[0], 5), round(xy[1], 5), round(top - Z_BAR, 5), int(up), round(m["tip_z"] - Z_BAR, 5),
            int(m["formed"]), m["p"], m["centre_err"], round(m["iron"], 3), int(m["tight"]), int(m["pushing"]), st,
            str(mem.get("fail")).replace(",", ";"), str(m["error"]).replace(",", ";")]
        hs = _HANDS["hands"] or []
        half = len(hs) // 2
        if half:
            rv = np.vstack([h[0] for h in hs[half:]])
            lv = np.vstack([h[0] for h in hs[:half]])
            row += [round(float(rv[:, 2].min()) - Z_BAR, 5), round(float(rv[:, 0].mean()) - BX, 5),
                    round(float(rv[:, 1].mean()) - HY, 5), round(float(lv[:, 2].min()) - Z_BAR, 5)]
        else:
            row += ["", "", "", ""]
        _TRACE["f"].write(",".join(str(v) for v in row) + "\n")
        _TRACE["f"].flush()
    except Exception as e:  # noqa: BLE001
        if _TRACE["f"] is not None:
            _TRACE["f"].write(f"# trace error {e!r}\n")
