

# -- appended by sim/mk.sh to the sim copies only: a per-step trace of the lancet, the tab, the hand and the model -------
import os as _os  # noqa: E402

_TRACE = {"f": None, "n": 0}


@on_step
def _trace(env):
    try:
        _TRACE["n"] += 1
        if _TRACE["n"] % 2:
            return
        if _TRACE["f"] is None:
            path = _os.path.join(_os.path.dirname(__file__), "trace_" + _os.path.basename(__file__)[:-3] + ".csv")
            _TRACE["f"] = open(path, "w")
            _TRACE["f"].write("step,q_mm,force,tab_deg,theta_deg,lift_mm,tab_off,out,positioned,fired,fire,free_t,"
                              "rx,ry,rz,rf,lx,ly,lz,lf,stage,fail,model_err\n")
        m, mem = _M, _MEM
        st = mem["now"]["stage"] if mem.get("now") else -1
        r = m["right"] if m["right"] else (np.full(3, np.nan), float("nan"))
        lh = m["left"] if m["left"] else (np.full(3, np.nan), float("nan"))
        r4 = lambda v: round(float(v), 4)  # noqa: E731
        row = [_TRACE["n"], round(1000 * m["q"], 3), round(shield_force(m["q"]), 2), round(math.degrees(m["tab"]), 2),
               round(math.degrees(m["theta"]), 3), round(1000 * wrist_lift(m["theta"]), 3), int(m["tab_off"]),
               int(m["out"]), int(m["positioned"]), int(m["fired"]), str(m["fire"]).replace(",", ";"),
               round(m["free_t"], 2), *[r4(v) for v in r[0]], round(r[1], 5), *[r4(v) for v in lh[0]],
               round(lh[1], 5), st, str(mem.get("fail")).replace(",", ";"), str(m["error"]).replace(",", ";")]
        _TRACE["f"].write(",".join(str(v) for v in row) + "\n")
        _TRACE["f"].flush()
    except Exception as e:  # noqa: BLE001
        if _TRACE["f"] is not None:
            _TRACE["f"].write(f"# trace error {e!r}\n")
