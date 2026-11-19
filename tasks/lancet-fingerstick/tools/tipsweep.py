"""tipsweep.py: does an unsteadied trainer hand tip before the lancet reaches a force F, for every press the scorer
allows? Quasi-static, about the hinge (no sim).

A press is allowed when the shield's face is on the site (site_ok) and the lancet's axis d is within TILT_MAX of the
skin's inward normal n (the scorer's square rule). The shield's spring carries the axial load, so the contact force C
on the skin has C . d = F (the force on the shield). Its direction c is set by friction, coefficient up to mu, in
either contact the shield can make: its flat face on the skin (c within atan(mu) of d) or its rim on the skin (c
within atan(mu) of n). So C = F c / (c . d). Its moment about the hinge's axis (y, through (PIVOT_X, HINGE_Z) in the
hand's mechanism frame) is (r x C)_y with r from the hinge to the contact point; negative tips the fingertips down.
The hand's weight holds them up with G = hold_moment(). Per newton of shield force a press tips the hand by
k = -(r x c)_y / (c . d); the least-tipping allowed press gives k_min. Every allowed press at force F tips the hand
when F k_min - G > 0, and that difference is the margin.

The sweep: the ring and middle fingertips, s every 0.5 mm over SITE_S, the angle round every 2.5 deg over SITE_ANG on
the side that faces up; d on rings of 5, 10 and 15 deg round n (24 directions each, plus n itself); c on rings of
atan(mu)/3, 2 atan(mu)/3 and atan(mu) round d and round n (24 each, plus d and n).

    python3 tools/tipsweep.py          # the table the README quotes
"""
import math
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np  # noqa: E402
import lancet_fingerstick as F  # noqa: E402

S_STEP, ANG_STEP, NA, NR = 0.0005, math.radians(2.5), 24, 3


def cone(n, amax, na=NA, nr=NR):
    """Unit directions: n, and nr rings out to amax round it, na each."""
    n = np.asarray(n, float) / np.linalg.norm(n)
    if amax <= 0.0:
        return n[None]
    e1 = np.cross(n, [0.0, 1.0, 0.0])
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.cross(n, [1.0, 0.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    ph = 2 * math.pi * np.arange(na) / na
    out = [n[None]]
    for a in amax * np.arange(1, nr + 1) / nr:
        out.append(math.cos(a) * n + math.sin(a) * (np.cos(ph)[:, None] * e1 + np.sin(ph)[:, None] * e2))
    return np.vstack(out)


def site_point(name, s, ang):
    """A point on a finger's distal pad (mechanism frame) s back from its apex, ang round from its middle line."""
    v, u0, _, rd, _ = F.FINGERS[name]
    rho = math.sqrt(rd * rd - (rd - s) ** 2) if s < rd else rd
    return F.hand_to_mech((u0 + s, v + rho * math.sin(ang), rho * math.cos(ang)))


def sites(side=-1, fingers=F.SITE_FINGERS):
    """(finger, s, ang, point, inward normal) over the site; side -1 is the side that faces up, +1 the other."""
    out = []
    for name in fingers:
        for s in np.arange(F.SITE_S[0], F.SITE_S[1] + 1e-9, S_STEP):
            for a in np.arange(F.SITE_ANG[0], F.SITE_ANG[1] + 1e-9, ANG_STEP):
                p = site_point(name, float(s), side * float(a))
                st = F.site_of(p)
                out.append((name, float(s), side * float(a), p, st[3]))
    return out


def k_site(p, n, mu, tilt=F.TILT_MAX, hinge=None):
    """The least tipping moment per newton of shield force (N m / N) over every allowed press at one site; hinge is
    (x, z) in the mechanism frame, the task's by default."""
    hx, hz = hinge if hinge is not None else (F.PIVOT_X, F.HINGE_Z)
    r = np.asarray(p, float) - np.array([hx, 0.0, hz])
    am = math.atan(mu)
    Cn = cone(n, am)
    best = -math.inf
    for d in cone(n, tilt):
        C = np.vstack([Cn, cone(d, am)])
        cd = C @ d
        ok = cd > 1e-9
        hold = (r[2] * C[ok, 0] - r[0] * C[ok, 2]) / cd[ok]
        best = max(best, float(hold.max()))
    return -best


def sweep(mu, side=-1, tilt=F.TILT_MAX):
    """(k_min, (finger, s mm, ang deg)) over the whole site."""
    worst = (math.inf, None)
    for name, s, a, p, n in sites(side):
        k = k_site(p, n, mu, tilt)
        if k < worst[0]:
            worst = (k, (name, round(1000 * s, 1), round(math.degrees(a), 1)))
    return worst


def normal_tip_force(name, s, ang):
    """N: the force along the inward normal at one site that tips the hand (no friction needed)."""
    p = site_point(name, s, ang)
    return F.hold_moment() / k_site(p, F.site_of(p)[3], 0.0, tilt=0.0)


def vertical_tip_force(name, s):
    """N: a straight-down press on the top of a finger's pad (s past its radius) that tips the hand."""
    return F.hold_moment() / (F.PIVOT_X - (F.FINGERS[name][1] + s - F.APEX[0]))


if __name__ == "__main__":
    G = F.hold_moment()
    print(f"hinge {1000 * F.PIVOT_X:.0f} mm behind the ring's apex, {1000 * (F.HZ + F.HINGE_Z):.1f} mm over the table; "
          f"G = {G:.4f} N m")
    for name in F.SITE_FINGERS:
        print(f"{name}: straight-down press tips at {vertical_tip_force(name, F.SITE_S[0]):.2f} to "
              f"{vertical_tip_force(name, F.SITE_S[1]):.2f} N; along the inward normal at "
              f"{1000 * F.SITE_S[1]:.0f} mm, {math.degrees(F.SITE_ANG[1]):.0f} deg: "
              f"{normal_tip_force(name, F.SITE_S[1], -F.SITE_ANG[1]):.2f} N")
    for mu in (0.0, 0.25, 0.5):
        k, w = sweep(mu)
        print(f"mu {mu}: k_min {k:.4f} N m/N at {w}; every allowed press tips by {G / k:.2f} N; margin at F_FIRM "
              f"{F.F_FIRM:g} N {F.F_FIRM * k - G:+.3f} N m, at F_FIRE {F.F_FIRE:g} N {F.F_FIRE * k - G:+.3f} N m")
    k, w = sweep(0.0, side=1)
    print(f"the side that faces down (not a site), mu 0: k_min {k:.4f} at {w}: "
          + ("never tips" if k <= 0 else f"tips by {G / k:.2f} N"))
