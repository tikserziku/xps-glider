# -*- coding: utf-8 -*-
"""aero.py - mass, balance, stability, trim, glide performance and launch trajectory of the flat foam gliders.

Method (all planar, longitudinal):
  * vortex-lattice (horseshoe vortices, staircase strips) over the real H outline -> lift slope, neutral point,
    control power, pitch damping;
  * flat plate without leading-edge suction: lift-dependent drag = normal force tilted back by the local incidence;
  * zero-lift drag = laminar skin friction (Blasius) + drag of the 5 mm blunt edges (ASSUMPTION K_EDGE);
  * nose ballast sized for a static margin of SM_TARGET; trim by bending the control surface;
  * 3-DOF simulation (x, z, pitch) of a hand launch.
USAGE:  python aero.py [--selftest] [--scale 1.0] [--out <results.json>]
Numbers come only from glider_geom.py. Run with tools\\cad-venv python (numpy).
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from glider_geom import AREAL, RHO_FOAM, T_FOAM, get_forms, self_check  # noqa: E402

RHO, NU, G = 1.225, 1.5e-5, 9.81
K_EDGE = 0.30          # ASSUMPTION: drag coefficient of the sheet edges on their frontal area (LE sanded round, TE square)
CL_MAX = 0.70          # ASSUMPTION: flat plate, Re 1e4..5e4
CL_DESIGN_MAX = 0.45   # do not trim closer to the stall than this (0.55 left no margin: a level throw stalled)
SM_TARGET = 0.10       # static margin, fraction of the reference chord
H_LAUNCH = 1.6         # m, release height
SIG_ALLOW = 0.25e6     # Pa, ASSUMPTION: bending strength of XPS underlay
N_LOAD = 3.0           # launch load factor for the wing root check
ASSUMPTIONS = dict(rho_foam_kg_m3=RHO_FOAM, areal_kg_m2=AREAL, k_edge=K_EDGE, cl_max=CL_MAX, sm_target=SM_TARGET,
                   h_launch_m=H_LAUNCH, sigma_allow_pa=SIG_ALLOW, n_load=N_LOAD, rho_air=RHO, nu_air=NU)


# ---------------------------------------------------------------- polygons
def poly_props(pts):
    """area, centroid (cx, cy), second moments about the origin (int x2 dA, int y2 dA). Any orientation."""
    x = np.array([p[0] for p in pts], float)
    y = np.array([p[1] for p in pts], float)
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cr = x * y1 - x1 * y
    a = cr.sum() / 2.0
    cx = ((x + x1) * cr).sum() / (6 * a)
    cy = ((y + y1) * cr).sum() / (6 * a)
    ix2 = ((x * x + x * x1 + x1 * x1) * cr).sum() / 12.0
    iy2 = ((y * y + y * y1 + y1 * y1) * cr).sum() / 12.0
    if a < 0:
        a, ix2, iy2 = -a, -ix2, -iy2
    return a, cx, cy, ix2, iy2


def crossings(pts, yc):
    xs = []
    n = len(pts)
    for i in range(n):
        (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % n]
        if (y0 <= yc < y1) or (y1 <= yc < y0):
            xs.append(x0 + (x1 - x0) * (yc - y0) / (y1 - y0))
    xs.sort()
    return list(zip(xs[0::2], xs[1::2]))


def inside(pt, poly):
    x, y = pt
    c = False
    n = len(poly)
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        if (y0 > y) != (y1 > y) and x < x0 + (x1 - x0) * (y - y0) / (y1 - y0):
            c = not c
    return c


# ---------------------------------------------------------------- vortex lattice
def _wseg(px, py, x1, y1, x2, y2):
    """z-velocity at points (px, py) induced by unit-strength straight vortex segments 1->2, all in the plane z = 0."""
    r1x, r1y, r2x, r2y = px - x1, py - y1, px - x2, py - y2
    cz = r1x * r2y - r1y * r2x
    n1, n2 = np.hypot(r1x, r1y), np.hypot(r2x, r2y)
    r0x, r0y = x2 - x1, y2 - y1
    dot = (r0x * r1x + r0y * r1y) / n1 - (r0x * r2x + r0y * r2y) / n2
    with np.errstate(divide="ignore", invalid="ignore"):
        w = dot / (4 * np.pi * cz)
    w[np.abs(cz) < 1e-12] = 0.0
    return w


def vlm(H_mm, flaps_mm, dy_max, dx_t):
    """Unit solutions of the planar VLM. Lengths in metres. Returns the sums needed for every coefficient."""
    pts = [(x / 1000.0, y / 1000.0) for x, y in H_mm]
    flaps = [[(x / 1000.0, y / 1000.0) for x, y in q] for q in flaps_mm]
    ys = sorted(set(round(p[1], 7) for p in pts))
    edges = [ys[0]]
    for a, b in zip(ys, ys[1:]):
        if b - a < 2e-5:
            continue
        n = max(1, math.ceil((b - a) / dy_max))
        edges += [a + (b - a) * k / n for k in range(1, n + 1)]
    P = []
    w_edge = 0.0
    for ya, yb in zip(edges, edges[1:]):
        yc = 0.5 * (ya + yb)
        for x0, x1 in crossings(pts, yc):
            if x1 - x0 < 1e-5:
                continue
            w_edge += yb - ya
            n = max(2, math.ceil((x1 - x0) / dx_t))
            d = (x1 - x0) / n
            for k in range(n):
                xa = x0 + k * d
                xc = xa + 0.75 * d
                m = 1.0 if any(inside((xc, yc), q) for q in flaps) else 0.0
                P.append((xa + 0.25 * d, xc, ya, yb, yc, m))
    P = np.array(P)
    xb, xc, ya, yb, yc, mask = P.T
    dy = yb - ya
    far = 200.0
    px, py = xc[:, None], yc[:, None]
    A = (_wseg(px, py, xb[None] + far, ya[None], xb[None], ya[None])
         + _wseg(px, py, xb[None], ya[None], xb[None], yb[None])
         + _wseg(px, py, xb[None], yb[None], xb[None] + far, yb[None]))
    rhs = np.stack([-np.ones_like(xc), -mask, -xc], axis=1)
    g = np.linalg.solve(A, rhs)
    ga, gd, gx = g.T
    s = lambda v: float((v * dy).sum())
    return dict(n=len(P), a0=s(ga), a1=s(ga * xb), d0=s(gd), d1=s(gd * xb), x0=s(gx), x1=s(gx * xb),
                am=s(ga * mask), dm=s(gd * mask), w_edge=w_edge)


def coeffs(v, S, c, xcg):
    """Aerodynamic derivatives about the centre of gravity (per rad; q-derivatives per q*c/2V)."""
    return dict(
        CLa=2 * v["a0"] / S, CLd=2 * v["d0"] / S,
        Cma=-2 * (v["a1"] - xcg * v["a0"]) / (S * c), Cmd=-2 * (v["d1"] - xcg * v["d0"]) / (S * c),
        CLq=4 * (v["x0"] - xcg * v["a0"]) / (c * S),
        Cmq=-4 * (v["x1"] - xcg * v["x0"] - xcg * v["a1"] + xcg * xcg * v["a0"]) / (c * c * S),
        kaa=2 * v["a0"] / S, kad=2 * (v["am"] + v["d0"]) / S, kdd=2 * v["dm"] / S,
    )


def cd0(V, S, c, swet, edge_area):
    cf = 1.328 / math.sqrt(max(V * c / NU, 100.0))
    return cf * swet / S + K_EDGE * edge_area / S


def trim(k, CL):
    det = k["CLa"] * k["Cmd"] - k["CLd"] * k["Cma"]
    alpha = CL * k["Cmd"] / det
    delta = -CL * k["Cma"] / det
    cdl = k["kaa"] * alpha * alpha + k["kad"] * alpha * delta + k["kdd"] * delta * delta
    return alpha, delta, cdl


# ---------------------------------------------------------------- 3-DOF launch simulation
def simulate(p, v0, gamma0_deg=None, delta=None, h0=H_LAUNCH, dt=0.002, t_max=20.0):
    """p: dict(m, Iyy, S, c, CLa, CLd, CLq, Cma, Cmd, Cmq, CD0, kpol, alpha_trim, delta_trim, gamma_trim).
    Earth axes, z up. Default launch = along the trimmed glide path (a smooth push slightly downwards)."""
    d = p["delta_trim"] if delta is None else delta
    g0 = p["gamma_trim"] if gamma0_deg is None else math.radians(gamma0_deg)

    def f(s):
        x, z, vx, vz, th, q = s
        V = math.hypot(vx, vz)
        gam = math.atan2(vz, vx)
        al = th - gam
        qh = q * p["c"] / (2 * V)
        cl_lin = p["CLa"] * al + p["CLd"] * d + p["CLq"] * qh
        cl = max(-CL_MAX, min(CL_MAX, cl_lin))
        # flat plate without LE suction: drag due to lift = normal force tilted back by the incidence.
        # Below the stall this equals kpol*CL^2 (the trimmed polar); beyond it CL stays at CL_MAX and drag keeps growing.
        a_eq = min(abs(cl_lin) / p["CLa"], 1.2)
        cd = p["CD0"] + p["kpol"] * p["CLa"] * abs(cl) * math.tan(a_eq)
        cm = p["Cma"] * al + p["Cmd"] * d + p["Cmq"] * qh
        qd = 0.5 * RHO * V * V
        L, D = qd * p["S"] * cl, qd * p["S"] * cd
        ax = (-D * math.cos(gam) - L * math.sin(gam)) / p["m"]
        az = (-D * math.sin(gam) + L * math.cos(gam)) / p["m"] - G
        return np.array([vx, vz, ax, az, q, qd * p["S"] * p["c"] * cm / p["Iyy"]])

    s = np.array([0.0, h0, v0 * math.cos(g0), v0 * math.sin(g0), g0 + p["alpha_trim"], 0.0])
    path = [(0.0, h0)]
    t, zmax = 0.0, h0
    while s[1] > 0 and t < t_max:
        k1 = f(s)
        k2 = f(s + 0.5 * dt * k1)
        k3 = f(s + 0.5 * dt * k2)
        k4 = f(s + dt * k3)
        s = s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        t += dt
        zmax = max(zmax, s[1])
        if len(path) < 4000 and int(t / dt) % 10 == 0:
            path.append((round(float(s[0]), 3), round(float(s[1]), 3)))
    return dict(range_m=float(s[0]), time_s=t, zmax_m=float(zmax), path=path)


# ---------------------------------------------------------------- one form
def analyze(f, areal=AREAL, sm=SM_TARGET, with_path=True):
    S, c = f["S_ref"] * 1e-6, f["c_ref"] * 1e-3
    t = f["t"] / 1000.0
    v = vlm(f["H"], f["flaps"], dy_max=f["span"] / 1000.0 / 44.0, dx_t=c / 7.0)
    x_np = v["a1"] / v["a0"]
    aH, cxH, _, ix2H, _ = poly_props(f["H_cut"])
    aV, cxV, czV, ix2V, iz2V = poly_props(f["V_cut"])
    aH, aV, cxH, cxV = aH * 1e-6, aV * 1e-6, cxH * 1e-3, cxV * 1e-3
    ix2H, ix2V, iz2V = ix2H * 1e-12, ix2V * 1e-12, iz2V * 1e-12
    m_foam = areal * (aH + aV)
    x_foam = (aH * cxH + aV * cxV) / (aH + aV)
    i0_foam = areal * (ix2H + ix2V + iz2V)                 # about the origin (nose)
    # gross outlines (no slots): the page scales the planform by k, but the slots stay 5 mm wide, so their
    # area grows as k, not k^2.  gross ~ k^2 / k^3 / k^4, slot part ~ k / k^2 / k^3.
    gH, gV = poly_props(f["H"]), poly_props(f["V"])
    a_g = (gH[0] + gV[0]) * 1e-6
    ax_g = (gH[0] * gH[1] + gV[0] * gV[1]) * 1e-9
    i_g = (gH[3] + gV[3] + gV[4]) * 1e-12
    slot = dict(Ag=a_g, Agx=ax_g, Ig=i_g, dA=a_g - (aH + aV), dAx=ax_g - (aH * cxH + aV * cxV),
                dI=i_g - (ix2H + ix2V + iz2V))
    xb = f["x_ballast"] / 1000.0
    x_cg = x_np - sm * c
    m_b = m_foam * (x_foam - x_cg) / (x_cg - xb)
    if m_b < 0:
        m_b, x_cg = 0.0, x_foam
    m = m_foam + m_b
    iyy = i0_foam - m_foam * (2 * x_cg * x_foam - x_cg * x_cg) + m_b * (xb - x_cg) ** 2
    k = coeffs(v, S, c, x_cg)
    swet = 2 * (aH + aV)
    vz = [p[1] for p in f["V"]]
    edge_area = t * (v["w_edge"] + (max(vz) - min(vz)) / 1000.0)

    best = None
    polar = []
    for cl in np.arange(0.12, CL_DESIGN_MAX + 1e-9, 0.01):
        al, de, cdl = trim(k, cl)
        V = math.sqrt(2 * m * G / (RHO * S * cl))
        c0 = cd0(V, S, c, swet, edge_area)
        cd = c0 + cdl
        row = dict(CL=float(cl), V=V, CD=cd, CD0=c0, LD=cl / cd, sink=V * math.sin(math.atan(cd / cl)), alpha=al, delta=de)
        polar.append(row)
        if best is None or row["LD"] > best["LD"]:
            best = row
    msink = min(polar, key=lambda r: r["sink"])
    p = dict(m=m, Iyy=iyy, S=S, c=c, CD0=best["CD0"], kpol=(best["CD"] - best["CD0"]) / best["CL"] ** 2,
             alpha_trim=best["alpha"], delta_trim=best["delta"], gamma_trim=-math.atan(best["CD"] / best["CL"]),
             **{n: k[n] for n in ("CLa", "CLd", "CLq", "Cma", "Cmd", "Cmq")})
    sim = simulate(p, best["V"])
    sim_level = simulate(p, best["V"], gamma0_deg=0.0)       # level throw at the trim speed
    half = f["span"] / 2000.0
    m_root = N_LOAD * m * G / 2.0 * 0.42 * half
    w_root = 0.7 * f["wing_root_chord"] / 1000.0 * t * t / 6.0
    r3 = lambda pts: [[round(a, 2), round(b, 2)] for a, b in pts]
    out = dict(
        id=f["id"], name_ru=f["name_ru"], name_lt=f["name_lt"], kind=f["kind"], scale=f["scale"],
        why_ru=f["why_ru"], why_lt=f["why_lt"], flap_name=f["flap_name"],
        span_mm=f["span"], S_ref_cm2=S * 1e4, c_ref_mm=c * 1e3, AR=(f["span"] / 1000.0) ** 2 / S,
        area_H_cm2=aH * 1e4, area_V_cm2=aV * 1e4,
        m_foam_g=m_foam * 1e3, m_ballast_g=m_b * 1e3, m_total_g=m * 1e3, wing_loading_g_dm2=m * 1e3 / (S * 100),
        x_np_mm=x_np * 1e3, x_cg_mm=x_cg * 1e3, x_foam_cg_mm=x_foam * 1e3, x_ballast_mm=xb * 1e3,
        static_margin=(x_np - x_cg) / c,
        CL=best["CL"], V_ms=best["V"], LD=best["LD"], sink_ms=best["sink"], CD0=best["CD0"],
        Re=best["V"] * c / NU, alpha_deg=math.degrees(best["alpha"]), delta_deg=math.degrees(best["delta"]),
        flap_te_mm=f["flap_chord"] * math.sin(best["delta"]),
        min_sink_ms=msink["sink"], V_min_sink_ms=msink["V"],
        range_glide_m=H_LAUNCH * best["LD"], range_sim_m=sim["range_m"], time_sim_s=sim["time_s"],
        range_level_m=sim_level["range_m"], time_level_s=sim_level["time_s"],
        gamma_deg=math.degrees(p["gamma_trim"]), kpol=p["kpol"],
        sigma_root_kpa=m_root / w_root / 1e3, bending_use=m_root / w_root / SIG_ALLOW,
        n_panels=v["n"],
        js=dict(S=S, c=c, a0=v["a0"], a1=v["a1"], d0=v["d0"], d1=v["d1"], x0=v["x0"], x1=v["x1"], am=v["am"], dm=v["dm"],
                m_foam=m_foam, x_foam=x_foam, i0_foam=i0_foam, xb=xb, swet=swet, edge_area=edge_area, **slot,
                flap_chord=f["flap_chord"] / 1000.0),
        geom=dict(H=r3(f["H_cut"]), V=r3(f["V_cut"]), flaps=[r3(q) for q in f["flaps"]],
                  bends=[r3(b) for b in f["bends"]], bend_deg=f["bend_deg"], t=f["t"]),
    )
    if with_path:
        out["path"] = sim["path"]
    return out


# ---------------------------------------------------------------- checks
def selftest():
    ok = True
    # 1. rectangular wing AR = 6 against Helmbold
    b, c = 0.6, 0.1
    rect = [(0, -300), (100, -300), (100, 300), (0, 300)]
    v = vlm(rect, [], dy_max=b / 44, dx_t=c / 7)
    cla = 2 * v["a0"] / (b * c)
    ar = b / c
    helm = 2 * math.pi * ar / (2 + math.sqrt(ar * ar + 4))
    xnp = v["a1"] / v["a0"] / c
    e1 = abs(cla / helm - 1)
    print(f"T1 rect AR6: CLa VLM {cla:.3f}  Helmbold {helm:.3f}  diff {e1 * 100:.1f} %  (<= 7 %, Helmbold is approximate)"
          f"   x_np/c {xnp:.3f} (0.22..0.26)")
    ok &= e1 <= 0.07 and 0.22 <= xnp <= 0.26
    # 1b. grid convergence of the lift slope and the lifting-surface benchmark: rectangular AR = 2 -> CLa = 2.474
    vf = vlm(rect, [], dy_max=b / 120, dx_t=c / 14)
    claf = 2 * vf["a0"] / (b * c)
    sq = [(0, -100), (100, -100), (100, 100), (0, 100)]
    v2 = vlm(sq, [], dy_max=0.2 / 44, dx_t=0.1 / 7)
    cla2 = 2 * v2["a0"] / (0.2 * 0.1)
    e1b, e1c = abs(claf / cla - 1), abs(cla2 / 2.474 - 1)
    print(f"T1b rect AR6 fine grid: CLa {claf:.3f} ({vf['n']} panels)  diff {e1b * 100:.2f} %  (<= 1 %)")
    print(f"T1c rect AR2: CLa VLM {cla2:.3f}  lifting-surface benchmark 2.474  diff {e1c * 100:.1f} %  (<= 2 %)")
    ok &= e1b <= 0.01 and e1c <= 0.02
    # 2. neutral point of the conventional form F01 against the tail-volume hand formula
    f = get_forms()[0]
    r = analyze(f, with_path=False)
    Sw, cw, bw, bt, ct = 270 * 50e-6, 0.05, 0.27, 0.10, 0.032
    St = bt * ct
    hel = lambda a: 2 * math.pi * a / (2 + math.sqrt(a * a + 4))
    aw, at = hel(bw / cw), hel(bt / ct)
    eps = 2 * aw / (math.pi * bw / cw)
    xw, xt = 0.075 + 0.25 * cw, 0.075 + 0.118 + 0.25 * ct
    xnp_hand = (aw * Sw * xw + at * (1 - eps) * St * xt) / (aw * Sw + at * (1 - eps) * St)
    e2 = abs(r["x_np_mm"] / 1000 - xnp_hand) / cw
    print(f"T2 F01 neutral point: VLM {r['x_np_mm']:.1f} mm  hand {xnp_hand * 1000:.1f} mm  diff {e2 * 100:.1f} % MAC  (<= 3 %)")
    ok &= e2 <= 0.03
    # 3. trim really balances: Cm = 0, CL = target, and the simulated steady glide matches the polar
    e3 = abs(r["range_sim_m"] / r["range_glide_m"] - 1)
    print(f"T3 F01 range: steady {r['range_glide_m']:.2f} m  simulated {r['range_sim_m']:.2f} m  diff {e3 * 100:.1f} %  (<= 10 %)")
    ok &= e3 <= 0.10
    # 4. grid convergence of the neutral point
    S, c0 = f["S_ref"] * 1e-6, f["c_ref"] * 1e-3
    v2 = vlm(f["H"], f["flaps"], dy_max=f["span"] / 1000 / 80, dx_t=c0 / 12)
    e4 = abs(v2["a1"] / v2["a0"] * 1000 - r["x_np_mm"]) / (c0 * 1000)
    print(f"T4 F01 grid: x_np {r['x_np_mm']:.2f} mm ({r['n_panels']} panels) vs {v2['a1'] / v2['a0'] * 1000:.2f} mm ({v2['n']} panels)"
          f"  diff {e4 * 100:.2f} % MAC  (<= 1 %)")
    ok &= e4 <= 0.01
    print("SELFTEST", "PASS" if ok else "FAIL")
    return ok


def main():
    args = sys.argv[1:]
    if "--selftest" in args:
        sys.exit(0 if selftest() else 1)
    scale = float(args[args.index("--scale") + 1]) if "--scale" in args else 1.0
    out = args[args.index("--out") + 1] if "--out" in args else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "out", "results.json")
    forms = get_forms(scale)
    self_check(forms)                                        # simple polygons only, or nothing is written
    res = [analyze(f) for f in forms]
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(dict(assumptions=ASSUMPTIONS, scale=scale, forms=res), fh, ensure_ascii=False)
    print(f"scale {scale}   areal {AREAL * 1000:.0f} g/m2 (ASSUMPTION)   launch height {H_LAUNCH} m")
    print("id   name            span  AR   foam  ball  total  W/S    xNP   xCG  SM%   CL    V    L/D  sink  range  sim   level time  delta  TEmm  bend  CD0")
    for r in res:
        print(f"{r['id']} {r['name_ru']:<15} {r['span_mm']:4.0f} {r['AR']:4.1f} {r['m_foam_g']:5.2f} {r['m_ballast_g']:5.2f} "
              f"{r['m_total_g']:5.2f} {r['wing_loading_g_dm2']:5.2f} {r['x_np_mm']:6.1f} {r['x_cg_mm']:5.1f} {r['static_margin'] * 100:4.1f} "
              f"{r['CL']:4.2f} {r['V_ms']:4.2f} {r['LD']:5.2f} {r['sink_ms']:5.2f} {r['range_glide_m']:5.2f} {r['range_sim_m']:5.2f} "
              f"{r['range_level_m']:5.2f} {r['time_sim_s']:4.2f} {r['delta_deg']:6.1f} {r['flap_te_mm']:5.1f} {r['bending_use']:5.2f} {r['CD0']:.3f}")
    print("json:", os.path.abspath(out))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
