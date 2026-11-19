# -*- coding: utf-8 -*-
"""glider_geom.py - the ONLY source of numbers for the foam glider project.

Cross layout, 2 flat parts cut from one XPS sheet:
  H = horizontal part (wing + boom + tail in one piece), plane z = 0
  V = vertical part (fuselage + fin), plane y = 0
Half-lap joint: H has a slot on the centreline open at its front edge, V has a slot open at its rear end.
Units: millimetres. Axes: x aft from the fuselage nose, y to the right, z up.
Everything downstream (aero.py, sheets, 3D, page) derives from get_forms(); never type a number elsewhere.
"""
import math

# ---- material and build constants (one place) -------------------------------------------------
T_FOAM = 5.0            # sheet thickness, mm (XPS sheet underlay 5 mm)
SLOT_W = 5.0            # slot width in both parts, mm (= thickness; foam compresses, no clearance)
RHO_FOAM = 33.0         # kg/m3 - ASSUMPTION until a real sheet is weighed
AREAL = RHO_FOAM * T_FOAM / 1000.0   # kg/m2  (0.165)
R_CORNER_H = 8.0        # fillet radius of the horizontal part at mock-up scale, mm (scales with the planform)
R_CORNER_V = 5.0        # fillet radius of the fuselage / fin outline at mock-up scale, mm
MINI_SPAN_MAX = 270.0   # A4 mock-up limit, mm
H_LEN_MAX = 150.0       # A4 mock-up: chordwise extent of H, mm


def _lobe(x0, b, cr, ct, dx=0.0, shape="trap", n=12):
    """One lifting surface (right half). Returns LE points root->tip, TE points tip->root, area, MAC, x of MAC LE."""
    h = b / 2.0
    if shape == "ellipse":
        le, te = [], []
        # true ellipse right to the tip: monotonic stations every 90/24 deg, the tip is one shared point at eta = 1
        # (19.11: a forced 0.985 station after sin(82.5 deg) = 0.991 gave a self-intersecting tip)
        n = 24
        for i in range(n + 1):
            e = math.sin(math.pi / 2 * i / n)
            c = cr * math.sqrt(max(0.0, 1.0 - e * e))
            xl = x0 + 0.30 * (cr - c)          # straight 30 % chord line
            le.append((xl, e * h))
            te.append((xl + c, e * h))
        te = te[:-1]                            # the tip point belongs to the LE list
        area = math.pi / 4.0 * b * cr
        mac = 8.0 / (3.0 * math.pi) * cr
        x_mac_le = x0 + 0.30 * (cr - mac)
        return le, te[::-1], area, mac, x_mac_le
    lam = ct / cr
    le = [(x0, 0.0), (x0 + dx, h)]
    te = [(x0 + dx + ct, h), (x0 + cr, 0.0)]
    area = (cr + ct) / 2.0 * b
    mac = 2.0 / 3.0 * cr * (1 + lam + lam * lam) / (1 + lam)
    y_mac = h / 3.0 * (1 + 2 * lam) / (1 + lam)
    x_mac_le = x0 + dx * y_mac / h
    return le, te, area, mac, x_mac_le


def _x_at_y(pts, y):
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        lo, hi = min(y0, y1), max(y0, y1)
        if lo <= y <= hi and hi > lo:
            return x0 + (x1 - x0) * (y - y0) / (y1 - y0)
    raise ValueError("y outside polyline")


def _mirror(right):
    """right half listed from the front centre point round to the rear centre point (y >= 0) -> closed outline."""
    left = [(x, -y) for (x, y) in reversed(right) if y > 1e-9]
    return right + left


def _round_closed(pts, r, keep_axis=False, min_turn_deg=25.0, n_per_90=6):
    """Fillet every corner of a closed outline with radius r (all forms have rounded corners).
    The tangent length is limited to 45 % of the shorter adjacent edge, so two fillets never meet on one edge.
    Corners turning less than min_turn_deg are left alone (smooth polylines such as the ellipse).
    keep_axis: vertices on the centreline y = 0 stay sharp (they sit inside the fuselage slot)."""
    n, out = len(pts), []
    for i in range(n):
        p0, p1, p2 = pts[i - 1], pts[i], pts[(i + 1) % n]
        v1 = (p0[0] - p1[0], p0[1] - p1[1])
        v2 = (p2[0] - p1[0], p2[1] - p1[1])
        l1, l2 = math.hypot(*v1), math.hypot(*v2)
        if r <= 0 or l1 < 1e-9 or l2 < 1e-9 or (keep_axis and abs(p1[1]) < 1e-9):
            out.append(p1)
            continue
        u1, u2 = (v1[0] / l1, v1[1] / l1), (v2[0] / l2, v2[1] / l2)
        ang = math.acos(max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1])))     # interior angle at p1
        if math.pi - ang < math.radians(min_turn_deg):
            out.append(p1)
            continue
        t = min(r / math.tan(ang / 2.0), 0.45 * min(l1, l2))
        rr = t * math.tan(ang / 2.0)
        a = (p1[0] + u1[0] * t, p1[1] + u1[1] * t)
        b = (p1[0] + u2[0] * t, p1[1] + u2[1] * t)
        bl = math.hypot(u1[0] + u2[0], u1[1] + u2[1])
        d = rr / math.sin(ang / 2.0)
        c = (p1[0] + (u1[0] + u2[0]) / bl * d, p1[1] + (u1[1] + u2[1]) / bl * d)
        a0 = math.atan2(a[1] - c[1], a[0] - c[0])
        sw = math.atan2(b[1] - c[1], b[0] - c[0]) - a0
        sw = (sw + math.pi) % (2 * math.pi) - math.pi
        k = max(2, math.ceil(abs(sw) / (math.pi / 2) * n_per_90))
        out += [(c[0] + rr * math.cos(a0 + sw * j / k), c[1] + rr * math.sin(a0 + sw * j / k)) for j in range(k + 1)]
    return out


def _cut_slot(poly, x_mid, w2, front):
    """Cut the straight half-lap slot |y| < w2 into a closed outline: on the front side (x < x_mid) or the rear side."""
    side = (lambda x: x < x_mid) if front else (lambda x: x > x_mid)
    pts, n = [], len(poly)
    for i in range(n):                                   # guarantee a vertex on the axis inside the slot region
        a, b = poly[i], poly[(i + 1) % n]
        pts.append(a)
        if a[1] * b[1] < 0:
            x = a[0] + (b[0] - a[0]) * (0.0 - a[1]) / (b[1] - a[1])
            if side(x):
                pts.append((x, 0.0))
    inside = lambda p: abs(p[1]) < w2 - 1e-9 and side(p[0])
    out, n = [], len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        ia, ib = inside(a), inside(b)
        if not ia:
            out.append(a)
        if ia != ib:
            o, q = (a, b) if ib else (b, a)               # o outside, q inside
            sgn = 1.0 if o[1] > 0 else -1.0
            tt = (sgn * w2 - o[1]) / (q[1] - o[1])
            c = (o[0] + (q[0] - o[0]) * tt, sgn * w2)
            out += [c, (x_mid, sgn * w2)] if ib else [(x_mid, sgn * w2), c]
    return out


def _flap(lobe_le, lobe_te, b, frac, eta1, eta2):
    """Two quads (right, left): aft `frac` of the local chord between eta1..eta2 of the half-span."""
    te_fwd = lobe_te[::-1]                     # root -> tip
    quads = []
    for sgn in (1, -1):
        q = []
        for eta in (eta1, eta2):
            y = eta * b / 2.0
            xl, xt = _x_at_y(lobe_le, y), _x_at_y(te_fwd, y)
            q.append((xt - frac * (xt - xl), sgn * y, xt))
        (h1, y1, t1), (h2, y2, t2) = q
        quads.append([(h1, y1), (h2, y2), (t2 + 0.5, y2), (t1 + 0.5, y1)])
    return quads


def _v_profile(x_h0, x_end, s, fin=1.0):
    """Side profile of the fuselage + fin, (x, z). s = scale relative to the A4 mock-up."""
    zb, zt, zbt = 15.0 * s, 13.0 * s, 9.0 * s
    hf = 30.0 * s * fin
    return [
        (0.0, -5.0 * s), (6.0 * s, -zb), (x_h0 + 20.0 * s, -zb), (x_end, -zbt),
        (x_end + 5.0 * s, zt + hf), (x_end - 14.0 * s, zt + hf), (x_end - 40.0 * s * fin, zt),
        (8.0 * s, zt), (0.0, 5.0 * s),
    ]


def _form(fid, ru, lt, kind, s, x_h0, lobes, ref, flap_spec, boom_w=16.0, fin=1.0, why_ru="", why_lt=""):
    """lobes: 1 (tailless) or 2 (front, rear) dicts for _lobe (already in mock-up mm, scaled here)."""
    built = []
    for L in lobes:
        le, te, area, mac, xm = _lobe((x_h0 + L["x"]) * s, L["b"] * s, L["cr"] * s, L.get("ct", L["cr"]) * s,
                                      L.get("dx", 0.0) * s, L.get("shape", "trap"))
        built.append(dict(le=le, te=te, area=area, mac=mac, x_mac_le=xm, b=L["b"] * s, cr=L["cr"] * s))
    bw = boom_w * s
    if len(built) == 1:
        right = built[0]["le"] + built[0]["te"]
    else:
        f, r = built
        te_f = f["te"]
        xa = _x_at_y(te_f[::-1], bw / 2)                      # front TE at the boom edge
        cut_te = [p for p in te_f if p[1] > bw / 2 + 1e-9] + [(xa, bw / 2)]
        xb = _x_at_y(r["le"], bw / 2)                          # rear LE at the boom edge
        cut_le = [(xb, bw / 2)] + [p for p in r["le"] if p[1] > bw / 2 + 1e-9]
        right = f["le"] + cut_te + cut_le + r["te"]
    H_sharp = _mirror(right)
    x_front = H_sharp[0][0]
    x_end = max(p[0] for p in H_sharp if abs(p[1]) < bw / 2 + 1e-6)   # rear end on the centreline
    x_mid = x_front + max(0.30 * built[0]["cr"], 8.0)          # where the two slots meet
    # all corners filleted (tips, nose, fin, inner corners at the boom); the slots stay straight so the parts mate
    H = _round_closed(H_sharp, R_CORNER_H * s, keep_axis=True)
    V = _round_closed(_v_profile(x_h0 * s, x_end, s, fin), R_CORNER_V * s)
    # cut outlines with the half-lap slots (what is actually cut from the sheet)
    w2 = SLOT_W / 2.0
    H_cut = _cut_slot(H, x_mid, w2, front=True)
    V_cut = _cut_slot(V, x_mid, w2, front=False)
    if ref == "sum":
        S = sum(b_["area"] for b_ in built)
        c = sum(b_["area"] * b_["mac"] for b_ in built) / S
        x_mac_le = built[0]["x_mac_le"]
    else:
        S, c, x_mac_le = built[ref]["area"], built[ref]["mac"], built[ref]["x_mac_le"]
    fl = built[flap_spec["lobe"]]
    flaps = _flap(fl["le"], fl["te"], fl["b"], flap_spec["frac"], flap_spec["eta1"], flap_spec["eta2"])
    wing = built[ref if ref != "sum" else 1]
    bend_y = 0.60 * wing["b"] / 2.0                              # tip panels bent up (roll stability)
    bends = []
    for sgn in (1, -1):
        y = sgn * bend_y
        bends.append(((_x_at_y(wing["le"], abs(y)), y), (_x_at_y(wing["te"][::-1], abs(y)), y)))
    return dict(
        id=fid, name_ru=ru, name_lt=lt, kind=kind, scale=s, why_ru=why_ru, why_lt=why_lt,
        H=H, V=V, H_cut=H_cut, V_cut=V_cut,
        H_slot=(x_front, x_mid), V_slot=(x_mid, x_end), slot_w=SLOT_W, t=T_FOAM,
        flaps=flaps, flap_chord=flap_spec["frac"] * fl["cr"], flap_name=flap_spec["name"],
        bends=bends, bend_deg=15.0,
        S_ref=S, c_ref=c, x_mac_le=x_mac_le, span=max(b_["b"] for b_ in built),
        wing_root_chord=wing["cr"], x_ballast=7.0 * s,
    )


def get_forms(s=1.0):
    """The 10 forms. Numbers are the A4 mock-up (s = 1); s > 1 scales the planform, thickness stays 5 mm."""
    stab = dict(lobe=1, frac=0.35, eta1=0.0, eta2=1.0, name="elevator")
    F = []
    F.append(_form("F01", "Классика", "Klasika", "conventional", s, 75,
                   [dict(x=0, b=270, cr=50), dict(x=118, b=100, cr=32)], 0, stab,
                   why_ru="Прямоугольное крыло и хвост — точка отсчёта для сравнения.",
                   why_lt="Stačiakampis sparnas ir uodega — atskaitos taškas palyginimui."))
    F.append(_form("F02", "Трапеция", "Trapecija", "conventional", s, 75,
                   [dict(x=0, b=270, cr=62, ct=36, dx=6.5), dict(x=114, b=104, cr=36, ct=24, dx=6)], 0, stab,
                   why_ru="Сужение к концам: меньше массы на концах и ближе к эллиптической нагрузке.",
                   why_lt="Siaurėjantis sparnas: mažiau masės galuose, apkrova artimesnė elipsinei."))
    F.append(_form("F03", "Эллипс", "Elipsė", "conventional", s, 75,
                   [dict(x=0, b=270, cr=62, shape="ellipse"), dict(x=116, b=104, cr=34, ct=24, dx=5)], 0, stab,
                   why_ru="Эллиптическое крыло — теоретический минимум индуктивного сопротивления.",
                   why_lt="Elipsinis sparnas — teorinis indukcinio pasipriešinimo minimumas."))
    F.append(_form("F04", "Стрела", "Strėlė", "conventional", s, 75,
                   [dict(x=0, b=270, cr=50, ct=50, dx=63), dict(x=120, b=100, cr=30)], 0, stab,
                   why_ru="Стреловидность 25°: красиво и устойчиво по курсу, но фокус уходит назад, плечо хвоста короче.",
                   why_lt="25° strėliškumas: gražu ir stabilu pagal kursą, bet židinys pasislenka atgal."))
    F.append(_form("F05", "Парящий", "Sklandytojas", "conventional", s, 75,
                   [dict(x=0, b=280, cr=44, ct=30, dx=3.5), dict(x=122, b=96, cr=28, ct=20, dx=4)], 0, stab,
                   why_ru="Большое удлинение (≈ 7,6): меньше индуктивное сопротивление, но узкая хорда хуже при малых Re.",
                   why_lt="Didelis proilgis (≈ 7,6): mažesnis indukcinis pasipriešinimas, bet siaura styga prasčiau veikia esant mažam Re."))
    F.append(_form("F06", "Широкое крыло", "Platus sparnas", "conventional", s, 75,
                   [dict(x=0, b=190, cr=72), dict(x=116, b=90, cr=34)], 0, stab,
                   why_ru="Малое удлинение (≈ 2,6): прочное и компактное, но планирует круче.",
                   why_lt="Mažas proilgis (≈ 2,6): tvirtas ir kompaktiškas, bet sklendžia stačiau."))
    F.append(_form("F07", "Дельта", "Delta", "tailless", s, 50,
                   [dict(x=0, b=230, cr=124, ct=14, dx=110)], 0,
                   dict(lobe=0, frac=0.14, eta1=0.25, eta2=0.95, name="elevons"), fin=1.2,
                   why_ru="Бесхвостка-треугольник: одна несущая поверхность, балансировка отгибом задней кромки вверх.",
                   why_lt="Beuodegis trikampis: vienas laikantysis paviršius, balansuojama atlenkiant galinę briauną aukštyn."))
    F.append(_form("F08", "Утка", "Antis", "canard", s, 40,
                   [dict(x=0, b=110, cr=30), dict(x=102, b=270, cr=48)], 1,
                   dict(lobe=0, frac=0.35, eta1=0.0, eta2=1.0, name="canard flap"), fin=1.25,
                   why_ru="Малое крыло впереди: оно тоже несёт, а сваливается первым — нос сам опускается. Плечо киля короткое.",
                   why_lt="Mažas sparnas priekyje: jis irgi kelia, o srautas nuo jo atitrūksta pirmiau — nosis nusileidžia pati. Kilio petys trumpas."))
    F.append(_form("F09", "Тандем", "Tandemas", "tandem", s, 45,
                   [dict(x=0, b=220, cr=38), dict(x=88, b=250, cr=42)], "sum",
                   dict(lobe=1, frac=0.30, eta1=0.0, eta2=0.6, name="rear wing flap"),
                   why_ru="Два почти равных крыла: большая площадь при малом размахе, но заднее работает в скосе потока.",
                   why_lt="Du beveik vienodi sparnai: didelis plotas esant mažam sparnų mostui, bet galinis dirba nuopūtos sraute."))
    F.append(_form("F10", "Летающее крыло", "Skraidantis sparnas", "tailless", s, 60,
                   [dict(x=0, b=270, cr=62, ct=40, dx=72)], 0,
                   dict(lobe=0, frac=0.25, eta1=0.5, eta2=1.0, name="elevons"), fin=1.25,
                   why_ru="Стреловидное крыло без хвоста: концы работают как стабилизатор; чувствительно к центровке.",
                   why_lt="Strėlinis sparnas be uodegos: galai dirba kaip stabilizatorius; jautrus centruotei."))
    return F


def self_intersections(pts):
    """Number of crossings between non-adjacent edges of a closed outline (0 = simple polygon)."""
    def ccw(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    n, bad = len(pts), 0
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            c, d = pts[j], pts[(j + 1) % n]
            if ccw(a, b, c) * ccw(a, b, d) < -1e-9 and ccw(c, d, a) * ccw(c, d, b) < -1e-9:
                bad += 1
    return bad


def self_check(forms=None):
    """Every outline must be a simple polygon, symmetric H, slot width = SLOT_W. Raises on failure."""
    for f in forms or get_forms():
        for key in ("H", "V", "H_cut", "V_cut"):
            n = self_intersections(f[key])
            if n:
                raise ValueError(f"{f['id']} {key}: {n} self-intersection(s)")
        ys = [p[1] for p in f["H_cut"]]
        if abs(max(ys) + min(ys)) > 1e-6:
            raise ValueError(f"{f['id']} H_cut is not symmetric")
        if abs((max(ys) - min(ys)) - f["span"]) > 1e-6:
            raise ValueError(f"{f['id']} fillets changed the span: {max(ys) - min(ys):.3f} vs {f['span']:.3f}")
        # both slots must be exactly SLOT_W wide and end at the same station x_mid
        x_mid = f["H_slot"][1]
        for key in ("H_cut", "V_cut"):
            ends = sorted(p[1] for p in f[key] if abs(p[0] - x_mid) < 1e-9 and abs(abs(p[1]) - SLOT_W / 2) < 1e-9)
            if len(ends) != 2 or abs((ends[1] - ends[0]) - SLOT_W) > 1e-9:
                raise ValueError(f"{f['id']} {key}: slot end not found or width != {SLOT_W}")
    return True


if __name__ == "__main__":
    import sys
    sys.stdout.reconfigure(encoding="utf-8")
    print("self_check", "PASS" if self_check() else "FAIL")
    for f in get_forms():
        xs = [p[0] for p in f["H"]]
        ys = [p[1] for p in f["H"]]
        vx = [p[0] for p in f["V"]]
        vz = [p[1] for p in f["V"]]
        print(f"{f['id']} {f['name_ru']:<15} H {max(ys) - min(ys):6.1f} x {max(xs) - min(xs):6.1f}  "
              f"V {max(vx) - min(vx):6.1f} x {max(vz) - min(vz):5.1f}  S_ref {f['S_ref']:8.0f}  c_ref {f['c_ref']:5.1f}  "
              f"slotH {f['H_slot'][0]:.1f}..{f['H_slot'][1]:.1f}  slotV ..{f['V_slot'][1]:.1f}")
