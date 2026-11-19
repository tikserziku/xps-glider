# -*- coding: utf-8 -*-
"""make_a1.py - ONE glider form F07 "Delta" at the largest size on an A1 landscape sheet (841 x 594 mm), 1:1.

USAGE (FreeCAD python, any folder):
    python make_a1.py            solve scale, build DXF + PDF + PNG, run the check
    python make_a1.py --check    re-run only the numeric check (rebuilds the placement deterministically)
Outputs (project/out): F07_A1.dxf F07_A1.pdf F07_A1.png a1_check.json (+ a1_crop_title.png a1_crop_slot.png)
Glider axes: x aft from nose, y right, z up. Sheet: X right 0..841, Y up 0..594. Text: Lithuanian only.
"""
import os, sys, json, math, subprocess, re, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_sheets as ms                      # reuse helpers (fonts, text_w, Polygon, Layout, ...)
from author import author_line

AUTHOR_LINE = author_line()
import ezdxf
from shapely.geometry import LineString, Point, box
from shapely.prepared import prep
from shapely.ops import unary_union

# A1 sheet: override the A4 constants used inside the reused Layout class
SW, SH = 841.0, 594.0
ms.SW, ms.SH = SW, SH
M = ms.M                      # 8.0 mm hard margin
INSET = ms.INSET              # 0.2 mm
OUT = ms.OUT
Polygon, text_w, text_box, Layout = ms.Polygon, ms.text_w, ms.text_box, ms.Layout
GAP_MIN = 6.0
TH = 5.0                      # default text height
S_START, S_STEP = 3.70, 0.01
DXF = os.path.join(OUT, "F07_A1.dxf")
PDF = os.path.join(OUT, "F07_A1.pdf")
PNG = os.path.join(OUT, "F07_A1.png")
CHK = os.path.join(OUT, "a1_check.json")
sys.path.insert(0, HERE)


def fc(x, n=1):
    """number with decimal comma"""
    return f"{x:.{n}f}".replace(".", ",")


# ---------------------------------------------------------------- glyph support
def has(ch):
    try:
        return ms.xfonts.make_font(ms.FONT_FILE, 5).has_glyph(ord(ch))
    except Exception:
        return False


Q1, Q2 = ("„", "“") if (has("„") and has("“")) else ('"', '"')
APPROX = "≈" if has("≈") else "~"
GLYPH_NOTE = {"font": ms.FONT_FILE, "quotes_ok": Q1 != '"', "approx_ok": APPROX != "~"}


# ---------------------------------------------------------------- data
def get(s):
    from glider_geom import get_forms
    import aero
    f = [x for x in get_forms(s) if x["id"] == "F07"][0]
    r = aero.analyze(f, with_path=False)
    return f, r


# ---------------------------------------------------------------- placement
def place(s):
    """H bottom (TE on bottom margin, apex up, centred); V top, mirrored, leftmost X0 with gap >= GAP_MIN."""
    f, r = get(s)
    H, V = f["H_cut"], f["V_cut"]
    hxm = max(p[0] for p in H)
    top, bot = SH - M - INSET, M + INSET
    left, right = M + INSET, SW - M - INSET
    cx = SW / 2.0
    Hs = [(cx + p[1], bot + (hxm - p[0])) for p in H]
    if min(p[0] for p in Hs) < left or max(p[0] for p in Hs) > right or max(p[1] for p in Hs) > top:
        return None
    Hp = Polygon(Hs)
    vmin_z = min(p[1] for p in V)
    vmax_z = max(p[1] for p in V)
    vmin_x = min(p[0] for p in V)
    vmax_x = max(p[0] for p in V)
    Y0 = top + vmin_z                              # Y = Y0 - z : smallest z on the top margin
    if Y0 - vmax_z < bot:
        return None
    n = int((right - vmax_x - (left - vmin_x)) / 0.5)
    for i in range(n + 1):
        X0 = left - vmin_x + 0.5 * i
        Vs = [(X0 + p[0], Y0 - p[1]) for p in V]
        g = Hp.distance(Polygon(Vs))
        if g >= GAP_MIN + 0.05:
            return dict(s=s, f=f, r=r, Hs=Hs, Vs=Vs, X0=X0, Y0=Y0, cx=cx, Ybot=bot, hxm=hxm, gap=g)
    return None


# ---------------------------------------------------------------- drawing
def build_doc(P, strict=True):
    f, r = P["f"], P["r"]
    s = P["s"]
    Hs, Vs, X0, Y0, cx, Ybot, hxm = P["Hs"], P["Vs"], P["X0"], P["Y0"], P["cx"], P["Ybot"], P["hxm"]
    Hp, Vp = Polygon(Hs), Polygon(Vs)
    doc = ezdxf.new("R2018", setup=True)
    doc.units = 4
    doc.header["$LTSCALE"] = 8.0
    for nm, lw, lt in (("CUT", 50, "Continuous"), ("MARK", 18, "DASHED"), ("TEXT", 18, "Continuous"),
                       ("QR", 13, "Continuous"), ("FRAME", 13, "Continuous")):
        doc.layers.add(nm, lineweight=lw, linetype=lt, color=7)
    doc.styles.add("ISO", font=ms.FONT_FILE)
    msp = doc.modelspace()
    cont = {"linetype": "Continuous"}

    def hx(x, y):
        return (cx + y, Ybot + (hxm - x))

    def vx(x, z):
        return (X0 + x, Y0 - z)

    msp.add_lwpolyline([(0, 0), (SW, 0), (SW, SH), (0, SH)], close=True,
                       dxfattribs={"layer": "FRAME", "lineweight": 13})
    msp.add_lwpolyline(Hs, close=True, dxfattribs={"layer": "CUT", "lineweight": 50})
    msp.add_lwpolyline(Vs, close=True, dxfattribs={"layer": "CUT", "lineweight": 50})
    lay = Layout([Hp, Vp])
    lay.add_line(LineString(Hs + [Hs[0]]))
    lay.add_line(LineString(Vs + [Vs[0]]))
    texts, where = [], {}

    hinges = []
    for q in f["flaps"]:
        a, b = hx(*q[0]), hx(*q[1])
        msp.add_line(a, b, dxfattribs={"layer": "MARK", "lineweight": 18})
        hinges.append((a, b))
        lay.add_line(LineString([a, b]))
    bends = []
    for (p1, p2) in f["bends"]:
        a, b = hx(*p1), hx(*p2)
        msp.add_line(a, b, dxfattribs={"layer": "MARK", "lineweight": 18})
        bends.append((a, b))
        lay.add_line(LineString([a, b]))
    # centre of gravity
    xcg = r["x_cg_mm"]
    zs = [p[1] for p in f["V_cut"]]
    raw = LineString([(xcg, min(zs) - 5), (xcg, max(zs) + 5)])
    inter = raw.intersection(Polygon(f["V_cut"]))
    segs = [inter] if inter.geom_type == "LineString" else list(inter.geoms)
    tick_top = None
    for sg in segs:
        if sg.is_empty or sg.length < 1e-6:
            continue
        a, b = vx(*sg.coords[0]), vx(*sg.coords[-1])
        msp.add_line(a, b, dxfattribs={"layer": "MARK", "lineweight": 18, **cont})
        lay.add_line(LineString([a, b]))
        for p in (a, b):
            if tick_top is None or p[1] > tick_top[1]:
                tick_top = p
    cgc = hx(xcg, 0.0)
    msp.add_circle(cgc, 6.0, dxfattribs={"layer": "MARK", "lineweight": 18, **cont})
    lay.add_line(Point(cgc).buffer(6.0).exterior)
    bc = vx(f["x_ballast"], 0.0)
    msp.add_circle(bc, 10.0, dxfattribs={"layer": "MARK", "lineweight": 18, **cont})
    lay.add_line(Point(bc).buffer(10.0).exterior)

    # ---- title block, QR, ruler (free areas first)
    ln = [
        (10, f"SKLANDYTUVAS F07 {Q1}DELTA{Q2}"),
        (TH, "XPS paklotas 5 mm · M 1:1 · spausdinti 100 %"),
        (TH, f"Sparnų mostas {r['span_mm']:.0f} mm · masė {r['m_total_g']:.0f} g · balastas {fc(r['m_ballast_g'])} g"),
        (TH, f"Svorio centras (SC) – {r['x_cg_mm']:.0f} mm nuo nosies"),
        (TH, f"Sklendimo greitis {fc(r['V_ms'])} m/s · kokybė K = {fc(r['LD'])} · nuotolis {APPROX} {fc(r['range_sim_m'])} m (h = 1,6 m)"),
        *([(7, AUTHOR_LINE)] if AUTHOR_LINE else []),      # optional, from author.local.json (see author.py)
        (7, "https://xps-glider.pages.dev/"),
    ]
    title_lines = [t for _, t in ln]
    base, y = [], 0.0
    for i, (h, t) in enumerate(ln):
        y = -1.05 * h if i == 0 else y - 1.7 * h
        base.append(y)
    tw = max(text_w(t, h) for h, t in ln) + 1.0
    thh = -base[-1] + 0.3 * ln[-1][0] + 1.0
    # title block: prefer the top-left corner, QR beside it
    t_res = lay.free(tw, thh, prefer=(M, SH - M - thh), allow_inside=None)
    if not t_res:
        where["title"] = "NOT PLACED"
        if strict:
            return None
    else:
        tx, ty, tb, _ = t_res
        lay.boxes.append(tb)
        ytop = ty + thh
        for (h, t), b in zip(ln, base):
            texts.append((tx + 0.5, ytop + b - 0.0, h, t))
        where["title"] = (round(tx, 1), round(ty, 1), round(tw, 1), round(thh, 1))
    # QR
    qr = json.load(open(os.path.join(OUT, "qr.json"), encoding="utf-8"))
    n = qr["size"]
    assert n == 29 and len(qr["rows"]) == n and all(len(rw) == n for rw in qr["rows"])
    mod = 50.0 / n
    side = (n + 8) * mod
    pref = (tx + tw + 3, ty) if t_res else (M, M)
    q_res = lay.free(side, side, prefer=pref)
    if not q_res:
        where["qr"] = "NOT PLACED"
        if strict:
            return None
        qx = qy = None
    else:
        qx, qy, qb, _ = q_res
        lay.boxes.append(qb)
        ox, oy = qx + 4 * mod, qy + 4 * mod          # lower-left of the 29-module field
        for ri, row in enumerate(qr["rows"]):
            ybot = oy + (n - 1 - ri) * mod
            c = 0
            while c < n:
                if row[c] == "1":
                    c2 = c
                    while c2 < n and row[c2] == "1":
                        c2 += 1
                    h = msp.add_hatch(color=7, dxfattribs={"layer": "QR"})
                    h.paths.add_polyline_path([(ox + c * mod, ybot), (ox + c2 * mod, ybot),
                                               (ox + c2 * mod, ybot + mod), (ox + c * mod, ybot + mod)],
                                              is_closed=True)
                    c = c2
                else:
                    c += 1
        where["qr"] = {"quiet_zone_box": [round(qx, 3), round(qy, 3), round(qx + side, 3), round(qy + side, 3)],
                       "module_mm": round(mod, 4), "field_mm": 50.0}
    # ruler: 100.000 mm, ticks every 10 mm
    rr = lay.free(102.0, 12.0, prefer=(SW - M - 102, M), allow_inside=Hp)
    if not rr:
        where["ruler"] = "NOT PLACED"
        if strict:
            return None
    else:
        rx, ry, rb, rmode = rr
        lay.boxes.append(rb)
        x0, y0 = rx + 1.0, ry + 3.0
        msp.add_line((x0, y0), (x0 + 100.0, y0), dxfattribs={"layer": "TEXT", "lineweight": 18})
        for k in range(11):
            tl = 3.0 if k in (0, 10) else 1.5
            msp.add_line((x0 + 10 * k, y0 - tl), (x0 + 10 * k, y0 + tl), dxfattribs={"layer": "TEXT", "lineweight": 18})
        texts.append((x0, y0 + 4.0, TH, "100 mm"))
        where["ruler"] = (rmode, round(x0, 3), round(y0, 3))

    # ---- big labels inside parts
    def inside(poly, anchor, txt, h=TH):
        w = text_w(txt, h)
        hh = 1.35 * h
        ins = prep(poly.buffer(-1.5))
        cands = []
        minx, miny, maxx, maxy = poly.bounds
        for xi in range(int(minx), int(maxx - w) + 1):
            for yi in range(int(miny), int(maxy - hh) + 1):
                cands.append((math.hypot(xi + w / 2 - anchor[0], yi + hh / 2 - anchor[1]), xi, yi))
        cands.sort()
        for d, xi, yi in cands:
            bx = box(xi, yi, xi + w, yi + hh)
            if ins.contains(bx) and lay.clear(bx, 1.0, False):
                lay.boxes.append(bx)
                texts.append((xi, yi + 0.3 * h, h, txt))
                return (xi, yi)
        return None

    hc = hx(hxm * 0.62, 0.0)
    where["lbl_H"] = inside(Hp, (hc[0], hc[1] - 25), "SPARNAS (horizontali detalė)")
    vc = vx(f["V_cut"][0][0] + 130, 0.0)
    where["lbl_V"] = inside(Vp, vc, "LIEMUO SU KILIU (vertikali detalė)")
    if strict and (where["lbl_H"] is None or where["lbl_V"] is None):
        return None

    # ---- small labels near features
    ha, hb = hinges[0]
    ba, bb = bends[0]
    word = "aukštyn" if r["flap_te_mm"] < 0 else "žemyn"
    xs, xm = f["H_slot"]
    slotH = hx((xs + xm) / 2.0, 0.0)
    slotV = vx(xm, 0.0)
    labels = [
        ("sc_tick", "SC", tick_top),
        ("sc_circle", "SC", (cgc[0], cgc[1])),
        ("ballast", f"balastas {fc(r['m_ballast_g'])} g", (bc[0], bc[1] + 12)),
        ("flap", f"atlenkti {word} {abs(r['flap_te_mm']):.0f} mm", ((ha[0] + hb[0]) / 2, (ha[1] + hb[1]) / 2)),
        ("bend", f"galą atlenkti aukštyn {f['bend_deg']:.0f}°", ((ba[0] + bb[0]) / 2, (ba[1] + bb[1]) / 2)),
        ("slot_H", "išpjova 5 mm", slotH),
        ("slot_V", "išpjova 5 mm", slotV),
    ]
    for key, txt, anc in labels:
        w = text_w(txt, TH)
        res = lay.near(anc[0], anc[1], w, 1.35 * TH, 0)
        if not res:
            where[key] = "NOT PLACED"
            if strict:
                return None
            continue
        x0, y0, bx = res
        lay.boxes.append(bx)
        texts.append((x0, y0 + 0.3 * TH, TH, txt))
        where[key] = (round(x0, 1), round(y0, 1))
    for x, yb, h, t in texts:
        msp.add_text(t, height=h, dxfattribs={"layer": "TEXT", "style": "ISO", "insert": (x, yb)})
    doc.saveas(DXF)
    return where, title_lines


def make_pdf():
    p = subprocess.run([ms.PY, "-X", "utf8", ms.PLOT_PDF, DXF, PDF, "--size=841x594"],
                       capture_output=True, text=True, encoding="utf-8")
    if "OK:" not in p.stdout or not re.search(r"OK:\s*\S", p.stdout):
        raise RuntimeError("plot_pdf failed: " + p.stdout + p.stderr)
    return p.stdout.strip()


# ---------------------------------------------------------------- solver
def solve():
    k = 0
    while True:
        s = round(S_START - k * S_STEP, 2)
        if s < 2.0:
            raise RuntimeError("no scale fits")
        P = place(s)
        if P is not None:
            yield P
        k += 1


def build():
    tried = []
    for P in solve():
        res = build_doc(P)
        if res is not None:
            where, tl = res
            print("scale", P["s"], "X0 %.1f gap %.3f" % (P["X0"], P["gap"]), where)
            return P, where, tl, tried
        tried.append(P["s"])
        print("layout failed at s =", P["s"])


# ---------------------------------------------------------------- check
def check(P=None, where=None, tl=None, tried=None):
    import fitz, cv2, numpy as np
    from ezdxf import bbox as xbbox
    if P is None:
        for P in solve():
            break
        # rebuild deterministic placement in the same way as build(): need the first layout-feasible s
        P, where, tl, tried = build()
    f, r = P["f"], P["r"]
    cx, Ybot, hxm, X0, Y0 = P["cx"], P["Ybot"], P["hxm"], P["X0"], P["Y0"]
    rep = {"scale": P["s"], "scale_rejected_by_layout": tried, "glyphs": GLYPH_NOTE, "where": where}
    doc = ezdxf.readfile(DXF)
    msp = doc.modelspace()
    ok = True
    cut = [e for e in msp if e.dxftype() == "LWPOLYLINE" and e.dxf.layer == "CUT"]
    c1 = len(cut) == 2 and all(e.closed for e in cut)
    polys = [[(p[0], p[1]) for p in e.get_points("xy")] for e in cut]
    Hn = len(f["H_cut"])
    hi = [i for i in range(2) if len(polys[i]) == Hn]
    c1 &= len(hi) == 1
    Hp_ = polys[hi[0]] if hi else polys[0]
    Vp_ = polys[1 - hi[0]] if hi else polys[1]
    span_dxf = max(p[0] for p in Hp_) - min(p[0] for p in Hp_)
    rep["cut_closed_polylines"] = len(cut)
    rep["H_span_sheet_mm"] = round(span_dxf, 4)
    rep["span_expected"] = r["span_mm"]
    c1 &= abs(span_dxf - r["span_mm"]) <= 0.05
    # slot widths from read-back vertices
    xm = f["H_slot"][1]
    yH = Ybot + (hxm - xm)
    hv = sorted(p[0] for p in Hp_ if abs(p[1] - yH) < 1e-6)
    xV = X0 + xm
    vv = sorted(p[1] for p in Vp_ if abs(p[0] - xV) < 1e-6)
    sw_h = hv[-1] - hv[0] if len(hv) == 2 else float("nan")
    sw_v = vv[-1] - vv[0] if len(vv) == 2 else float("nan")
    rep["slot_w_H"], rep["slot_w_V"] = round(sw_h, 4), round(sw_v, 4)
    c1 &= abs(sw_h - 5.0) <= 0.01 and abs(sw_v - 5.0) <= 0.01
    # margins
    worst, lines, tb = 1e9, [], []
    for e in msp:
        if e.dxf.layer == "FRAME":
            continue
        if e.dxftype() == "TEXT":
            x0, y0, x1, y1 = text_box(e.dxf.insert.x, e.dxf.insert.y, e.dxf.height, e.dxf.text).bounds
        else:
            bb = xbbox.extents([e])
            x0, y0, x1, y1 = bb.extmin.x, bb.extmin.y, bb.extmax.x, bb.extmax.y
        worst = min(worst, x0, y0, SW - x1, SH - y1)
    rep["min_margin_mm"] = round(worst, 3)
    c1 &= worst >= M - 1e-6
    gap = Polygon(Hp_).distance(Polygon(Vp_))
    rep["gap_mm"] = round(gap, 3)
    c1 &= gap >= GAP_MIN
    rep["H_bbox"] = [round(v, 2) for v in Polygon(Hp_).bounds]
    rep["V_bbox"] = [round(v, 2) for v in Polygon(Vp_).bounds]
    # text overlaps (informative but also gates ok)
    ln = [LineString(P_ + [P_[0]]) for P_ in polys]
    for e in msp:
        if e.dxftype() == "LINE" and e.dxf.layer == "MARK":
            ln.append(LineString([(e.dxf.start.x, e.dxf.start.y), (e.dxf.end.x, e.dxf.end.y)]))
        elif e.dxftype() == "CIRCLE":
            ln.append(Point(e.dxf.center.x, e.dxf.center.y).buffer(e.dxf.radius).exterior)
        elif e.dxftype() == "TEXT":
            tb.append((e.dxf.text, text_box(e.dxf.insert.x, e.dxf.insert.y, e.dxf.height, e.dxf.text)))
    hits = [t for t, b in tb if any(b.intersects(l) for l in ln)]
    for a in range(len(tb)):
        for c in range(a + 1, len(tb)):
            if tb[a][1].intersects(tb[c][1]):
                hits.append(tb[a][0] + " x " + tb[c][0])
    qz = rep["where"]["qr"]["quiet_zone_box"]
    qzb = box(*qz)
    qhits = [t for t, b in tb if b.intersects(qzb)]
    for e in msp:
        if e.dxf.layer in ("CUT", "MARK") or (e.dxf.layer == "TEXT" and e.dxftype() == "LINE"):
            if e.dxftype() == "LWPOLYLINE":
                g = LineString([(p[0], p[1]) for p in e.get_points("xy")] + [(e.get_points("xy")[0][0], e.get_points("xy")[0][1])])
            elif e.dxftype() == "LINE":
                g = LineString([(e.dxf.start.x, e.dxf.start.y), (e.dxf.end.x, e.dxf.end.y)])
            else:
                g = Point(e.dxf.center.x, e.dxf.center.y).buffer(e.dxf.radius).exterior
            if g.intersects(qzb):
                qhits.append(e.dxftype())
    rep["text_overlaps"] = hits
    rep["quiet_zone_intrusions"] = qhits
    c1 &= not hits and not qhits
    rep["item1_dxf_ok"] = bool(c1)
    # item 2: PDF
    pdf = fitz.open(PDF)
    pg = pdf[0]
    wmm, hmm = pg.rect.width * 25.4 / 72, pg.rect.height * 25.4 / 72
    rep["pdf_pages"] = len(pdf)
    rep["pdf_page_mm"] = [round(wmm, 3), round(hmm, 3)]
    c2 = len(pdf) == 1 and abs(wmm - 841) <= 0.3 and abs(hmm - 594) <= 0.3
    rul = [e for e in msp if e.dxftype() == "LINE" and e.dxf.layer == "TEXT"
           and abs(e.dxf.start.y - e.dxf.end.y) < 1e-9 and abs(abs(e.dxf.end.x - e.dxf.start.x) - 100) < 1e-6]
    rep["ruler_dxf_count"] = len(rul)
    ruler_pdf = None
    if len(rul) == 1:
        ry = (SH - rul[0].dxf.start.y) * 72 / 25.4
        cand = []
        for dr in pg.get_drawings():
            for it in dr["items"]:
                if it[0] == "l":
                    p1, p2 = it[1], it[2]
                    if abs(p1.y - p2.y) < 0.3 and abs(p1.y - ry) < 0.5:
                        cand.append(abs(p2.x - p1.x) * 25.4 / 72)
        if cand:
            ruler_pdf = min(cand, key=lambda v: abs(v - 100))
    rep["ruler_pdf_mm"] = None if ruler_pdf is None else round(ruler_pdf, 4)
    c2 &= ruler_pdf is not None and abs(ruler_pdf - 100.0) <= 0.1
    rep["item2_pdf_ok"] = bool(c2)
    # item 3: QR decode
    k = 72 / 25.4
    clip = fitz.Rect(qz[0] * k, (SH - qz[3]) * k, qz[2] * k, (SH - qz[1]) * k)
    pix = pg.get_pixmap(dpi=300, clip=clip)
    arr = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width, pix.n)
    gray = cv2.cvtColor(arr[:, :, :3].copy(), cv2.COLOR_RGB2GRAY)
    txt, _, _ = cv2.QRCodeDetector().detectAndDecode(gray)
    url = json.load(open(os.path.join(OUT, "qr.json"), encoding="utf-8"))["url"]
    rep["qr_decoded"] = txt
    rep["qr_render_dpi"] = 300
    c3 = txt == url
    rep["item3_qr_ok"] = bool(c3)
    # item 4: PNGs for visual reading
    pg.get_pixmap(dpi=60).save(PNG)
    tx, ty, tw_, th_ = rep["where"]["title"]
    x0 = tx - 3
    x1 = qz[2] + 3
    y0, y1 = min(ty, qz[1]) - 3, max(ty + th_, qz[3]) + 3
    pg.get_pixmap(dpi=150, clip=fitz.Rect(x0 * k, (SH - y1) * k, x1 * k, (SH - y0) * k)).save(
        os.path.join(OUT, "a1_crop_title.png"))
    sx, sy = P["Hs"] and (cx, Ybot + (hxm - xm))
    pg.get_pixmap(dpi=150, clip=fitz.Rect((sx - 40) * k, (SH - sy - 30) * k, (sx + 40) * k, (SH - sy + 30) * k)).save(
        os.path.join(OUT, "a1_crop_slot.png"))
    rep["pngs"] = ["F07_A1.png (60 dpi)", "a1_crop_title.png (150 dpi)", "a1_crop_slot.png (150 dpi)"]
    rep["title_lines"] = tl
    rep["ok"] = bool(c1 and c2 and c3)
    json.dump(rep, open(CHK, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("CHECK ok =", rep["ok"], {k_: rep[k_] for k_ in ("scale", "item1_dxf_ok", "item2_pdf_ok", "item3_qr_ok",
                                                         "min_margin_mm", "gap_mm", "qr_decoded", "ruler_pdf_mm")})
    return rep


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    P, where, tl, tried = build()
    if not a.check:
        make_pdf()
    check(P, where, tl, tried)
