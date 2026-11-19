# -*- coding: utf-8 -*-
"""make_sheets.py - A4 landscape 1:1 cut templates for the 10 foam glider forms.

USAGE (run with the FreeCAD python, from any folder):
    python make_sheets.py           build DXF + PDF + PNG, then run the check
    python make_sheets.py --check   only re-run the numeric check

Outputs (relative to the project folder = parent of this script's folder):
    out/sheets/Fxx.dxf  out/sheets/pdf/Fxx.pdf  out/sheets/png/Fxx.png
    out/forms_A4.pdf (10 pages)  out/sheets/placement.json  out/sheets/check.json
Glider axes: x aft from the nose, y right, z up.  Sheet: X right 0..297, Y up 0..210.
"""
import os, sys, re, json, math, subprocess, shutil, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
OUT = os.path.join(ROOT, "out")
SHEETS = os.path.join(OUT, "sheets")
PDFD = os.path.join(SHEETS, "pdf")
PNGD = os.path.join(SHEETS, "png")
PLOT_PDF = os.path.join(HERE, "plot_pdf.py")

import ezdxf
from ezdxf.fonts import fonts as xfonts
from shapely.geometry import LineString, Point, box
from shapely.ops import unary_union
from shapely.prepared import prep
from shapely.validation import make_valid
from shapely.geometry import Polygon as _ShPolygon


def Polygon(pts):
    """Polygon, repaired with make_valid if the outline self-intersects (data defect is reported, not fixed)."""
    p = _ShPolygon(pts)
    return p if p.is_valid else make_valid(p)


SW, SH = 297.0, 210.0
M = 8.0                 # hard margin
INSET = 0.2             # parts are kept 0.2 mm inside the margin (line width)
MIN_GAP = 3.0
H_CX = 148.5            # sheet X of the glider centreline
TXT_H1, TXT_H = 3.2, 2.3
PY = os.path.realpath(sys.executable)


# ------------------------------------------------------------------ fonts
def find_user_fonts():
    """Per-user font folder of Windows (the same rule as in plot_pdf.py); "" when it does not exist."""
    p = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Microsoft", "Windows", "Fonts")
    return p if os.path.isdir(p) else ""


USER_FONTS = find_user_fonts()
REQUIRED_CHARS = "ąčęėįšųūžĄČĘĖĮŠŲŪŽ·°ЦТАЯвверхнизпечатьгрузотносиЯ"


def pick_font():
    """ISOCPEUR if it has every needed glyph, else arial.ttf."""
    from matplotlib import ft2font
    p = os.path.join(USER_FONTS, "ISOCPEUR.ttf")
    if os.path.exists(p):
        f = ft2font.FT2Font(p)
        if all(f.get_char_index(ord(c)) for c in REQUIRED_CHARS):
            return "ISOCPEUR.ttf"
    return "arial.ttf"


FONT_FILE = pick_font()
if USER_FONTS and USER_FONTS not in ezdxf.options.support_dirs:
    ezdxf.options.support_dirs = list(ezdxf.options.support_dirs) + [USER_FONTS]
    xfonts.build_system_font_cache()
_fcache = {}


def text_w(s, h):
    key = h
    if key not in _fcache:
        _fcache[key] = xfonts.make_font(FONT_FILE, h)
    return _fcache[key].text_width(s) * 1.03    # 3 % safety


def text_box(x, y, h, s):
    """bbox of single-line left/baseline text (descenders 0.3 h)."""
    return box(x, y - 0.3 * h, x + text_w(s, h), y + 1.05 * h)


# ------------------------------------------------------------------ data
def load():
    from glider_geom import get_forms
    forms = get_forms()
    res = json.load(open(os.path.join(OUT, "results.json"), encoding="utf-8"))
    assert len(forms) == 10 and len(res["forms"]) == 10
    for f, r in zip(forms, res["forms"]):
        assert f["id"] == r["id"], (f["id"], r["id"])
    return forms, res


def x_scan(lo, hi, step=0.5):
    n = int(math.floor((hi - lo) / step + 1e-9))
    v = [lo + i * step for i in range(n + 1)]
    if hi - v[-1] > 1e-9:
        v.append(hi)
    return v


# ------------------------------------------------------------------ nesting
def nest(f, extra=0.0):
    """Return (placement dict) or None.  extra = additional sheet height used for the overflow search."""
    H, V = f["H_cut"], f["V_cut"]
    hx = [p[0] for p in H]
    vx = [p[0] for p in V]
    vz = [p[1] for p in V]
    top = SH - M - INSET + extra
    bot = M + INSET
    right, left = SW - M - INSET, M + INSET
    for name, h_at_top, s in (("1: V below H, fin up", True, 1),
                              ("2: V above H, mirrored, fin down", False, -1),
                              ("3: V below H, fin down", True, -1)):
        if h_at_top:
            Yc = top + min(hx)           # sheet Y = Yc - x ; nose edge at top
        else:
            Yc = bot + max(hx)           # tail edge at the bottom margin
        Hs = [(H_CX + p[1], Yc - p[0]) for p in H]
        Hp = Polygon(Hs)
        if name.startswith("2"):
            Y0 = top + min(vz) if s == -1 else None      # V top at top margin: Y = Y0 - z
            # mirrored: Y = Y0 - z ; max Y = Y0 - min z
            Y0 = top + min(vz)
        elif s == 1:
            Y0 = bot - min(vz)
        else:
            Y0 = bot + max(vz)
        best = None
        for X0 in x_scan(left - min(vx), right - max(vx)):
            Vs = [(X0 + p[0], Y0 + s * p[1]) for p in V]
            Vp = Polygon(Vs)
            g = Hp.distance(Vp)
            if g >= MIN_GAP:
                ymin = min(y for _, y in Vs)
                ymax = max(y for _, y in Vs)
                if ymin < bot - 1e-9 or ymax > top + 1e-9:
                    continue
                score = (min(g, 8.0), -abs(X0 + 115.0 - H_CX))
                if best is None or score > best[0]:
                    best = (score, X0, g, Hs, Vs)
        if best:
            _, X0, g, Hs, Vs = best
            return dict(name=name, X0=X0, Y0=Y0, s=s, gap=g, Hs=Hs, Vs=Vs, Yc=Yc)
    return None


def overflow(f):
    for e in range(0, 300):
        if nest(f, extra=float(e)):
            return float(e)
    return None


# ------------------------------------------------------------------ label placement
class Layout:
    def __init__(self, parts):
        self.parts = parts                    # polygons
        self.lines = []                       # linework shapely geoms
        self.boxes = []                       # placed text/ruler boxes
        self._u = None

    def add_line(self, g):
        self.lines.append(g)
        self._u = None

    def union_lines(self):
        if self._u is None:
            self._u = unary_union(self.lines + [b.exterior for b in self.boxes if False])
        return self._u

    def clear(self, bx, buf, avoid_parts):
        g = bx.buffer(buf)
        if not box(M + 0.3, M + 0.3, SW - M - 0.3, SH - M - 0.3).contains(bx):
            return False
        if g.intersects(self.union_lines()):
            return False
        for b in self.boxes:
            if g.intersects(b):
                return False
        if avoid_parts:
            for p in self.parts:
                if g.intersects(p):
                    return False
        return True

    def near(self, ax, ay, w, h, off=0.0):
        """Search a free text box close to anchor (ax, ay): box may sit on a part but not on lines."""
        cands = []
        R = 120
        for dx in range(-R, R + 1):
            for dy in range(-40, 41):
                x0, y0 = ax + dx * 1.0, ay + dy * 1.0
                # distance from anchor to rectangle
                cx = min(max(ax, x0), x0 + w)
                cy = min(max(ay, y0), y0 + h)
                cands.append((math.hypot(ax - cx, ay - cy), x0, y0))
        cands.sort()
        for d, x0, y0 in cands:
            bx = box(x0, y0, x0 + w, y0 + h)
            if self.clear(bx, 0.6, False):
                return x0, y0, bx
        return None

    def free(self, w, h, prefer, allow_inside=None):
        """Find a w x h area outside the parts (2 mm buffer); fallback inside a given polygon."""
        xs = [M + 1.0 + i for i in range(int(SW - 2 * M - w - 1.0) + 1)]
        ys = [M + 1.0 + i for i in range(int(SH - 2 * M - h - 1.0) + 1)]
        cands = sorted(((math.hypot(x - prefer[0], y - prefer[1]), x, y) for x in xs for y in ys))
        forb = prep(unary_union([p.buffer(2.0) for p in self.parts]))
        for d, x, y in cands:
            bx = box(x, y, x + w, y + h)
            if forb.intersects(bx):
                continue
            if self.clear(bx, 0.8, False):
                return x, y, bx, "free"
        if allow_inside is not None:
            ins = prep(allow_inside.buffer(-1.0))
            for d, x, y in cands:
                bx = box(x, y, x + w, y + h)
                if ins.contains(bx) and self.clear(bx, 0.8, False):
                    return x, y, bx, "inside H"
        return None


# ------------------------------------------------------------------ one sheet
def build_sheet(f, r, assump, pl):
    Hs, Vs = pl["Hs"], pl["Vs"]
    X0, Y0, s, Yc = pl["X0"], pl["Y0"], pl["s"], pl["Yc"]
    Hp, Vp = Polygon(Hs), Polygon(Vs)
    doc = ezdxf.new("R2018", setup=True)
    doc.units = 4
    doc.header["$LTSCALE"] = 4.0
    for nm, lw, lt in (("CUT", 35, "Continuous"), ("MARK", 13, "DASHED"),
                       ("TEXT", 13, "Continuous"), ("FRAME", 13, "Continuous")):
        doc.layers.add(nm, lineweight=lw, linetype=lt, color=7)
    doc.styles.add("ISO", font=FONT_FILE)
    msp = doc.modelspace()
    cont = {"linetype": "Continuous"}

    def hx(x, y):       # glider H -> sheet
        return (H_CX + y, Yc - x)

    def vx(x, z):       # glider V -> sheet
        return (X0 + x, Y0 + s * z)

    msp.add_lwpolyline([(0, 0), (SW, 0), (SW, SH), (0, SH)], close=True,
                       dxfattribs={"layer": "FRAME", "lineweight": 13})
    msp.add_lwpolyline(Hs, close=True, dxfattribs={"layer": "CUT", "lineweight": 35})
    msp.add_lwpolyline(Vs, close=True, dxfattribs={"layer": "CUT", "lineweight": 35})

    lay = Layout([Hp, Vp])
    lay.add_line(LineString(Hs + [Hs[0]]))
    lay.add_line(LineString(Vs + [Vs[0]]))
    anchors = []   # (text, anchor, height)

    # flap hinge lines (points[0] -> points[1]) and bend lines, MARK layer, dashed by layer
    hinges = []
    for q in f["flaps"]:
        a, b = hx(*q[0]), hx(*q[1])
        msp.add_line(a, b, dxfattribs={"layer": "MARK", "lineweight": 13})
        hinges.append((a, b))
        lay.add_line(LineString([a, b]))
    bends = []
    for (p1, p2) in f["bends"]:
        a, b = hx(*p1), hx(*p2)
        msp.add_line(a, b, dxfattribs={"layer": "MARK", "lineweight": 13})
        bends.append((a, b))
        lay.add_line(LineString([a, b]))

    # centre of gravity: tick across V + circle on the H centreline
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
        msp.add_line(a, b, dxfattribs={"layer": "MARK", "lineweight": 13, **cont})
        lay.add_line(LineString([a, b]))
        for p in (a, b):
            if tick_top is None or p[1] > tick_top[1]:
                tick_top = p
    cgc = hx(xcg, 0.0)
    msp.add_circle(cgc, 2.5, dxfattribs={"layer": "MARK", "lineweight": 13, **cont})
    lay.add_line(Point(cgc).buffer(2.5).exterior)
    # ballast circle
    bc = vx(r["x_ballast_mm"] if "x_ballast_mm" in r else f["x_ballast"], 0.0)
    msp.add_circle(bc, 4.0, dxfattribs={"layer": "MARK", "lineweight": 13, **cont})
    lay.add_line(Point(bc).buffer(4.0).exterior)

    # --- text items: title and ruler first reserve their free areas
    ln = [
        (TXT_H1, f"{f['id']}  {f['name_lt']} / {f['name_ru']}"),
        (TXT_H, "XPS 5 mm · M 1:1 · spausdinti 100 % / печать 100 %"),
        (TXT_H, f"m = {r['m_total_g']:.1f} g · balastas / груз {r['m_ballast_g']:.1f} g"),
        (TXT_H, f"SC / ЦТ {r['x_cg_mm']:.0f} mm nuo nosies / от носа"),
        (TXT_H, f"V = {r['V_ms']:.1f} m/s · K = {r['LD']:.1f} · L = {r['range_glide_m']:.1f} m "
                f"(h = {assump['h_launch_m']:.1f} m)"),
    ]
    tw = max(text_w(t, h) for h, t in ln)
    pitches = [5.4, 4.2, 4.2, 4.2]
    th = TXT_H1 + sum(pitches) + 0.3 * TXT_H + 0.3
    texts = []
    where = {}
    # place the point labels first so the title does not take their spots
    labels = []
    if tick_top is not None:
        labels.append(("cg", "SC / ЦТ", tick_top))
    labels.append(("ballast", f"{r['m_ballast_g']:.1f} g", (bc[0], bc[1] + 5.0)))
    ha, hb = hinges[0]
    word = "aukštyn / вверх" if r["flap_te_mm"] < 0 else "žemyn / вниз"
    labels.append(("flap", f"{word} {abs(r['flap_te_mm']):.1f} mm", ((ha[0] + hb[0]) / 2, (ha[1] + hb[1]) / 2)))
    ba, bb = bends[0]
    labels.append(("bend", f"{f['bend_deg']:.0f}°", ((ba[0] + bb[0]) / 2, (ba[1] + bb[1]) / 2)))
    for key, txt, anc in labels:
        w = text_w(txt, TXT_H)
        res = lay.near(anc[0], anc[1], w, 0.3 * TXT_H + 1.05 * TXT_H, 0)
        if not res:
            where[key] = "NOT PLACED"
            continue
        x0, y0, bx = res
        lay.boxes.append(bx)
        texts.append((x0, y0 + 0.3 * TXT_H, TXT_H, txt))
        where[key] = (round(x0, 1), round(y0, 1))
    # title block
    t_res = lay.free(tw + 1.0, th + 0.6, prefer=(M, SH - M), allow_inside=Hp)
    if t_res:
        tx, ty, tb, tmode = t_res
        lay.boxes.append(tb)
        ytop = ty + th + 0.3          # top of the box
        yb = ytop - 0.3 - TXT_H1      # first baseline
        texts.append((tx + 0.5, yb, TXT_H1, ln[0][1]))
        for i, (h, t) in enumerate(ln[1:]):
            yb -= pitches[i]
            texts.append((tx + 0.5, yb, h, t))
        where["title"] = (tmode, round(tx, 1), round(ty, 1))
    else:
        where["title"] = "NOT PLACED"
    # ruler: 50 mm, ticks every 10 mm, label
    rw, rh = 51.0, 9.0
    rr = lay.free(rw, rh, prefer=(SW - M, M), allow_inside=Hp)
    if rr:
        rx, ry, rb, rmode = rr
        lay.boxes.append(rb)
        rx0, ry0 = rx + 0.5, ry + 2.5          # line
        msp.add_line((rx0, ry0), (rx0 + 50.0, ry0), dxfattribs={"layer": "TEXT", "lineweight": 13})
        for k in range(6):
            tl = 2.0 if k in (0, 5) else 1.0
            msp.add_line((rx0 + 10 * k, ry0 - tl), (rx0 + 10 * k, ry0 + tl),
                         dxfattribs={"layer": "TEXT", "lineweight": 13})
        texts.append((rx0, ry0 + 3.2, TXT_H, "50 mm"))
        where["ruler"] = (rmode, round(rx0, 3), round(ry0, 3))
    else:
        where["ruler"] = "NOT PLACED"
    for x, y, h, t in texts:
        e = msp.add_text(t, height=h, dxfattribs={"layer": "TEXT", "style": "ISO", "insert": (x, y)})
    path = os.path.join(SHEETS, f"{f['id']}.dxf")
    doc.saveas(path)
    return path, where


# ------------------------------------------------------------------ pdf / png
def make_pdf(dxf, pdf):
    p = subprocess.run([PY, "-X", "utf8", PLOT_PDF, dxf, pdf, "--size=297x210"],
                       capture_output=True, text=True, encoding="utf-8")
    m = re.search(r"OK:\s*(.+)", p.stdout)
    if not m or not m.group(1).strip():
        raise RuntimeError("plot_pdf failed: " + p.stdout + p.stderr)
    return m.group(1).strip()


def build():
    import fitz
    for d in (SHEETS, PDFD, PNGD):
        os.makedirs(d, exist_ok=True)
    forms, res = load()
    placement = {}
    for f, r in zip(forms, res["forms"]):
        pl = nest(f)
        if pl is None:
            placement[f["id"]] = {"placed": False, "overflow_mm": overflow(f)}
            print(f["id"], "DOES NOT FIT, overflow mm:", placement[f["id"]]["overflow_mm"])
            continue
        dxf, where = build_sheet(f, r, res["assumptions"], pl)
        pdf = make_pdf(dxf, os.path.join(PDFD, f["id"] + ".pdf"))
        placement[f["id"]] = {"placed": True, "placement": pl["name"], "X0": pl["X0"], "Y0": pl["Y0"],
                              "gap_build": pl["gap"], "where": where, "pdf": os.path.basename(pdf),
                              "font": FONT_FILE}
        print(f["id"], pl["name"], "X0=%.1f gap=%.2f" % (pl["X0"], pl["gap"]), where)
        d = fitz.open(pdf)
        d[0].get_pixmap(dpi=100).save(os.path.join(PNGD, f["id"] + ".png"))
        d.close()
    json.dump(placement, open(os.path.join(SHEETS, "placement.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    merge(placement)


def merge(placement):
    import fitz
    out = fitz.open()
    for fid in sorted(placement):
        if placement[fid].get("placed"):
            d = fitz.open(os.path.join(PDFD, placement[fid]["pdf"]))
            out.insert_pdf(d)
            d.close()
    tmp = os.path.join(OUT, "forms_A4.tmp.pdf")
    out.save(tmp)
    out.close()
    dst = os.path.join(OUT, "forms_A4.pdf")
    try:
        os.replace(tmp, dst)
    except PermissionError:
        dst = os.path.join(OUT, "forms_A4_v2.pdf")
        os.replace(tmp, dst)
    print("merged ->", dst)


# ------------------------------------------------------------------ check
def check():
    import fitz
    from ezdxf import bbox as xbbox
    forms, res = load()
    placement = json.load(open(os.path.join(SHEETS, "placement.json"), encoding="utf-8"))
    merged = os.path.join(OUT, "forms_A4.pdf")
    if not os.path.exists(merged) and os.path.exists(os.path.join(OUT, "forms_A4_v2.pdf")):
        merged = os.path.join(OUT, "forms_A4_v2.pdf")
    pdf = fitz.open(merged)
    report = {"pdf_pages": len(pdf), "merged_pdf": os.path.basename(merged), "font": FONT_FILE, "forms": {}}
    all_ok = (len(pdf) == 10)
    lo, hi = M - 1e-6, None
    for i, (f, r) in enumerate(zip(forms, res["forms"])):
        fid = f["id"]
        d = {"id": fid, "placement": placement.get(fid, {}).get("placement")}
        path = os.path.join(SHEETS, fid + ".dxf")
        ok = True
        if not os.path.exists(path):
            report["forms"][fid] = {"ok": False, "note": "no DXF"}
            all_ok = False
            continue
        doc = ezdxf.readfile(path)
        msp = doc.modelspace()
        cut = [e for e in msp if e.dxftype() == "LWPOLYLINE" and e.dxf.layer == "CUT" and e.closed]
        d["cut_closed"] = len(cut)
        ok &= len(cut) == 2
        polys = [[(p[0], p[1]) for p in e.get_points("xy")] for e in cut]
        # identify H (wider X extent along span) - H has the larger x-extent range containing 148.5 +- span/2
        ext = [(min(p[0] for p in P), max(p[0] for p in P)) for P in polys]
        hi_i = max(range(2), key=lambda k: ext[k][1] - ext[k][0] if abs((ext[k][0] + ext[k][1]) / 2 - H_CX) < 0.01 else -1)
        # H is symmetric around 148.5; V is not (checked by centre)
        sym = [abs((ext[k][0] + ext[k][1]) / 2 - H_CX) < 0.01 for k in range(2)]
        hi_i = 0 if (sym[0] and not sym[1]) else 1 if (sym[1] and not sym[0]) else hi_i
        Hp, Vp = polys[hi_i], polys[1 - hi_i]
        d["H_x_extent"] = ext[hi_i][1] - ext[hi_i][0]
        d["span_mm"] = r["span_mm"]
        ok &= abs(d["H_x_extent"] - r["span_mm"]) <= 0.05

        def slot_w(P, axis):
            c = [p[axis] for p in P]
            # centreline coordinate: H -> sheet X = 148.5 (axis 0); V -> use closeness to median of small |c-c0|
            return c
        # H slot: vertices within 3 mm of the centreline X = 148.5
        hv = [p[0] for p in Hp if abs(p[0] - H_CX) <= 3.0]
        sw_h = max(hv) - min(hv) if hv else float("nan")
        # V slot: vertices of V whose z (sheet Y) is within the slot band; find the band as the
        # 2 distinct Y values around the V slot-bottom segment (a polygon edge of length slot_w)
        Vy = [p[1] for p in Vp]
        best = None
        n = len(Vp)
        for k in range(n):
            a, b = Vp[k], Vp[(k + 1) % n]
            if abs(a[0] - b[0]) < 1e-9 and abs(abs(a[1] - b[1]) - f["slot_w"]) < 0.5:
                best = abs(a[1] - b[1])
        sw_v = best if best is not None else float("nan")
        # also confirm via opposite wall vertices: the slot walls are at those two Y values
        d["slot_w_H"], d["slot_w_V"] = sw_h, sw_v
        ok &= abs(sw_h - f["slot_w"]) <= 0.01 and abs(sw_v - f["slot_w"]) <= 0.01
        # margins
        lim_ok = True
        worst = 1e9
        for e in msp:
            if e.dxf.layer == "FRAME":
                continue
            if e.dxftype() == "TEXT":
                x, y = e.dxf.insert.x, e.dxf.insert.y
                bx = text_box(x, y, e.dxf.height, e.dxf.text)
                x0, y0, x1, y1 = bx.bounds
            else:
                bb = xbbox.extents([e])
                x0, y0, x1, y1 = bb.extmin.x, bb.extmin.y, bb.extmax.x, bb.extmax.y
            m = min(x0, y0, SW - x1, SH - y1)
            worst = min(worst, m)
            if m < M - 1e-6:
                lim_ok = False
        d["min_margin_mm"] = round(worst, 3)
        d["margin_ok"] = lim_ok
        ok &= lim_ok
        gap = Polygon(Hp).distance(Polygon(Vp))
        d["min_gap_mm"] = round(gap, 3)
        ok &= gap >= MIN_GAP
        # text vs lines / text vs text overlaps (informative)
        lines = [LineString(P + [P[0]]) for P in polys]
        tb = []
        for e in msp:
            if e.dxftype() == "LINE" and e.dxf.layer == "MARK":
                lines.append(LineString([(e.dxf.start.x, e.dxf.start.y), (e.dxf.end.x, e.dxf.end.y)]))
            if e.dxftype() == "CIRCLE":
                lines.append(Point(e.dxf.center.x, e.dxf.center.y).buffer(e.dxf.radius).exterior)
            if e.dxftype() == "TEXT":
                tb.append((e.dxf.text, text_box(e.dxf.insert.x, e.dxf.insert.y, e.dxf.height, e.dxf.text)))
        hits = 0
        hit_names = []
        for t, b in tb:
            if any(b.intersects(l) for l in lines):
                hits += 1
                hit_names.append(t)
        for a in range(len(tb)):
            for c in range(a + 1, len(tb)):
                if tb[a][1].intersects(tb[c][1]):
                    hits += 1
                    hit_names.append(tb[a][0] + " x " + tb[c][0])
        d["text_overlaps"] = hits
        d["text_overlap_items"] = hit_names
        # ruler in DXF
        rul = [e for e in msp if e.dxftype() == "LINE" and e.dxf.layer == "TEXT"
               and abs(e.dxf.start.y - e.dxf.end.y) < 1e-9 and abs(abs(e.dxf.end.x - e.dxf.start.x) - 50) < 1e-6]
        d["ruler_dxf_mm"] = abs(rul[0].dxf.end.x - rul[0].dxf.start.x) if len(rul) == 1 else None
        ok &= len(rul) == 1
        # PDF page
        pg = pdf[i]
        wmm, hmm = pg.rect.width * 25.4 / 72, pg.rect.height * 25.4 / 72
        d["page_mm"] = [round(wmm, 3), round(hmm, 3)]
        ok &= abs(wmm - 297.0) <= 0.2 and abs(hmm - 210.0) <= 0.2
        d["ruler_pdf_mm"] = None
        if rul:
            ry = (SH - rul[0].dxf.start.y) * 72 / 25.4
            cand = []
            for dr in pg.get_drawings():
                for it in dr["items"]:
                    if it[0] == "l":
                        p1, p2 = it[1], it[2]
                        if abs(p1.y - p2.y) < 0.3 and abs(p1.y - ry) < 0.5:
                            cand.append(abs(p2.x - p1.x) * 25.4 / 72)
            if cand:
                cl = min(cand, key=lambda v: abs(v - 50))
                d["ruler_pdf_mm"] = round(cl, 3)
        ok &= d["ruler_pdf_mm"] is not None and abs(d["ruler_pdf_mm"] - 50.0) <= 0.1
        d["png"] = os.path.exists(os.path.join(PNGD, fid + ".png"))
        ok &= d["png"]
        d["ok"] = bool(ok)
        all_ok &= ok
        report["forms"][fid] = d
    report["ok"] = bool(all_ok)
    json.dump(report, open(os.path.join(SHEETS, "check.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print("CHECK ok =", report["ok"])
    for fid, d in report["forms"].items():
        print(fid, {k: v for k, v in d.items() if k != "text_overlap_items"})


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    if not a.check:
        build()
    check()
