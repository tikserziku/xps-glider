# -*- coding: utf-8 -*-
"""make_3d.py - exact parametric 3D models of the 10 foam gliders (build123d) -> STEP + GLB + check.json.
Run with the cad-venv python:  python make_3d.py
All numbers come from glider_geom.get_forms(); nothing is typed here except export tolerances.
"""
import os, sys, json, struct, math
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from glider_geom import get_forms
from build123d import (Wire, Face, Solid, Compound, Vector, Plane, Box, Color, Location,
                       export_step, export_gltf, Unit)

OUT = os.path.join(os.path.dirname(HERE), "out", "3d")
os.makedirs(OUT, exist_ok=True)
LIN_DEFL, ANG_DEFL = 0.1, 0.2
VOL_TOL_PCT = 0.5


def shoelace(pts):
    s = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        s += x0 * y1 - x1 * y0
    return abs(s) / 2.0


def clean(pts):
    pts = [tuple(map(float, p)) for p in pts]
    if pts[0] == pts[-1]:
        pts = pts[:-1]
    return pts


def plate(pts3, normal_dir, t):
    """Extrude closed polygon (3D points) symmetric about its plane by thickness t."""
    face = Face(Wire.make_polygon([Vector(*p) for p in pts3], close=True))
    n = Vector(*normal_dir)
    if face.normal_at().dot(n) < 0:
        face = -face
    solid = Solid.extrude(face, n * t)
    return solid.moved(Location(n * (-t / 2.0)))


def solid_of(shape):
    ss = shape.solids()
    return ss[0] if len(ss) == 1 else Compound(list(ss))


def build(form):
    t = float(form["t"])
    hp = clean(form["H_cut"])
    vp = clean(form["V_cut"])
    H = plate([(x, y, 0.0) for x, y in hp], (0, 0, 1), t)
    V = plate([(x, 0.0, z) for x, z in vp], (0, 1, 0), t)
    yb = abs(float(form["bends"][0][0][1]))
    ang = float(form["bend_deg"])
    big = 1e4
    centre = H & Box(big, 2 * yb, big)
    right = H & Box(big, big, big).moved(Location((0, yb + big / 2, 0)))
    left = H & Box(big, big, big).moved(Location((0, -yb - big / 2, 0)))
    from build123d import Axis
    right = right.rotate(Axis((0, yb, 0), (1, 0, 0)), ang)      # +y tip goes toward +z
    left = left.rotate(Axis((0, -yb, 0), (1, 0, 0)), -ang)      # -y tip goes toward +z
    pieces = [s for part in (centre, right, left) for s in part.solids()]
    Hc = Compound(pieces, label="H")
    Hc.color = Color(0.93, 0.93, 0.90)
    Vc = Compound(list(V.solids()), label="V")
    Vc.color = Color(1.0, 0.45, 0.10)
    asm = Compound(children=[Hc, Vc], label=form["id"])
    return asm, H, V, hp, vp, t


def read_glb(path):
    with open(path, "rb") as f:
        data = f.read()
    magic, ver, total = struct.unpack_from("<4sII", data, 0)
    assert magic == b"glTF" and total == len(data)
    off, js, binchunk = 12, None, None
    while off < len(data):
        ln, ty = struct.unpack_from("<I4s", data, off)
        body = data[off + 8: off + 8 + ln]
        if ty == b"JSON":
            js = json.loads(body.decode("utf-8"))
        elif ty.startswith(b"BIN"):
            binchunk = body
        off += 8 + ln
    return len(data), js


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def node_matrix(n):
    if "matrix" in n:
        m = n["matrix"]  # column-major
        return [[m[c * 4 + r] for c in range(4)] for r in range(4)]
    T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    S = [r[:] for r in T]
    R = [r[:] for r in T]
    if "translation" in n:
        for i in range(3):
            T[i][3] = n["translation"][i]
    if "scale" in n:
        for i in range(3):
            S[i][i] = n["scale"][i]
    if "rotation" in n:
        x, y, z, w = n["rotation"]
        R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
             [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
             [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
             [0, 0, 0, 1]]
    return mat_mul(T, mat_mul(R, S))


def glb_stats(path):
    size, js = read_glb(path)
    nodes = js["nodes"]
    names = [n.get("name") for n in nodes]
    meshes = js.get("meshes", [])
    acc = js["accessors"]
    lo, hi = [1e30] * 3, [-1e30] * 3
    tris = 0
    I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    def walk(i, parent):
        nonlocal tris
        n = nodes[i]
        m = mat_mul(parent, node_matrix(n))
        if "mesh" in n:
            for p in meshes[n["mesh"]]["primitives"]:
                a = acc[p["attributes"]["POSITION"]]
                mn, mx = a["min"], a["max"]
                for cx in (mn[0], mx[0]):
                    for cy in (mn[1], mx[1]):
                        for cz in (mn[2], mx[2]):
                            for k in range(3):
                                v = m[k][0] * cx + m[k][1] * cy + m[k][2] * cz + m[k][3]
                                lo[k] = min(lo[k], v)
                                hi[k] = max(hi[k], v)
                if "indices" in p:
                    tris += acc[p["indices"]]["count"] // 3
                else:
                    tris += a["count"] // 3
        for c in n.get("children", []):
            walk(c, m)

    scene = js["scenes"][js.get("scene", 0)]
    for r in scene["nodes"]:
        walk(r, I)
    return {"bytes": size, "node_names": names, "mesh_count": len(meshes), "triangles": tris,
            "glb_min": lo, "glb_max": hi, "asset": js.get("asset", {})}


def main():
    forms = get_forms()
    assert len(forms) == 10
    res, all_ok = {}, True
    for f in forms:
        fid = f["id"]
        asm, H, V, hp, vp, t = build(f)
        h_exp, v_exp = shoelace(hp) * t, shoelace(vp) * t
        h_diff = (H.volume - h_exp) / h_exp * 100.0
        v_diff = (V.volume - v_exp) / v_exp * 100.0
        bb = asm.bounding_box()
        bbmm = {"min": [bb.min.X, bb.min.Y, bb.min.Z], "max": [bb.max.X, bb.max.Y, bb.max.Z],
                "size": [bb.size.X, bb.size.Y, bb.size.Z]}
        step = os.path.join(OUT, fid + ".step")
        glb = os.path.join(OUT, fid + ".glb")
        export_step(asm, step)
        export_gltf(asm, glb, unit=Unit.MM, binary=True,
                    linear_deflection=LIN_DEFL, angular_deflection=ANG_DEFL)
        g = glb_stats(glb)
        # mapping hypothesis from the glTF exporter: GLB (X,Y,Z) = s*(x, z, -y); derive s from extents
        size = [g["glb_max"][k] - g["glb_min"][k] for k in range(3)]
        bs = bbmm["size"]
        cand = {"glb_x=x": size[0], "glb_y=z": size[1], "glb_z=y": size[2]}
        scale = size[0] / bs[0] if bs[0] else float("nan")
        unit = "metres" if abs(scale - 0.001) < 1e-5 else ("millimetres" if abs(scale - 1) < 1e-3 else "other(%g)" % scale)
        mapped = [size[0] / scale, size[1] / scale, size[2] / scale]
        expect = [bs[0], bs[2], bs[1]]
        ext_ok = all(abs(a - b) <= max(0.05, 0.002 * b) for a, b in zip(mapped, expect))
        # sign check: GLB z-range should be -y range, GLB y-range = z range
        zmap_ok = (abs(g["glb_min"][1] / scale - bbmm["min"][2]) < 0.2 and
                   abs(g["glb_max"][1] / scale - bbmm["max"][2]) < 0.2 and
                   abs(g["glb_min"][0] / scale - bbmm["min"][0]) < 0.2 and
                   abs(g["glb_min"][2] / scale + bbmm["max"][1]) < 0.2)
        names = [n for n in g["node_names"] if n]
        kids = ("H" in names) and ("V" in names)
        ok = (abs(h_diff) <= VOL_TOL_PCT and abs(v_diff) <= VOL_TOL_PCT and kids and ext_ok and zmap_ok
              and g["bytes"] > 1024 and g["triangles"] > 0 and os.path.getsize(step) > 0)
        all_ok &= ok
        res[fid] = {"ok": ok, "H_volume_mm3": H.volume, "H_expected_mm3": h_exp, "H_diff_pct": h_diff,
                    "V_volume_mm3": V.volume, "V_expected_mm3": v_exp, "V_diff_pct": v_diff,
                    "bbox_mm": bbmm, "glb": g, "glb_unit": unit, "glb_scale_to_mm": scale,
                    "glb_extents_mapped_mm": mapped, "expected_extents_mm_x_z_y": expect,
                    "extents_ok": ext_ok, "axis_signs_ok": zmap_ok, "children_in_glb": kids,
                    "step_bytes": os.path.getsize(step)}
        print(fid, "ok" if ok else "FAIL", "Hd%.4f Vd%.4f" % (h_diff, v_diff), "tri", g["triangles"],
              "bytes", g["bytes"], unit, flush=True)
    out = {"ok": bool(all_ok),
           "glb_mapping": "GLB X = model x (aft), GLB Y(up) = model z, GLB Z = -model y; unit per forms[*].glb_unit",
           "forms": res}
    with open(os.path.join(OUT, "check.json"), "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
    print("ALL_OK", all_ok)


if __name__ == "__main__":
    main()
