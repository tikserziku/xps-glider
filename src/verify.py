# -*- coding: utf-8 -*-
"""verify.py - one gate for the whole project -> out\\check.json (ok true/false).

Checks (all numbers re-read from the produced files, nothing is trusted from reports):
  1. geometry self_check (simple polygons) and aero selftest (VLM benchmarks, grid convergence, sim vs polar);
  2. every form: static margin 5..15 %, trimmed CL below CL_MAX, ballast >= 0;
  3. out\\3d\\check.json and out\\sheets\\check.json are ok;
  4. the page physics (JS port, out\\shots\\selfcheck.json written by page\\shot.mjs) against aero.py:
     mock-up scale and scale 2.8 (the JS scales the coefficients analytically, aero.py re-runs the VLM).
USAGE: python verify.py      (tools\\cad-venv python)
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import aero  # noqa: E402
from glider_geom import get_forms, self_check  # noqa: E402

OUT = os.path.join(HERE, "..", "out")
load = lambda *p: json.load(open(os.path.join(OUT, *p), encoding="utf-8"))


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    rep, ok = {}, True

    def gate(name, cond, detail):
        nonlocal ok
        rep[name] = dict(ok=bool(cond), detail=detail)
        ok &= bool(cond)
        print(("PASS " if cond else "FAIL ") + name + " - " + str(detail))

    gate("geometry_simple_polygons", self_check(get_forms()), "10 forms, H/V/H_cut/V_cut")
    gate("aero_selftest", aero.selftest(), "see lines above")

    res = load("results.json")
    bad = [f["id"] for f in res["forms"]
           if not (0.05 <= f["static_margin"] <= 0.15 and f["CL"] < aero.CL_MAX and f["m_ballast_g"] >= 0)]
    gate("forms_balance", not bad, f"static margin 5..15 %, CL < {aero.CL_MAX}; failing: {bad or 'none'}")
    rng = [f["range_sim_m"] / f["range_glide_m"] for f in res["forms"]]
    gate("sim_vs_polar_all_forms", max(abs(r - 1) for r in rng) <= 0.10, f"range sim/steady {min(rng):.3f}..{max(rng):.3f}")

    gate("3d_check", load("3d", "check.json")["ok"], "volumes = area x t, GLB extents")
    sh = load("sheets", "check.json")
    gate("sheets_check", sh["ok"], "2 closed cut outlines, slot 5.0, margins, page 297x210, ruler 50.0 mm")

    sc_path = os.path.join(OUT, "shots", "selfcheck.json")
    if os.path.exists(sc_path):
        js = {r["id"]: r for r in json.load(open(sc_path, encoding="utf-8"))}
        big = {f["id"]: f for f in load("results_x2.8.json")["forms"]}
        worst = {}

        def cmp(key, a, b):
            worst[key] = max(worst.get(key, 0.0), abs(a / b - 1))

        for f in res["forms"]:
            j, b = js[f["id"]], big[f["id"]]
            cmp("m", j["m"], f["m_total_g"]); cmp("V", j["V"], f["V_ms"]); cmp("LD", j["LD"], f["LD"])
            cmp("range", j["range"], f["range_sim_m"]); cmp("time", j["time"], f["time_sim_s"])
            cmp("m28", j["m28"], b["m_total_g"]); cmp("V28", j["V28"], b["V_ms"]); cmp("LD28", j["LD28"], b["LD"])
            cmp("range28", j["range28"], b["range_sim_m"]); cmp("bend28", j["bend28"], b["bending_use"])
        tol = dict(m=0.005, V=0.005, LD=0.005, range=0.02, time=0.02, m28=0.02, V28=0.02, LD28=0.02, range28=0.03, bend28=0.03)
        fails = {k: round(v, 4) for k, v in worst.items() if v > tol[k]}
        gate("page_js_vs_python", not fails, "worst relative differences " + json.dumps({k: round(v, 4) for k, v in worst.items()})
             + (f"; over tolerance: {fails}" if fails else ""))
    else:
        gate("page_js_vs_python", False, "out\\shots\\selfcheck.json missing - run page\\shot.mjs first")

    rep["ok"] = ok
    json.dump(rep, open(os.path.join(OUT, "check.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("VERIFY", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
