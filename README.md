# XPS Glider — flat-foam gliders for an aerodynamics lesson

Ten hand-launch glider planforms cut from **5 mm XPS sheet underlay**, each made of **two flat parts** joined by a
half-lap slot (a horizontal part = wing + boom + tail, and a vertical part = fuselage + fin). The project contains the
geometry, the aerodynamic calculation, 1:1 cut templates, 3D models and an interactive page.

**Live page (Lithuanian / Russian):** https://xps-glider.pages.dev/ — 3D flight of the computed trajectory with a HUD,
live sliders (size, sheet mass, ballast, elevator, launch), a comparison of the ten forms. The page is one offline HTML
file: [`out/glider.html`](out/glider.html).

## What is inside

| Path | Content |
|---|---|
| `src/glider_geom.py` | **the only source of numbers**: 10 planforms, material, slots, fillets; `self_check()` |
| `src/aero.py` | mass and balance, vortex-lattice method, trim, drag polar, 3-DOF launch simulation; `--selftest`, `--scale k` |
| `src/make_sheets.py` | A4 cut templates at 1:1 (DXF + PDF) with nesting and a numeric check |
| `src/make_a1.py` | the chosen form (F07 "Delta") at the largest size on an A1 sheet, 1:1 |
| `src/make_3d.py` | build123d → STEP + GLB, volume / extent check |
| `src/verify.py` | one gate for everything → `out/check.json` |
| `page/` | the page (`src/`), its builder, the overview sheet, screenshots + JS-vs-Python comparison |
| `out/` | results: `forms_A4.pdf`, `sheets/*.dxf`, `F07_A1.pdf/.dxf`, `overview_A4.pdf`, `3d/*.step`, `3d/*.glb`, `results*.json` |

## Results (A4 mock-ups, 5 mm XPS, launch height 1.6 m)

| Form | Layout | Span, mm | Mass, g | Ballast, g | Speed, m/s | Glide ratio | Sink, m/s | Range, m | Time, s |
|---|---|---|---|---|---|---|---|---|---|
| F01 Klasika | conventional | 270 | 5.3 | 1.39 | 3.74 | 3.36 | 1.07 | 5.51 | 1.56 |
| F02 Trapecija | conventional | 270 | 5.2 | 1.33 | 3.72 | 3.34 | 1.07 | 5.49 | 1.57 |
| F03 Elipsė | conventional | 270 | 5.2 | 1.36 | 3.74 | 3.36 | 1.06 | 5.52 | 1.57 |
| F04 Strėlė | conventional | 270 | 4.6 | 0.70 | 3.48 | 3.28 | 1.01 | 5.36 | 1.65 |
| F05 Sklandytojas | conventional | 280 | 4.3 | 1.03 | 3.85 | 2.95 | 1.24 | 4.84 | 1.35 |
| F06 Platus sparnas | conventional | 190 | 5.6 | 1.75 | 3.95 | 3.28 | 1.15 | 5.41 | 1.46 |
| F07 Delta | tailless | 230 | 4.0 | 0.51 | 3.43 | 3.36 | 0.98 | 5.44 | 1.69 |
| F08 Antis | canard | 270 | 4.0 | 0.25 | 3.32 | 3.28 | 0.97 | 5.34 | 1.72 |
| F09 Tandemas | tandem | 250 | 5.3 | 1.24 | 3.15 | 3.18 | 0.95 | 5.17 | 1.76 |
| F10 Skraidantis sparnas | tailless | 270 | 3.2 | 0.25 | 2.88 | 3.34 | 0.83 | 5.38 | 1.99 |

At this size the shape matters less than the size and the balance: the sheet is always 5 mm thick, so on a small model
the blunt edges and skin friction dominate the drag. The same planform scaled to an A1 sheet glides about 1.5 times better
(F07 at 823 mm span: glide ratio ≈ 5.5, range ≈ 9 m).

## Method and honest limits

* Planar vortex-lattice method (horseshoe vortices, staircase strips) over the real outline → lift slope, neutral point,
  control power, pitch damping. Checked against a lifting-surface benchmark (rectangular wing AR 2), a tail-volume hand
  formula and grid convergence (`python src/aero.py --selftest`).
* Flat plate without leading-edge suction: lift-dependent drag = normal force tilted back by the local incidence.
  Zero-lift drag = laminar skin friction + drag of the 5 mm edges.
* Nose ballast sized for a 10 % static margin; trim by bending the trailing edge; 3-DOF simulation of the launch.
* The page re-implements the physics in JavaScript; `page/shot.mjs` + `src/verify.py` compare it with Python at two
  scales (agreement within 0.2 %).
* **Assumptions, not measurements:** foam density 33 kg/m³, edge drag coefficient 0.30, bending strength 0.25 MPa,
  flat-plate CLmax 0.7. Only the longitudinal motion is computed — roll and yaw behaviour must be found by throwing the
  mock-ups. Treat every number as an estimate; the real test is a throw in the gym.

## Build it

1. Print a template at **100 %** (no "fit to page") and check the control ruler on the sheet.
2. Fix the template on the foam and cut both parts along the solid line.
3. Slide the horizontal part into the fuselage slot from behind until the two slots lock; check the right angle; glue.
4. Add the nose ballast shown on the sheet — the glider must balance on the "SC" mark.
5. Bend the wing tips up about 15° and the control surface by the amount on the sheet.
6. Launch smoothly, slightly downwards, at gliding speed.

Trim card: dives → more up-elevator or less ballast; pitches up and stalls in waves → more ballast or less elevator;
turns → check that the fin is straight and both wing halves are equal.

## Rebuild from source

Python 3.11+ with `numpy`, `ezdxf`, `shapely`, `matplotlib`, `pymupdf`, `opencv-python` (sheets) and `build123d` (3D);
Node 22+ with `npm install` in `page/` (three, esbuild, qrcode); a Chromium with `--remote-debugging-port=9222` for the
screenshots and the PDF of the overview sheet.

```
python src/aero.py --selftest
python src/aero.py
python src/aero.py --scale 2.8 --out out/results_x2.8.json
python src/make_3d.py
python src/make_sheets.py
node   src/make_qr.mjs https://xps-glider.pages.dev/
python src/make_a1.py
node   page/build.mjs https://xps-glider.pages.dev/forms_A4.pdf F07
node   page/shot.mjs out/glider.html out/shots
node   page/overview.mjs F07 3.58
python src/verify.py
```

An author line on the printed sheets is optional: copy `author.example.json` to `author.local.json` (git-ignored).

## Sources

Videos that were studied are listed in [`SOURCES.md`](SOURCES.md).

## Licence

Code: MIT (see `LICENSE`). Drawings, 3D models and texts: CC BY 4.0. The page bundles three.js (MIT).
