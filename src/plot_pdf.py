# -*- coding: utf-8 -*-
"""plot_pdf.py - plot a DXF (model space, millimetres) to a PDF page of an exact size: white background, black lines.
USAGE: python plot_pdf.py <file.dxf> [out.pdf] [--size=420x594]
The page shows model space x 0..W, y 0..H at 1:1. Needs ezdxf + matplotlib."""
import os
import sys

sys.stdout.reconfigure(encoding="utf-8")
import ezdxf

# ezdxf looks for fonts only in the system folders; add the per-user font folder (ISOCPEUR.ttf may live there)
USER_FONTS = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Microsoft", "Windows", "Fonts")
from ezdxf.fonts import fonts as xdxf_fonts

if os.path.isdir(USER_FONTS) and USER_FONTS not in ezdxf.options.support_dirs:
    ezdxf.options.support_dirs = list(ezdxf.options.support_dirs) + [USER_FONTS]
    xdxf_fonts.build_system_font_cache()
import matplotlib

matplotlib.use("Agg")
from matplotlib import font_manager

ISOCPEUR = os.path.join(USER_FONTS, "ISOCPEUR.ttf")
if os.path.exists(ISOCPEUR):
    font_manager.fontManager.addfont(ISOCPEUR)
import matplotlib.pyplot as plt
from ezdxf.addons.drawing import Frontend, RenderContext
from ezdxf.addons.drawing.config import BackgroundPolicy, ColorPolicy, Configuration, LineweightPolicy
from ezdxf.addons.drawing.matplotlib import MatplotlibBackend

args = [a for a in sys.argv[1:] if not a.startswith("--")]
F = args[0]
OUT = args[1] if len(args) > 1 else F.rsplit(".", 1)[0] + "_plot.pdf"
size = next((a.split("=")[-1] for a in sys.argv[1:] if a.startswith("--size")), "420x594")
W, H = (float(v) for v in size.replace("=", "x").split("x")[-2:])

doc = ezdxf.readfile(F)
msp = doc.modelspace()
THIN = 13
for container in [msp] + [b for b in doc.blocks if not b.name.startswith("*")]:
    for e in container:
        if e.dxftype() in ("HATCH", "DIMENSION", "MULTILEADER"):
            try:
                e.dxf.lineweight = THIN
            except Exception:
                pass

cfg = Configuration(background_policy=BackgroundPolicy.WHITE, color_policy=ColorPolicy.BLACK,
                    lineweight_policy=LineweightPolicy.ABSOLUTE, lineweight_scaling=0.5, min_lineweight=0.05)
fig = plt.figure(figsize=(W / 25.4, H / 25.4))
ax = fig.add_axes([0, 0, 1, 1])
Frontend(RenderContext(doc), MatplotlibBackend(ax), config=cfg).draw_layout(msp, finalize=True)
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.set_facecolor("white")
ax.set_aspect("equal", adjustable="box")
fig.set_size_inches(W / 25.4, H / 25.4, forward=True)
saved = None
base = OUT[:-4]
for suffix in ("", "_v2", "_v3"):       # the target may be locked by a PDF viewer
    try:
        cand = base + suffix + ".pdf"
        fig.savefig(cand, format="pdf", facecolor="white", metadata={"Creator": "plot_pdf.py", "Author": None, "Producer": None})
        saved = cand
        break
    except PermissionError:
        continue
plt.close(fig)
print("OK:" if saved else "FAIL: every candidate name is locked", saved or "")
