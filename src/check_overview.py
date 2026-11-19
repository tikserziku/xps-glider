# -*- coding: utf-8 -*-
"""check_overview.py - numeric check of out\\overview_A4.pdf -> out\\overview_check.json, PNG preview.
Checks: 1 page, 210 x 297 mm, required strings present in the text layer, all 10 form ids, QR decodes to the site url,
no Cyrillic on the sheet (Lithuanian only).   USAGE: python check_overview.py   (FreeCAD python: pymupdf, cv2, numpy)"""
import json
import os
import re
import sys

import cv2
import numpy as np
import pymupdf

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "out")
sys.stdout.reconfigure(encoding="utf-8")
qr = json.load(open(os.path.join(OUT, "qr.json"), encoding="utf-8"))
doc = pymupdf.open(os.path.join(OUT, "overview_A4.pdf"))
page = doc[0]
mm = 25.4 / 72
text = page.get_text()
rep = dict(pages=len(doc), page_mm=[round(page.rect.width * mm, 2), round(page.rect.height * mm, 2)])
sys.path.insert(0, HERE)
from author import author  # noqa: E402

need = [s for s in author() if s] + [qr["url"], "PASIRINKTA"] + [f"F{i:02d}" for i in range(1, 11)]
rep["missing"] = [s for s in need if s not in text]
rep["cyrillic_chars"] = len(re.findall(r"[Ѐ-ӿ]", text))
pix = page.get_pixmap(dpi=300)
img = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width, pix.n)[:, :, :3].copy()
val, pts, _ = cv2.QRCodeDetector().detectAndDecode(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
rep["qr_decoded"] = val
if pts is not None and len(pts):
    p = pts.reshape(-1, 2)
    rep["qr_size_mm"] = round(float(p[:, 0].max() - p[:, 0].min()) / 300 * 25.4, 1)
page.get_pixmap(dpi=110).save(os.path.join(OUT, "overview_A4.png"))
rep["ok"] = (rep["pages"] == 1 and abs(rep["page_mm"][0] - 210) < 0.3 and abs(rep["page_mm"][1] - 297) < 0.3
             and not rep["missing"] and rep["cyrillic_chars"] == 0 and val == qr["url"])
json.dump(rep, open(os.path.join(OUT, "overview_check.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
print(json.dumps(rep, ensure_ascii=False))
