# -*- coding: utf-8 -*-
"""pdf_to_png.py - rasterise page 1 of a PDF for printing through print_png.ps1.
USAGE: python pdf_to_png.py <in.pdf> <out.png> <dpi> <gray|bw>      (FreeCAD python: pymupdf, PIL)
gray = 8-bit greyscale (office printers), bw = 1-bit line art (plotter; small in memory, crisp lines).
Prints the page size in mm - pass exactly these numbers to print_png.ps1 so the print is 1:1."""
import sys

import pymupdf
from PIL import Image

pdf, out, dpi, mode = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
page = pymupdf.open(pdf)[0]
pix = page.get_pixmap(dpi=dpi, colorspace=pymupdf.csGRAY, alpha=False)
img = Image.frombytes("L", (pix.width, pix.height), pix.samples)
if mode == "bw":
    img = img.point(lambda v: 255 if v > 170 else 0).convert("1")
img.save(out, dpi=(dpi, dpi), optimize=True)
mm = 25.4 / 72
print(f"png {pix.width}x{pix.height} px @ {dpi} dpi mode {mode}; page_mm {page.rect.width * mm:.2f} {page.rect.height * mm:.2f}")
