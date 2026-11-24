#!/usr/bin/env python3
"""Print-ready PDF of the ArUco grid board in tewatashi_bringup/config/handeye_board.yaml.

  python3 tools/make_board_pdf.py [-o handeye_board.pdf]

A4 landscape, true scale. Print at 100 % ("actual size"), glue flat onto a
rigid board, then measure a marker and update marker_size / separation.
Needs OpenCV >= 4.7 with aruco (pip install opencv-contrib-python) and Pillow.
"""
import argparse
import pathlib

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw

A4_MM = (297.0, 210.0)  # landscape
PX_PER_MM = 20          # 508 dpi


def main():
    here = pathlib.Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--config", default=here / "tewatashi_bringup/config/handeye_board.yaml")
    ap.add_argument("-o", "--output", default="handeye_board.pdf")
    args = ap.parse_args()
    b = yaml.safe_load(open(args.config))[0]

    mx, my = b["markers_x"], b["markers_y"]
    m_mm, s_mm = b["marker_size"] * 1000, b["separation"] * 1000
    w_mm = mx * m_mm + (mx - 1) * s_mm
    h_mm = my * m_mm + (my - 1) * s_mm
    if w_mm > A4_MM[0] - 20 or h_mm > A4_MM[1] - 20:
        raise SystemExit(f"board {w_mm:.0f}x{h_mm:.0f} mm does not fit on A4 with 10 mm margins")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    ids = np.arange(b["first_id"], b["first_id"] + mx * my, dtype=np.int32)
    board = cv2.aruco.GridBoard((mx, my), b["marker_size"], b["separation"], dictionary, ids)
    board_px = (round(w_mm * PX_PER_MM), round(h_mm * PX_PER_MM))
    img = board.generateImage(board_px, marginSize=0, borderBits=1)

    page = Image.new("L", (round(A4_MM[0] * PX_PER_MM), round(A4_MM[1] * PX_PER_MM)), 255)
    ox = (page.width - board_px[0]) // 2
    oy = (page.height - board_px[1]) // 2
    page.paste(Image.fromarray(img), (ox, oy))
    draw = ImageDraw.Draw(page)
    draw.text((10 * PX_PER_MM, page.height - 8 * PX_PER_MM),
              f"tewatashi hand-eye board  4X4_50 ids {ids[0]}-{ids[-1]}  {mx}x{my}  "
              f"marker {m_mm:.1f} mm  gap {s_mm:.1f} mm  print at 100 % and measure",
              fill=0)
    # 100 mm scale bar to check the print scale
    y = page.height - 14 * PX_PER_MM
    draw.rectangle([10 * PX_PER_MM, y, 110 * PX_PER_MM, y + PX_PER_MM], fill=0)
    draw.text((10 * PX_PER_MM, y - 5 * PX_PER_MM), "100 mm", fill=0)
    page.save(args.output, resolution=PX_PER_MM * 25.4)
    print(f"wrote {args.output}: board {w_mm:.1f} x {h_mm:.1f} mm on A4 landscape")


if __name__ == "__main__":
    main()
