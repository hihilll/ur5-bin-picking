"""生成手眼标定用 ArUco 码图片（Original ArUco 字典，配 aruco_ros 默认设置）。

用法:
  python scripts/gen_aruco_marker.py                 # id=26, 60mm, 300DPI
  python scripts/gen_aruco_marker.py --id 26 --size-mm 60

打印时选"实际大小/100% 缩放"，打完用卡尺量黑框边长，
量到的真实值(米)传给标定 launch 的 marker_size。
"""

import argparse
import os

import cv2
import numpy as np
from PIL import Image

MM_PER_INCH = 25.4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--id', type=int, default=26, help='marker id')
    ap.add_argument('--size-mm', type=float, default=60, help='黑框边长(mm)')
    ap.add_argument('--margin-mm', type=float, default=15, help='四周白色静区(mm)')
    ap.add_argument('--dpi', type=int, default=300)
    ap.add_argument('--out', default='', help='输出 PNG 路径')
    args = ap.parse_args()

    px = int(round(args.size_mm / MM_PER_INCH * args.dpi))
    margin = int(round(args.margin_mm / MM_PER_INCH * args.dpi))

    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    marker = cv2.aruco.generateImageMarker(d, args.id, px)

    canvas = np.full((px + 2 * margin, px + 2 * margin), 255, np.uint8)
    canvas[margin:margin + px, margin:margin + px] = marker

    out = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f'aruco_id{args.id}_{args.size_mm:g}mm.png')
    Image.fromarray(canvas).save(out, dpi=(args.dpi, args.dpi))
    print(f'已生成 {out}')
    print(f'  字典=Original ArUco  id={args.id}  黑框边长={args.size_mm}mm '
          f'(共 {px}px @ {args.dpi}DPI)')
    print('打印后务必用卡尺量黑框实际边长，作为 marker_size(米) 传给标定 launch。')


if __name__ == '__main__':
    main()
