#!/usr/bin/env python3
"""도로망 + 계획 경로 + 경유지를 PNG 로 그린다 (외부 라이브러리 없이 PPM → ffmpeg).

사용: python3 plot_path.py path.csv route.csv out.png [여백m]
"""
from __future__ import annotations
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent.parent))

import csv
import math
import subprocess
import sys

import numpy as np

from hlfma.odr import RoadNetwork
from hlfma.planner import read_path_csv, load_waypoints

XODR = str(_pl.Path(__file__).resolve().parent.parent / "data" / "HL_FMA_VTD_LivingLab.xodr")


def draw_line(img, x0, y0, x1, y1, color, w=1):
    H, W = img.shape[:2]
    if (max(x0, x1) < 0 or min(x0, x1) >= W or max(y0, y1) < 0 or min(y0, y1) >= H):
        return                                            # 완전히 화면 밖
    n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
    for k in range(n + 1):
        t = k / max(n, 1)
        x = int(round(x0 + (x1 - x0) * t)); y = int(round(y0 + (y1 - y0) * t))
        # 음수 인덱스는 numpy 에서 뒤에서부터 세므로 반드시 클리핑한다
        ya, yb = max(0, y - w + 1), min(H, y + w)
        xa, xb = max(0, x - w + 1), min(W, x + w)
        if ya < yb and xa < xb:
            img[ya:yb, xa:xb] = color


def main():
    path_csv, route_csv, out = sys.argv[1], sys.argv[2], sys.argv[3]
    margin = float(sys.argv[4]) if len(sys.argv) > 4 else 60.0
    net = RoadNetwork(XODR)
    path = read_path_csv(path_csv)
    wps = load_waypoints(route_csv)
    xs = [p.x for p in path] + [w[0] for w in wps]
    ys = [p.y for p in path] + [w[1] for w in wps]
    xmin, xmax = min(xs) - margin, max(xs) + margin
    ymin, ymax = min(ys) - margin, max(ys) + margin
    W = 1600
    scale = W / (xmax - xmin)
    H = int((ymax - ymin) * scale) + 1
    img = np.full((H, W, 3), 245, dtype=np.uint8)

    def px(x, y):
        return (x - xmin) * scale, (ymax - y) * scale

    # 도로 참조선 (회색), 교차로 도로는 연한 파랑
    for rd in net.roads.values():
        if len(rd.X) < 2:
            continue
        if rd.X.max() < xmin or rd.X.min() > xmax or rd.Y.max() < ymin or rd.Y.min() > ymax:
            continue
        col = (170, 190, 230) if rd.junction != "-1" else (160, 160, 160)
        pts = [px(x, y) for x, y in zip(rd.X, rd.Y)]
        for (a, b), (c, d) in zip(pts, pts[1:]):
            draw_line(img, a, b, c, d, col, 1)
    # 경로 (진한 주황), 교차로 구간은 빨강
    pts = [(px(p.x, p.y), p.in_junction) for p in path]
    for ((a, b), j1), ((c, d), _) in zip(pts, pts[1:]):
        draw_line(img, a, b, c, d, (220, 40, 40) if j1 else (240, 130, 20), 2)
    # 경유지 (파란 네모 + 번호는 생략)
    for i, (x, y) in enumerate(wps):
        cx, cy = px(x, y)
        r = 5
        img[max(0, int(cy) - r):int(cy) + r, max(0, int(cx) - r):int(cx) + r] = (20, 60, 220)
    # 시작점 초록, 끝점 검정
    for (x, y), col in ((wps[0], (0, 160, 0)), (wps[-1], (0, 0, 0))):
        cx, cy = px(x, y); r = 7
        img[max(0, int(cy) - r):int(cy) + r, max(0, int(cx) - r):int(cx) + r] = col

    ppm = out + ".ppm"
    with open(ppm, "wb") as f:
        f.write(f"P6 {W} {H} 255\n".encode()); f.write(img.tobytes())
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", ppm, out], check=True)
    print(f"저장 {out} ({W}x{H}), 축척 {scale:.2f} px/m")


if __name__ == "__main__":
    main()
