#!/usr/bin/env python3
"""계획 경로가 입체교차(터널/고가) 구간을 지나는지 검사.

    python3 tools/check_overpass.py <route.csv>

제어기는 객체를 2D(x,y)로만 경로에 투영한다 — 위층/아래층 도로의 차·보행자가
'내 진로 위'로 잡힐 수 있다. 경로가 그런 구간을 지나면 알려준다. 주행 코드는 건드리지 않는다.
"""
import sys, math
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from hlfma.config import RunConfig
from hlfma.odr import RoadNetwork
from hlfma.planner import RoutePlanner, load_waypoints

DZ_MIN = 4.0      # 이 이상 고도가 다르면 다른 층
XY_MAX = 12.0     # 이 이내로 2D 겹치면 위/아래로 본다


def main(route_csv: str) -> int:
    if not Path(route_csv).is_file():
        print(f"경로 CSV 없음: {route_csv}\n사용: python3 tools/check_overpass.py <route.csv>")
        return 2
    cfg = RunConfig()
    net = RoadNetwork(cfg.xodr)
    path = RoutePlanner(net).plan(load_waypoints(route_csv), verbose=False)

    # 지도 전체 도로 중심선 샘플
    pts = []
    for rid, rd in net.roads.items():
        s = 0.0
        while s < rd.length:
            x, y, z, _ = rd.pose_at(s, 0.0)
            pts.append((x, y, z, rid))
            s += 4.0
    A = np.array([[p[0], p[1], p[2]] for p in pts])
    rids = [p[3] for p in pts]

    hits = []
    for k, p in enumerate(path):
        z = net.roads[p.road].elevation(p.s)
        d2 = (A[:, 0] - p.x) ** 2 + (A[:, 1] - p.y) ** 2
        for i in np.nonzero(d2 <= XY_MAX ** 2)[0]:
            if rids[i] == p.road:
                continue
            dz = A[i, 2] - z
            if abs(dz) >= DZ_MIN:
                hits.append((k, p.road, rids[i], dz, p.x, p.y))
                break

    if not hits:
        print("입체교차 없음 — 경로 전 구간이 단일 층. 객체 2D 투영 문제 없음.")
        return 0
    print(f"!! 입체교차 구간 {len(hits)}점 / 전체 {len(path)}점")
    seen = set()
    for k, r1, r2, dz, x, y in hits:
        key = (r1, r2)
        if key in seen:
            continue
        seen.add(key)
        where = "위" if dz > 0 else "아래"
        print(f"   경로 idx {k:4d} (road {r1}) {where}쪽 {abs(dz):.1f} m 에 road {r2}  @({x:.0f},{y:.0f})")
    print("→ 그 층의 차·보행자가 '내 진로 위'로 오인될 수 있다. 로그 notes 의 선행차/보행자 확인.")
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "route_day.csv"))
