#!/usr/bin/env python3
"""계획 경로의 차로변경이 실선(백색 실선/중앙선)을 넘는지 검사.

    python3 tools/check_solidline.py <route.csv>

대회 채점표에 '실선 차로변경 금지' 항목이 있는데(평가툴 화면 확인), 계획기는
roadMark 를 보지 않는다. 어디서 위반이 나는지만 알려준다 — 주행 코드는 안 건드린다.
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from collections import defaultdict

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from hlfma.config import RunConfig
from hlfma.odr import RoadNetwork
from hlfma.planner import RoutePlanner, load_waypoints


def load_marks(xodr: str):
    """road_id -> lane_id -> [(절대 s, type)] (laneSection s + roadMark sOffset)."""
    marks = defaultdict(lambda: defaultdict(list))
    for rd in ET.parse(xodr).getroot().findall("road"):
        rid = rd.get("id")
        for sec in rd.findall("lanes/laneSection"):
            s0 = float(sec.get("s"))
            for side in ("left", "right"):
                for ln in sec.findall(f"{side}/lane"):
                    lid = int(ln.get("id"))
                    for rm in ln.findall("roadMark"):
                        marks[rid][lid].append((s0 + float(rm.get("sOffset", 0)), rm.get("type", "none")))
        for lid in marks[rid]:
            marks[rid][lid].sort()
    return marks


def mark_at(marks, road: str, lane: int, s: float) -> str:
    seq = marks.get(road, {}).get(lane)
    if not seq:
        return "?"
    out = seq[0][1]
    for ms, t in seq:
        if ms <= s:
            out = t
    return out


def main() -> int:
    route_csv = sys.argv[1] if len(sys.argv) > 1 else "route_day.csv"
    if not Path(route_csv).is_file():
        print(f"경로 CSV 없음: {route_csv}\n사용: python3 tools/check_solidline.py <route.csv>")
        return 2
    cfg = RunConfig()
    net = RoadNetwork(cfg.xodr)
    marks = load_marks(cfg.xodr)
    path = RoutePlanner(net).plan(load_waypoints(route_csv), verbose=False)

    bad = ok = 0
    for i in range(1, len(path)):
        a, b = path[i - 1], path[i]
        if a.road != b.road or a.lane == b.lane or a.lane * b.lane <= 0:
            continue                      # 도로가 바뀌거나(연결로) 방향이 다르면 판정 제외
        inner = a.lane if abs(a.lane) < abs(b.lane) else b.lane   # 경계 = 안쪽 차로의 roadMark
        t = mark_at(marks, a.road, inner, b.s)
        if "solid" in t:
            bad += 1
            print(f"  !! 실선 위 차로변경: road {a.road} s={b.s:7.1f} lane {a.lane}→{b.lane}  roadMark={t}")
        else:
            ok += 1
            print(f"     점선 차로변경 OK : road {a.road} s={b.s:7.1f} lane {a.lane}→{b.lane}  roadMark={t}")
    print(f"\n차로변경 {ok + bad}회 중 실선 위반 {bad}회")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
