#!/usr/bin/env python3
"""경로 CSV 분석 및 시나리오 시작점 정합.

대회는 경유지 (x, y) 목록을 CSV 로 준다. 좌표계는 시나리오/OpenDRIVE 의
절대 좌표계와 동일하다 (실측 확인: Ego 초기 위치가 시나리오 XML 의
PosAbsolute 와 패킷 offset 0/4/8 에서 정확히 일치).

문제: 배포된 시나리오의 Ego 시작점이 경로 1번 경유지와 다를 수 있다.
      route_example.csv 의 경우 514.6 m 떨어져 있어 그대로 돌리면
      제어기가 엉뚱한 방향으로 최대 조향을 건다.

해결: --apply 로 Ego 시작 자세를 1번 경유지에 맞춘 시나리오 사본을 만든다.
      원본은 건드리지 않는다 (교육 지침: 원본 보존, Save As 로 v1, v2...).

사용:
    python3 route.py route.csv                    # 경로 분석만
    python3 route.py route.csv --apply 시나리오.xml  # 시작점 맞춘 사본 생성
"""
from __future__ import annotations
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent.parent))

import argparse
import csv
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

TURN_THRESHOLD = math.radians(20)   # 이 이상 꺾이면 회전 구간으로 본다


def load_route(path: str) -> list[tuple[float, float]]:
    pts = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            pts.append((float(row["x"]), float(row["y"])))
    if len(pts) < 2:
        sys.exit("경유지가 2개 미만이다.")
    return pts


def start_pose(pts: list[tuple[float, float]]) -> tuple[float, float, float]:
    """1번 경유지 위치 + 2번을 향하는 방위."""
    (x0, y0), (x1, y1) = pts[0], pts[1]
    return x0, y0, math.atan2(y1 - y0, x1 - x0)


def analyze(pts: list[tuple[float, float]]) -> None:
    print(f"{'seq':>4} {'x':>10} {'y':>10} | {'구간':>9} {'방위':>9}  회전")
    total, prev_h = 0.0, None
    for i, (x, y) in enumerate(pts):
        if i == 0:
            print(f"{i+1:>4} {x:10.3f} {y:10.3f} |     시작")
            continue
        px, py = pts[i - 1]
        d = math.hypot(x - px, y - py)
        total += d
        h = math.atan2(y - py, x - px)
        turn = ""
        if prev_h is not None:
            t = (h - prev_h + math.pi) % (2 * math.pi) - math.pi
            if abs(t) > TURN_THRESHOLD:
                turn = f"{math.degrees(t):+6.1f}° {'좌' if t > 0 else '우'}회전"
        print(f"{i+1:>4} {x:10.3f} {y:10.3f} | {d:8.1f}m {math.degrees(h):+8.1f}°  {turn}")
        prev_h = h

    x0, y0, h0 = start_pose(pts)
    print(f"\n총 주행거리 {total:.1f} m, 경유지 {len(pts)}개")
    print(f"권장 Ego 시작 자세: X={x0:.3f} Y={y0:.3f} Direction={h0:.5f} rad ({math.degrees(h0):.1f}°)")



def find_xodr(src: Path, root) -> Path | None:
    """시나리오 <Layout File=...> 은 $VTD_ROOT/Data 기준 상대경로. 못 찾으면 패키지 동봉 xodr."""
    layout = root.find("Layout")
    rel = layout.get("File") if layout is not None else None
    if rel:
        bases = [src.parent]
        for parent in src.parents:
            if (parent / "Data").is_dir() and (parent / "Runtime").is_dir():
                bases += [parent / "Data", parent]
                break
        for base in bases:
            cand = (base / rel).resolve()
            if cand.is_file():
                return cand
    from hlfma.config import DEFAULT_XODR
    return DEFAULT_XODR if Path(DEFAULT_XODR).is_file() else None

def apply_to_scenario(pts: list[tuple[float, float]], scenario: str) -> None:
    src = Path(scenario)
    if not src.is_file():
        sys.exit(f"시나리오를 찾을 수 없다: {src}")

    tree = ET.parse(src)
    root = tree.getroot()

    # Name="Ego" 인 Player 의 PosAbsolute 를 찾는다
    target = None
    for player in root.iter("Player"):
        desc = player.find("Description")
        if desc is not None and desc.get("Name") == "Ego":
            target = player.find(".//PosAbsolute")
            break
    if target is None:
        sys.exit("Ego 의 PosAbsolute 를 찾지 못했다. 시나리오에 Ego 가 있는지 확인할 것.")

    old = (target.get("X"), target.get("Y"), target.get("Z"), target.get("Direction"))
    x0, y0, h0 = start_pose(pts)

    # Z 를 원본 값 그대로 두면 안 된다. 원본의 Z 는 원래 시작 지점의 지형
    # 고도이고, 새 위치의 지형과 수 m 어긋나면 차량이 공중/지하에 스폰되어
    # 배치가 실패한다. 시나리오가 참조하는 xodr 에 정확히 투영해 계산한다.
    xodr = find_xodr(src, root)
    if xodr is None:
        sys.exit("Layout File(.xodr) 을 찾지 못했다. 시나리오의 <Layout File=...> 을 확인할 것.")
    from hlfma.odr import RoadNetwork
    net = RoadNetwork(xodr)
    hit = net.project(x0, y0)
    if hit is None:
        sys.exit(f"1번 경유지 ({x0:.1f},{y0:.1f}) 근처 25 m 안에 도로가 없다.")
    rd, s_on, t_on, dist = hit
    z0, rid = rd.elevation(s_on), rd.id
    lane_hit = net.project_lane(x0, y0)
    if rd.junction != "-1":
        print(f"  주의: 1번 경유지가 교차로 내부 도로(road {rid}, junction {rd.junction}) 위다. "
              f"공지대로 경유지는 대략적 위치이므로 차량이 정상 배치되는지 확인할 것.")
    elif lane_hit is None:
        print(f"  주의: 1번 경유지가 주행 차로 폭 밖(참조선에서 {t_on:+.2f} m)이다.")

    # 경유지는 대략적 위치라 차로 밖·교차로 안에 찍히기도 한다. 그대로 배치하면
    # Ego 스폰이 실패해 시뮬레이터가 프레임을 진행시키지 못한다 (InitDone 타임아웃).
    # 계획 경로의 첫 점은 정의상 차로 중심이고 진행방향도 풀려 있으므로 그것을 쓴다.
    from hlfma.planner import RoutePlanner, PlanError
    try:
        p0 = RoutePlanner(net).plan(pts, verbose=False)[0]
        z0 = net.roads[p0.road].elevation(p0.s)
        print(f"  Ego 시작을 계획 경로 첫 점으로 보정: road {p0.road} lane {p0.lane} "
              f"({x0:.1f},{y0:.1f}) → ({p0.x:.1f},{p0.y:.1f})")
        x0, y0, h0 = p0.x, p0.y, p0.heading
    except (PlanError, ValueError) as e:
        print(f"  경로계획 실패({e}) → 경유지 원좌표로 배치한다")

    target.set("X", f"{x0:.16e}")
    target.set("Y", f"{y0:.16e}")
    target.set("Z", f"{z0:.16e}")
    target.set("Direction", f"{h0:.16e}")
    target.set("AlignToRoad", "true")     # 도로 기울기는 VTD 가 맞춘다

    # 원본 보존: <이름>_route.xml 로 저장
    dst = src.with_name(f"{src.stem}_route{src.suffix}")
    if dst.exists():
        shutil.copy2(dst, dst.with_suffix(dst.suffix + ".bak"))

    # ElementTree 는 DOCTYPE 을 버린다. VTD 파서는 이걸 요구하므로
    # 원본의 프롤로그(루트 시작 태그 앞부분)를 그대로 복원해야 한다.
    # 없으면 LoadScenario 가 조용히 실패하고 VtGui 가
    # "set scenario file before starting the simulation" 을 낸다.
    head = src.read_text(encoding="utf-8", errors="ignore")[:2048]
    cut = head.find("<Scenario")
    prolog = head[:cut] if cut > 0 else '<?xml version="1.0" encoding="UTF-8"?>\n<!DOCTYPE Scenario>\n'

    body = ET.tostring(root, encoding="unicode")
    dst.write_text(prolog + body, encoding="utf-8")

    print(f"원본     : {src}")
    print(f"사본 생성: {dst}")
    print(f"  X         {float(old[0]):.4f}  ->  {x0:.4f}")
    print(f"  Y         {float(old[1]):.4f}  ->  {y0:.4f}")
    print(f"  Z         {float(old[2]):.4f}  ->  {z0:.4f}   (road {rid}, 최근접 {dist:.1f} m)")
    print(f"  Direction {float(old[3]):.5f}  ->  {h0:.5f} rad")
    print("\nVtGui 에서 이 사본을 열고 Configure → Apply → 재생 할 것.")


def main() -> None:
    ap = argparse.ArgumentParser(description="경로 CSV 분석 및 시나리오 시작점 정합")
    ap.add_argument("route", help="경유지 CSV (헤더: seq,x,y)")
    ap.add_argument("--apply", metavar="시나리오.xml",
                    help="Ego 시작 자세를 1번 경유지에 맞춘 사본을 만든다")
    args = ap.parse_args()

    pts = load_route(args.route)
    analyze(pts)
    if args.apply:
        print()
        apply_to_scenario(pts, args.apply)


if __name__ == "__main__":
    main()
