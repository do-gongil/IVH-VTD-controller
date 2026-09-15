#!/usr/bin/env python3
"""정적 장애물(구루마) + 라바콘 종료선 시험 시나리오 생성.

    python3 tools/make_obstacle_scenario.py <경로.csv> [출력.xml]

기준 시나리오에 Object 를 덧붙이기만 한다 (NPC 는 건드리지 않는다).
Type="other" 로 넣어야 RDB objects[] 에 실린다 — Type="sign" 은 안 실린다(실측).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from hlfma.config import RunConfig
from hlfma.odr import RoadNetwork
from hlfma.planner import RoutePlanner, load_waypoints
from hlfma.route import Route
from tools.make_scenario import e, lat_point, zof, SCEN_DIR, BASE

# (이름, Definition, 경로 cum[m], 횡오프셋[m])
# 대회 영상(fma.mp4 t=138~150 s) 재현: 터널(편도 2차로) 직후 넓어진 구간에 구루마가
# 차로를 옮겨가며 놓여 있다. 1 번을 피해 들어간 차로에 2 번이 있어 연속 회피가 필요하다.
OBJECTS = [
    ("Barrow1", "WheelBarrow01", 590.0,  0.0),   # 내 차로 한복판
    ("Barrow2", "WheelBarrow01", 630.0, -3.3),   # 우측 차로 = 1 번을 피한 자리
    ("Barrow3", "WheelBarrow01", 672.0, -6.6),   # 그 다음 차로
]
# 종료선 라바콘: 마지막 경유지 기준 뒤로 back[m], 좌우 ±half[m]
FINISH = ("RdMiscPylon03-32cm", 3.0, 1.75)       # 편도 1차로(왕복 2차로) 폭 3.5 m


def move_ego(src: str, p0) -> str:
    """기준 시나리오의 Ego 출발 위치를 이 경로의 시작점으로 옮긴다.
    안 옮기면 다른 경로를 시험할 때 자차가 수백 m 떨어진 곳에서 출발해 재계획이 걸린다."""
    import re
    m = re.search(r'(Name="Ego"[^>]*/>\s*<Init>\s*<Speed[^>]*/>\s*<PosAbsolute )([^/]*)(/>)', src, re.S)
    if not m:
        raise SystemExit("Ego PosAbsolute 를 찾지 못했다")
    new_pos = (f'X="{e(p0.x)}" Y="{e(p0.y)}" Z="{e(0.0)}" '
               f'Direction="{e(p0.heading)}" AlignToRoad="true" ')
    return src[:m.start(2)] + new_pos + src[m.end(2):]


def obj_xml(name, definition, x, y, z, hdg):
    return (f'        <Object Type="other" Name="{name}" Definition="{definition}">\n'
            f'            <StartPosAbs X="{e(x)}" Y="{e(y)}" Z="{e(z)}" '
            f'Direction="{e(hdg)}" Pitch="{e(0.0)}" Roll="{e(0.0)}"/>\n'
            f'        </Object>\n')


def main():
    route_csv = sys.argv[1]
    out_name = sys.argv[2] if len(sys.argv) > 2 else "HL_FMA_obstacle_test.xml"
    cfg = RunConfig()
    net = RoadNetwork(cfg.xodr)
    path = RoutePlanner(net).plan(load_waypoints(route_csv), verbose=False)
    r = Route(path, net, cfg.vehicle, cfg.control, cfg.speed_zone30)

    blocks, names = [], []
    for name, definition, cum, lat in OBJECTS:
        k = min(range(r.n), key=lambda i: abs(r.cum[i] - cum))
        p = r.p[k]
        x, y = lat_point(p, lat)
        blocks.append(obj_xml(name, definition, x, y, zof(net, x, y, 0.0), p.heading))
        names.append((name,))
        print(f"  {name:10s} {definition:20s} cum {r.cum[k]:6.1f} lat {lat:+.1f} @({x:.1f},{y:.1f})")

    definition, back, half = FINISH
    cum_f = r.length - back
    k = min(range(r.n), key=lambda i: abs(r.cum[i] - cum_f))
    p = r.p[k]
    for side, nm in ((+half, "FinishL"), (-half, "FinishR")):
        x, y = lat_point(p, side)
        blocks.append(obj_xml(nm, definition, x, y, zof(net, x, y, 0.0), p.heading))
        names.append((nm,))
        print(f"  {nm:10s} {definition:20s} cum {r.cum[k]:6.1f} lat {side:+.2f} @({x:.1f},{y:.1f})")
    print(f"  → 종료선 콘 간격 {2*half:.1f} m (find_cone_line 허용 2~8 m)")

    # Object 는 <MovingObjectsControl> 안에 있어야 한다. <TrafficControl> 에 넣으면
    # XML 은 유효하지만 VTD 가 조용히 무시한다 (실측: RDB objects[] 에 안 실린다).
    src = move_ego(BASE.read_text(encoding="utf-8"), r.p[0])
    acts = "".join(f'        <ObjectActions Object="{b[0]}"/>\n' for b in names)
    close = "    </MovingObjectsControl>"
    assert src.count(close) == 1, "MovingObjectsControl 을 찾지 못했다"
    src = src.replace(close, "".join(blocks) + acts + close, 1)
    out = SCEN_DIR / out_name
    out.write_text(src, encoding="utf-8")
    print(f"\n생성: {out}\n실행: python3 tools/simctl.py load {out_name}")


if __name__ == "__main__":
    main()
