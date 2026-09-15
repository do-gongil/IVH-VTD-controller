#!/usr/bin/env python3
"""테스트 시나리오 생성 — 경로 CSV 기준으로 Ego 배치 + NPC 차량 + 횡단 보행자 주입.

    python3 tools/build_scenario.py <route.csv> [--npc 2] [--oncoming 1] [--ped 1] [--out 이름]

  1) scenario.apply_to_scenario 로 Ego 를 1번 경유지에 배치한 사본(_route.xml)을 만들고
  2) 계획 경로를 따라 NPC 차량(같은 차로 80/160 m 앞, Control=internal → ghostdriver 가 몬다),
     반대 차로 차량 1대, 경로 ~40% 지점에 도로를 횡단하는 보행자(Ego 40 m 접근 시 출발)를 넣는다.
  XML 구조는 VTD 샘플 PassingPedestrian.xml 을 템플릿으로 복사한다.
"""
from __future__ import annotations

import argparse
import copy
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from hlfma.config import DEFAULT_XODR
from hlfma.odr import RoadNetwork
from hlfma.planner import RoutePlanner, load_waypoints
import scenario as sc                                   # tools/scenario.py

SCEN_DIR = Path("/home/dap/Hexagon/VTD.2025.2/Data/Projects/Current/Scenarios")
BASE = SCEN_DIR / "HL_FMA_VTD_LivingLab.xml"
TEMPLATE = Path("/home/dap/Hexagon/VTD.2025.2/Data/Projects/SampleProject/Scenarios/PassingPedestrian.xml")


def _parent_map(root):
    return {c: p for p in root.iter() for c in p}


def _find_ego(root):
    for p in root.iter("Player"):
        d = p.find("Description")
        if d is not None and d.get("Name") == "Ego":
            return p
    raise SystemExit("Ego Player 없음")


def _write(root, src: Path, dst: Path):
    head = src.read_text(encoding="utf-8", errors="ignore")[:2048]
    prolog = head[:head.find("<Scenario")]
    dst.write_text(prolog + ET.tostring(root, encoding="unicode"), encoding="utf-8")


def add_vehicle(root, name, x, y, z, hdg, speed=0.0, ego=None):
    ego = ego or _find_ego(root)
    parent = _parent_map(root)[ego]
    npc = copy.deepcopy(ego)
    d = npc.find("Description"); d.set("Name", name); d.set("Control", "internal")
    pa = npc.find(".//PosAbsolute")
    for k, v in (("X", x), ("Y", y), ("Z", z), ("Direction", hdg)):
        pa.set(k, f"{v:.6e}")
    pa.set("AlignToRoad", "true")
    sp = npc.find(".//Speed")
    if sp is not None:
        sp.set("Value", f"{speed:.3f}")
    parent.insert(list(parent).index(ego) + 1, npc)


def add_pedestrian(root, tmpl_root, name, start, cross, trigger_xy, shape_id):
    """start=(x,y,z,hdg) 보도 위 출발점, cross=[(x,y,z)...] 횡단 경로, Ego 가 trigger_xy 40 m 안에 오면 출발."""
    tp = _parent_map(tmpl_root)
    t_char = next(tmpl_root.iter("Character"))
    t_shape = next(tmpl_root.iter("PathShape"))
    t_act = next(tmpl_root.iter("CharacterActions"))
    targets = {}
    for el, tag in ((t_char, "Character"), (t_shape, "PathShape"), (t_act, "CharacterActions")):
        ptag = tp[el].tag
        dst_parent = root.find(f".//{ptag}") if ptag != "Scenario" else root
        if dst_parent is None:
            dst_parent = ET.SubElement(root, ptag)
        targets[tag] = dst_parent

    ch = copy.deepcopy(t_char); ch.set("Name", name)
    sp = ch.find("StartPosAbs")
    for k, v in zip(("X", "Y", "Z", "Direction"), start):
        sp.set(k, f"{v:.6e}")
    targets["Character"].append(ch)

    shape = copy.deepcopy(t_shape); shape.set("ShapeId", str(shape_id)); shape.set("Name", f"Cross{shape_id}")
    for w in list(shape):
        shape.remove(w)
    for (x, y, z) in cross:
        ET.SubElement(shape, "Waypoint", {"X": f"{x:.6e}", "Y": f"{y:.6e}", "Z": f"{z:.6e}", "Options": "0x00000000"})
    targets["PathShape"].append(shape)

    act = copy.deepcopy(t_act); act.set("Character", name)
    action = act.find("Action")
    for extra in list(act)[1:]:
        act.remove(extra)
    # 트리거를 Ego 접근으로 교체
    for child in list(action):
        if child.tag in ("PosRelative", "PosAbsolute", "EditorPos"):
            action.remove(child)
    trig = ET.Element("PosAbsolute", {"CounterID": "", "CounterComp": "COMP_EQ", "Radius": "40.0",
                                      "X": f"{trigger_xy[0]:.6e}", "Y": f"{trigger_xy[1]:.6e}",
                                      "NetDist": "false", "CounterVal": "0", "Pivot": "Ego"})
    action.insert(0, trig)
    for child in action:
        if child.tag == "CharacterPath":
            child.set("PathShape", str(shape_id))
        if child.tag == "Motion":
            child.set("Speed", "1.4")
    targets["CharacterActions"].append(act)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("route")
    ap.add_argument("--npc", type=int, default=2)
    ap.add_argument("--oncoming", type=int, default=1)
    ap.add_argument("--ped", type=int, default=1)
    ap.add_argument("--base", default=str(BASE))
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    base = Path(a.base)
    wps = load_waypoints(a.route)
    sc.apply_to_scenario(wps, str(base))                     # → <base>_route.xml (Ego 배치)
    src = base.with_name(f"{base.stem}_route{base.suffix}")
    root = ET.parse(src).getroot()
    tmpl = ET.parse(TEMPLATE).getroot()

    net = RoadNetwork(DEFAULT_XODR)
    path = RoutePlanner(net).plan(wps, verbose=False)
    cum = [0.0]
    for p, q in zip(path, path[1:]):
        cum.append(cum[-1] + math.hypot(q.x - p.x, q.y - p.y))

    def at(dist):
        k = min(range(len(cum)), key=lambda i: abs(cum[i] - dist))
        return path[k]

    n_veh = 0
    for m, dist in enumerate((80.0, 160.0, 240.0)[:a.npc]):
        p = at(dist)
        rd = net.roads[p.road]
        add_vehicle(root, f"Npc{m+1}", p.x, p.y, rd.elevation(p.s), p.heading)
        n_veh += 1
        print(f"  Npc{m+1}: 경로 {dist:.0f} m 지점 road {p.road} lane {p.lane}")
    for m in range(a.oncoming):
        p = at(120.0 + 100.0 * m)
        rd = net.roads[p.road]
        opp = [l for l in rd.driving_lanes(p.s) if (l < 0) != (p.lane < 0)]
        if not opp:
            print("  반대 차로 없음 — 건너뜀"); continue
        lane = min(opp, key=abs)
        x, y, z, h = rd.pose_at(p.s, rd.lane_center_t(p.s, lane))
        h = h + math.pi if lane > 0 else h
        add_vehicle(root, f"Oncoming{m+1}", x, y, z, h)
        n_veh += 1
        print(f"  Oncoming{m+1}: road {p.road} lane {lane}")
    for m in range(a.ped):
        p = at(cum[-1] * (0.4 + 0.25 * m))
        rd = net.roads[p.road]
        # 참조선 기준 t: 오른쪽 보도(-) 에서 왼쪽(+) 으로 횡단
        sign = -1.0 if p.lane < 0 else 1.0
        t_edge = 0.0
        for l in rd.driving_lanes(p.s):
            sec = rd.section_at(p.s)
            t_edge = max(t_edge, abs(rd.lane_center_t(p.s, l)) + sec.lane(l).width(p.s - sec.s) / 2)
        s_ped = p.s + (12.0 if p.lane < 0 else -12.0)          # Ego 진행방향 12 m 앞에서 횡단
        s_ped = max(0.5, min(rd.length - 0.5, s_ped))
        x0, y0, z0, h0 = rd.pose_at(s_ped, sign * (t_edge + 2.0))
        x1, y1, z1, _ = rd.pose_at(s_ped, -sign * (t_edge + 2.0))
        xm, ym, zm, _ = rd.pose_at(s_ped, 0.0)
        add_pedestrian(root, tmpl, f"Ped{m+1}", (x0, y0, z0, math.atan2(y1 - y0, x1 - x0)),
                       [(x0, y0, z0), (xm, ym, zm), (x1, y1, z1)], (p.x, p.y), 900 + m)
        print(f"  Ped{m+1}: road {p.road} s={s_ped:.1f} 횡단, 트리거 경로 {cum[-1]*(0.4+0.25*m):.0f} m 지점")

    out = Path(a.out) if a.out else base.with_name(f"{base.stem}_{Path(a.route).stem}_test.xml")
    _write(root, base, out)
    print(f"저장: {out}  (차량 {n_veh}, 보행자 {a.ped})")


if __name__ == "__main__":
    main()
