#!/usr/bin/env python3
"""OpenDRIVE(.xodr) 도로망 로더 — 참조선 샘플링 · 점→도로 투영 · 고도 · 차로 구조.

목적
  1) 임의 (x, y) 를 가장 가까운 도로의 (road, s, 횡오프셋 t) 로 투영한다.
     route.py 의 근사(기하 시작점만 비교)로는 긴 도로에서 수십 m 오차가 나서
     Ego 배치가 실패했다. 참조선을 1 m 간격으로 샘플링해 정확히 투영한다.
  2) 그 s 에서의 지형 고도(elevation 다항식)를 돌려준다.
  3) 경로계획을 위해 road 연결(link)·junction·차로(width) 정보를 보존한다.

지원 기하: line, arc, spiral(오일러 나선, 수치적분), poly3.
이 맵 실측: line 1527 / spiral 1246 / arc 656 / poly3 571, paramPoly3 없음.
"""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass
class Lane:
    id: int
    type: str
    widths: list[tuple[float, float, float, float, float]]   # (sOffset, a, b, c, d)
    marks: list[tuple[float, str]] = field(default_factory=list)   # (sOffset, roadMark type)

    def mark(self, ds: float) -> str:
        """이 차로의 **바깥쪽 경계선** 종류 (solid / broken / none). 차로변경 가부 판정용."""
        out = "none"
        for so, t in self.marks:
            if so <= ds:
                out = t
        return out

    def width(self, ds: float) -> float:
        w = 0.0
        for so, a, b, c, d in self.widths:
            if so <= ds:
                x = ds - so
                w = a + b * x + c * x * x + d * x ** 3
        return w


@dataclass
class LaneSection:
    s: float
    left: list[Lane]      # id > 0, 안쪽(1)부터 바깥쪽 순
    right: list[Lane]     # id < 0, 안쪽(-1)부터 바깥쪽 순

    def lane(self, lane_id: int) -> Lane | None:
        for ln in (self.left if lane_id > 0 else self.right):
            if ln.id == lane_id:
                return ln
        return None


@dataclass
class Road:
    id: str
    length: float
    junction: str                 # "-1" 이면 일반 도로
    pred: tuple[str, str, str] | None   # (elementType, elementId, contactPoint)
    succ: tuple[str, str, str] | None
    geoms: list[dict]
    elevations: list[tuple[float, float, float, float, float]]
    lane_offsets: list[tuple[float, float, float, float, float]]
    sections: list[LaneSection]
    objects: list[dict] = field(default_factory=list)   # {name, type, s, t, id}
    # 샘플링 결과 (s, x, y, hdg)
    S: np.ndarray = field(default_factory=lambda: np.zeros(0))
    X: np.ndarray = field(default_factory=lambda: np.zeros(0))
    Y: np.ndarray = field(default_factory=lambda: np.zeros(0))
    H: np.ndarray = field(default_factory=lambda: np.zeros(0))

    # ---- 고도 / 차로 오프셋 ----
    def elevation(self, s: float) -> float:
        z = 0.0
        for es, a, b, c, d in self.elevations:
            if es <= s:
                x = s - es
                z = a + b * x + c * x * x + d * x ** 3
        return z

    def lane_offset(self, s: float) -> float:
        off = 0.0
        for os_, a, b, c, d in self.lane_offsets:
            if os_ <= s:
                x = s - os_
                off = a + b * x + c * x * x + d * x ** 3
        return off

    def section_at(self, s: float) -> LaneSection | None:
        if not self.sections:
            return None
        sec = self.sections[0]
        for cand in self.sections:
            if cand.s <= s:
                sec = cand
        return sec

    def lane_center_t(self, s: float, lane_id: int) -> float:
        """lane_id 차로 중심의 횡오프셋 t (참조선 기준, 좌측 +)."""
        sec = self.section_at(s)
        if sec is None:
            raise KeyError(f"road {self.id}: laneSection 없음")
        ds = s - sec.s
        t = self.lane_offset(s)
        lanes = sec.left if lane_id > 0 else sec.right
        sign = 1.0 if lane_id > 0 else -1.0
        for ln in lanes:                       # 안쪽부터 바깥쪽으로 누적
            w = ln.width(ds)
            if ln.id == lane_id:
                return t + sign * w / 2.0
            t += sign * w
        raise KeyError(f"road {self.id} s={s:.1f}: lane {lane_id} 없음")

    def boundary_mark(self, s: float, lane_a: int, lane_b: int) -> str:
        """lane_a ↔ lane_b 사이 경계선 종류. 경계는 **안쪽(절대값이 작은) 차로**의 roadMark 다.
        방향이 다르면(중앙선) 'solid' 로 본다 — 넘으면 중앙선 침범이다."""
        if lane_a * lane_b <= 0:
            return "solid"
        sec = self.section_at(s)
        if sec is None:
            return "none"
        inner = lane_a if abs(lane_a) < abs(lane_b) else lane_b
        ln = sec.lane(inner)
        return ln.mark(s - sec.s) if ln is not None else "none"

    def driving_lanes(self, s: float) -> list[int]:
        sec = self.section_at(s)
        if sec is None:
            return []
        return [ln.id for ln in sec.left + sec.right if ln.type == "driving"]

    # ---- 참조선 → 월드 ----
    def pose_at(self, s: float, t: float = 0.0) -> tuple[float, float, float, float]:
        """(x, y, z, hdg). t 는 좌측 양(+) 횡오프셋."""
        s = min(max(s, 0.0), self.length)
        i = int(np.searchsorted(self.S, s))
        i = min(max(i, 1), len(self.S) - 1)
        s0, s1 = self.S[i - 1], self.S[i]
        w = 0.0 if s1 == s0 else (s - s0) / (s1 - s0)
        x = self.X[i - 1] + w * (self.X[i] - self.X[i - 1])
        y = self.Y[i - 1] + w * (self.Y[i] - self.Y[i - 1])
        h = self.H[i - 1] + w * _wrap(self.H[i] - self.H[i - 1])
        x += -math.sin(h) * t
        y += math.cos(h) * t
        return x, y, self.elevation(s), h


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------- 기하 샘플링
def _sample_geometry(g: dict, ds: float) -> list[tuple[float, float, float, float]]:
    """단일 geometry 를 ds 간격으로 (s, x, y, hdg) 샘플. 시작점 포함, 끝점 포함."""
    s0, x0, y0, h0, L = g["s"], g["x"], g["y"], g["hdg"], g["length"]
    n = max(2, int(math.ceil(L / ds)) + 1)
    out = []
    kind = g["type"]
    if kind == "line":
        for k in range(n):
            u = L * k / (n - 1)
            out.append((s0 + u, x0 + u * math.cos(h0), y0 + u * math.sin(h0), h0))
    elif kind == "arc":
        c = g["curvature"]
        for k in range(n):
            u = L * k / (n - 1)
            h = h0 + c * u
            if abs(c) < 1e-12:
                x, y = x0 + u * math.cos(h0), y0 + u * math.sin(h0)
            else:
                x = x0 + (math.sin(h) - math.sin(h0)) / c
                y = y0 - (math.cos(h) - math.cos(h0)) / c
            out.append((s0 + u, x, y, h))
    elif kind == "spiral":
        c0, c1 = g["curvStart"], g["curvEnd"]
        # 곡률이 s 에 선형: 작은 스텝으로 수치적분 (정확도 충분, 0.25 m)
        step = min(ds, 0.25)
        m = max(2, int(math.ceil(L / step)) + 1)
        x, y, h = x0, y0, h0
        pts = [(0.0, x, y, h)]
        for k in range(1, m):
            u_prev = L * (k - 1) / (m - 1)
            u = L * k / (m - 1)
            du = u - u_prev
            c_mid = c0 + (c1 - c0) * ((u_prev + u) / 2.0) / L
            h_mid = h + c_mid * du / 2.0
            x += du * math.cos(h_mid)
            y += du * math.sin(h_mid)
            h += c_mid * du
            pts.append((u, x, y, h))
        # ds 간격으로 솎아내기 (끝점 유지)
        want = [L * k / (n - 1) for k in range(n)]
        j = 0
        for wu in want:
            while j < len(pts) - 1 and pts[j + 1][0] <= wu + 1e-9:
                j += 1
            u, px, py, ph = pts[j]
            out.append((s0 + u, px, py, ph))
    elif kind == "poly3":
        a, b, c, d = g["a"], g["b"], g["c"], g["d"]
        # v = a + b u + c u^2 + d u^3 (국소 좌표: u 는 hdg 방향). 호장 L 만큼 u 를 전진.
        step = 0.25
        u, arc = 0.0, 0.0
        pts = []
        cos0, sin0 = math.cos(h0), math.sin(h0)
        def loc(uu):
            vv = a + b * uu + c * uu * uu + d * uu ** 3
            return x0 + uu * cos0 - vv * sin0, y0 + uu * sin0 + vv * cos0
        px, py = loc(0.0)
        pts.append((0.0, px, py))
        while arc < L - 1e-9:
            u += step
            nx, ny = loc(u)
            arc += math.hypot(nx - px, ny - py)
            px, py = nx, ny
            pts.append((min(arc, L), px, py))
        want = [L * k / (n - 1) for k in range(n)]
        j = 0
        for wu in want:
            while j < len(pts) - 1 and pts[j + 1][0] <= wu + 1e-9:
                j += 1
            su, px, py = pts[j]
            # 헤딩은 인접점 차분
            k2 = min(j + 1, len(pts) - 1)
            k1 = max(j - 1, 0)
            hh = math.atan2(pts[k2][2] - pts[k1][2], pts[k2][1] - pts[k1][1]) if k2 != k1 else h0
            out.append((s0 + su, px, py, hh))
    else:
        raise ValueError(f"미지원 기하 타입: {kind}")
    return out


# ---------------------------------------------------------------- 로더
class RoadNetwork:
    def __init__(self, xodr: str | Path, ds: float = 1.0):
        self.path = Path(xodr)
        self.ds = ds
        self.roads: dict[str, Road] = {}
        self.junctions: dict[str, list[dict]] = {}   # jid -> [ {incomingRoad, connectingRoad, contactPoint, laneLinks:[(from,to)]} ]
        self._parse()
        self._sample_all()
        self._build_index()

    # ---- 파싱 ----
    def _parse(self) -> None:
        root = ET.parse(self.path).getroot()
        for r in root.findall("road"):
            link = r.find("link")
            pred = succ = None
            if link is not None:
                p = link.find("predecessor")
                s_ = link.find("successor")
                if p is not None:
                    pred = (p.get("elementType"), p.get("elementId"), p.get("contactPoint", ""))
                if s_ is not None:
                    succ = (s_.get("elementType"), s_.get("elementId"), s_.get("contactPoint", ""))
            geoms = []
            for g in r.find("planView").findall("geometry"):
                d = {"s": float(g.get("s")), "x": float(g.get("x")), "y": float(g.get("y")),
                     "hdg": float(g.get("hdg")), "length": float(g.get("length"))}
                child = list(g)[0]
                d["type"] = child.tag
                for k, v in child.attrib.items():
                    d[k] = float(v)
                geoms.append(d)
            elev = []
            ep = r.find("elevationProfile")
            if ep is not None:
                for e in ep.findall("elevation"):
                    elev.append(tuple(float(e.get(k)) for k in ("s", "a", "b", "c", "d")))
            lanes_el = r.find("lanes")
            loffs = []
            sections = []
            if lanes_el is not None:
                for lo in lanes_el.findall("laneOffset"):
                    loffs.append(tuple(float(lo.get(k)) for k in ("s", "a", "b", "c", "d")))
                for sec in lanes_el.findall("laneSection"):
                    left, right = [], []
                    for side, bucket in (("left", left), ("right", right)):
                        se = sec.find(side)
                        if se is None:
                            continue
                        for ln in se.findall("lane"):
                            widths = [tuple(float(w.get(k)) for k in ("sOffset", "a", "b", "c", "d"))
                                      for w in ln.findall("width")]
                            # lane <link> 는 이 맵에서 비어 있다. 차로 연결은 planner 가
                            # 기하(차로 중심 거리)로 잇는다.
                            marks = sorted((float(m.get("sOffset", 0)), m.get("type", "none"))
                                           for m in ln.findall("roadMark"))
                            bucket.append(Lane(int(ln.get("id")), ln.get("type"), widths, marks))
                    left.sort(key=lambda l: l.id)            # 1, 2, 3 ...
                    right.sort(key=lambda l: -l.id)          # -1, -2, -3 ...
                    sections.append(LaneSection(float(sec.get("s")), left, right))
            objs = []
            oe = r.find("objects")
            if oe is not None:
                for o in oe.findall("object"):
                    try:
                        objs.append({"name": o.get("name", ""), "type": o.get("type", ""),
                                     "id": o.get("id", ""), "s": float(o.get("s", "0")),
                                     "t": float(o.get("t", "0"))})
                    except ValueError:
                        pass
            self.roads[r.get("id")] = Road(
                id=r.get("id"), length=float(r.get("length")), junction=r.get("junction", "-1"),
                pred=pred, succ=succ, geoms=geoms, elevations=elev, lane_offsets=loffs, sections=sections,
                objects=objs)

        for j in root.findall("junction"):
            conns = []
            for c in j.findall("connection"):
                conns.append({
                    "id": c.get("id"), "incomingRoad": c.get("incomingRoad"),
                    "connectingRoad": c.get("connectingRoad"), "contactPoint": c.get("contactPoint"),
                    "laneLinks": [(int(l.get("from")), int(l.get("to"))) for l in c.findall("laneLink")],
                })
            self.junctions[j.get("id")] = conns

    # ---- 샘플링 ----
    def _sample_all(self) -> None:
        for rd in self.roads.values():
            pts = []
            for g in rd.geoms:
                seg = _sample_geometry(g, self.ds)
                if pts and seg:
                    seg = seg[1:]                     # 이전 기하의 끝점과 중복 제거
                pts.extend(seg)
            if not pts:
                continue
            arr = np.array(pts, dtype=float)
            rd.S, rd.X, rd.Y, rd.H = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]

    def _build_index(self) -> None:
        ids, S, X, Y = [], [], [], []
        for rd in self.roads.values():
            n = len(rd.S)
            ids.extend([rd.id] * n)
            S.append(rd.S); X.append(rd.X); Y.append(rd.Y)
        self._idx_road = np.array(ids)
        self._idx_s = np.concatenate(S)
        self._idx_x = np.concatenate(X)
        self._idx_y = np.concatenate(Y)

    # ---- 투영 ----
    def project(self, x: float, y: float, max_dist: float = 25.0):
        """(x, y) → (road, s, t, dist). t 는 참조선 기준 좌측 양(+) 횡오프셋.
        dist 는 참조선까지의 거리(=|t|). max_dist 초과면 None."""
        d2 = (self._idx_x - x) ** 2 + (self._idx_y - y) ** 2
        i = int(np.argmin(d2))
        dist = math.sqrt(d2[i])
        if dist > max_dist:
            return None
        rd = self.roads[self._idx_road[i]]
        s = float(self._idx_s[i])
        _, _, _, h = rd.pose_at(s)
        # 횡오프셋 부호: 참조선 헤딩의 좌측이 양
        px, py, _, _ = rd.pose_at(s)
        t = -math.sin(h) * (x - px) + math.cos(h) * (y - py)
        return rd, s, t, dist

    def project_all(self, x: float, y: float, max_dist: float = 12.0):
        """max_dist 안의 모든 도로에 대해 (road, s, t, dist) — 도로별 최근접 1개씩, 가까운 순.
        교차로에서는 여러 연결도로가 겹치므로 후보를 전부 보고 골라야 한다."""
        d2 = (self._idx_x - x) ** 2 + (self._idx_y - y) ** 2
        mask = d2 <= max_dist * max_dist
        out = {}
        for i in np.nonzero(mask)[0]:
            rid = self._idx_road[i]
            if rid not in out or d2[i] < out[rid][0]:
                out[rid] = (d2[i], i)
        res = []
        for rid, (dd, i) in out.items():
            rd = self.roads[rid]
            s = float(self._idx_s[i])
            px, py, _, h = rd.pose_at(s)
            t = -math.sin(h) * (x - px) + math.cos(h) * (y - py)
            res.append((rd, s, t, math.sqrt(dd)))
        res.sort(key=lambda r: r[3])
        return res

    def stop_lines(self, road_id: str) -> list[float]:
        """해당 도로의 정지선 s 목록 (xodr object 'Rm_StopLine*'). 차로별 중복은 0.1 m 로 묶어 제거."""
        rd = self.roads[road_id]
        return sorted({round(o["s"], 1) for o in rd.objects if "StopLine" in o["name"]})

    def speed30_marks(self, road_id: str) -> list[tuple[float, float]]:
        """30 km/h 노면표시 객체 (s, t). 붉은 노면(보호구역) 좌표는 제공되지 않으므로 이 표식으로 근사한다."""
        rd = self.roads[road_id]
        return sorted((o["s"], o["t"]) for o in rd.objects if "speed_30" in o["name"])

    def limit_end_marks(self, road_id: str) -> list[float]:
        """제한 해제/50 표시 객체 s (RM_518, RM_517_50)."""
        rd = self.roads[road_id]
        return sorted(o["s"] for o in rd.objects if "RM_518" in o["name"] or "RM_517_50" in o["name"])

    def project_lane(self, x: float, y: float):
        """(x, y) 가 놓인 driving 차로까지 찾는다. → (road, s, lane_id, t) 또는 None."""
        hit = self.project(x, y)
        if hit is None:
            return None
        rd, s, t, _ = hit
        best = None
        sec = rd.section_at(s)
        if sec is None:
            return None
        for lid in rd.driving_lanes(s):
            try:
                tc = rd.lane_center_t(s, lid)
            except KeyError:
                continue
            w = sec.lane(lid).width(s - sec.s)
            err = abs(t - tc)
            if err <= w / 2.0 + 0.6 and (best is None or err < best[3]):
                best = (rd, s, lid, err)
        if best is None:
            return None
        return best[0], best[1], best[2], t

    def elevation_at(self, x: float, y: float) -> tuple[float, str, float] | None:
        hit = self.project(x, y)
        if hit is None:
            return None
        rd, s, t, dist = hit
        return rd.elevation(s), rd.id, dist


if __name__ == "__main__":
    import sys, time
    t0 = time.time()
    net = RoadNetwork(sys.argv[1] if len(sys.argv) > 1 else
                      str(Path(__file__).resolve().parent.parent / "data" / "HL_FMA_VTD_LivingLab.xodr"))
    print(f"로드 {time.time()-t0:.1f}s  road {len(net.roads)}  junction {len(net.junctions)}  샘플점 {len(net._idx_s)}")
    for name, (x, y) in {"Ego 원위치": (497.464, -173.709), "예제 wp1": (4.933, -24.564),
                         "사전2 wp1": (239.797, 145.955), "tltest 지점": (319.28, -44.47)}.items():
        r = net.project(x, y)
        if r is None:
            print(f"  {name:10s} 도로 없음"); continue
        rd, s, t, d = r
        pl = net.project_lane(x, y)
        lane = f"lane {pl[2]}" if pl else "차로 밖"
        print(f"  {name:10s} road {rd.id:>5} s={s:7.1f} t={t:+6.2f} dist={d:5.2f}  z={rd.elevation(s):6.2f}  {lane}  junction={rd.junction}")
