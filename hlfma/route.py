"""계획 경로의 런타임 표현 — 누적거리, 곡률, 교차로/정지선, 속도 상한 구간, 종료점.

controller.py 가 매 프레임 참조한다. 모든 거리는 경로 누적거리(cum, m) 기준.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .config import Vehicle, Control
from .odr import RoadNetwork
from .planner import PathPoint


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Junction:
    entry: int            # 교차로 첫 경로 인덱스
    exit: int             # 교차로 밖 첫 인덱스
    road: str             # 접근 도로
    dir: int
    s_line: float | None  # 정지선 s (접근 도로 좌표)
    entry_cum: float
    stop_cum: float       # 후륜축 정지 목표 누적거리
    turn: int             # 0 직진 / 1 좌 / 2 우
    done: bool = False
    tl_latched: int = -1  # 접근 중 래치한 신호 상태 (-1 없음)
    tl_wait_s: float = 0.0
    stopped_s: float = 0.0


class Route:
    def __init__(self, path: list[PathPoint], net: RoadNetwork, veh: Vehicle, ctl: Control,
                 speed_zone30: float = 30 / 3.6):
        self.p = path
        self.n = len(path)
        self.net = net          # 회피 경로가 주행 차로 위인지 확인용
        self.veh, self.ctl = veh, ctl
        self.cum = [0.0]
        for a, b in zip(path, path[1:]):
            self.cum.append(self.cum[-1] + math.hypot(b.x - a.x, b.y - a.y))
        self.length = self.cum[-1]
        self.kappa = [0.0] * self.n
        for i in range(1, self.n - 1):
            ds = self.cum[i + 1] - self.cum[i - 1]
            if ds > 1e-3:
                self.kappa[i] = abs(wrap(path[i + 1].heading - path[i - 1].heading)) / ds
        # 도로별 인덱스 블록 (cum_at 용)
        self._blocks: dict[str, list[int]] = {}
        for i, q in enumerate(path):
            self._blocks.setdefault(q.road, []).append(i)
        self.junctions = self._find_junctions(net)
        self.stop_cums = self._find_stop_lines(net)
        self.caps = self._speed_caps(net, speed_zone30)
        self.finish = (path[-1].x, path[-1].y, path[-1].heading)

    # ---------------------------------------------------------- 기본 조회
    def nearest(self, x: float, y: float, start: int, window: int) -> int:
        lo, hi = max(0, start), min(self.n, max(start, 0) + window)
        best, bd = lo, float("inf")
        for k in range(lo, hi):
            d = (self.p[k].x - x) ** 2 + (self.p[k].y - y) ** 2
            if d < bd:
                bd, best = d, k
        return best

    def cum_xy(self, x: float, y: float, idx: int) -> float:
        """경로 위 연속 누적거리 (idx 점에서 접선방향으로 투영).

        cum[idx] 를 그대로 쓰면 경로 샘플 간격(STEP_M=2 m)이 그대로 종방향 해상도가 되어
        정지선을 2 m 넘어서고, 마지막 인덱스에서 포화해 종료 판정이 영영 서지 않는다.
        """
        p = self.p[idx]
        return self.cum[idx] + math.cos(p.heading) * (x - p.x) + math.sin(p.heading) * (y - p.y)

    def index_at_cum(self, c: float) -> int:
        lo, hi = 0, self.n - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.cum[mid] < c:
                lo = mid + 1
            else:
                hi = mid
        return lo

    def index_ahead(self, idx: int, dist: float) -> int:
        return min(self.n - 1, self.index_at_cum(self.cum[idx] + dist))

    def max_kappa(self, idx: int, dist: float) -> float:
        k2 = self.index_ahead(idx, dist)
        return max(self.kappa[idx:k2 + 1] or [0.0])

    def lateral_error(self, x: float, y: float, idx: int) -> float:
        """경로 기준 부호 있는 횡오차 (좌측 +)."""
        p = self.p[idx]
        return -math.sin(p.heading) * (x - p.x) + math.cos(p.heading) * (y - p.y)

    def cum_at(self, road: str, s: float) -> float | None:
        """도로 좌표 s → 경로 누적거리 (해당 도로 구간 안에서 보간/외삽)."""
        blk = self._blocks.get(road)
        if not blk:
            return None
        k = min(blk, key=lambda i: abs(self.p[i].s - s))
        # 진행 방향 부호: 블록 안에서 s 가 증가하면 +1
        sign = 1.0
        if len(blk) >= 2:
            sign = 1.0 if self.p[blk[-1]].s >= self.p[blk[0]].s else -1.0
        return self.cum[k] + (s - self.p[k].s) * sign

    def cap_at(self, c: float, default: float) -> float:
        v = default
        for a, b, cap in self.caps:
            if a <= c <= b:
                v = min(v, cap)
        return v

    # ---------------------------------------------------------- 구성
    def _find_junctions(self, net: RoadNetwork) -> list[Junction]:
        out = []
        back = self.veh.rear_to_front + self.ctl.stop_gap
        i = 1
        while i < self.n:
            if self.p[i].in_junction and not self.p[i - 1].in_junction:
                entry = i
                appr = self.p[i - 1]
                rd = net.roads[appr.road]
                d = +1 if (i >= 2 and self.p[i - 1].s >= self.p[i - 2].s) else -1
                lines = net.stop_lines(rd.id)
                s_line = None
                if lines:
                    if d == +1:
                        c = [s for s in lines if s <= rd.length + 0.5]
                        s_line = max(c) if c else None
                    else:
                        c = [s for s in lines if s >= -0.5]
                        s_line = min(c) if c else None
                entry_cum = self.cum[entry]
                stop_cum = None
                if s_line is not None:
                    cl = self.cum_at(rd.id, s_line)
                    if cl is not None and cl <= entry_cum + 3.0:
                        stop_cum = cl - back
                if stop_cum is None:
                    stop_cum = entry_cum - back
                j = i
                while j < self.n and self.p[j].in_junction:
                    j += 1
                dh = wrap(self.p[min(j, self.n - 1)].heading - self.p[i - 1].heading)
                tmin = math.radians(self.ctl.turn_min_deg)
                turn = 1 if dh > tmin else 2 if dh < -tmin else 0
                out.append(Junction(entry, j, rd.id, d, s_line, entry_cum, stop_cum, turn))
                i = j
            else:
                i += 1
        return out

    def _find_stop_lines(self, net: RoadNetwork) -> list[float]:
        """경로가 지나는 모든 정지선의 (후륜축 기준) 누적거리, 오름차순.

        계획기가 교차로로 잡지 못한 신호에도 서야 한다. VTD 는 접근 도로에서만
        신호를 주므로(PROTOCOL.md) tl_id != 0 자체가 "앞에 신호 교차로가 있다" 는
        뜻이다. _find_junctions 는 경로점의 in_junction 전이에만 의존해서,
        junction 요소를 지나지 않는 교차로는 통째로 놓친다.
        """
        back = self.veh.rear_to_front + self.ctl.stop_gap
        out = []
        for rid in {q.road for q in self.p}:
            for s_line in net.stop_lines(rid):
                c = self.cum_at(rid, s_line)
                if c is not None:
                    out.append(c - back)
        return sorted(out)

    def _speed_caps(self, net: RoadNetwork, v30: float) -> list[tuple[float, float, float]]:
        """30 km/h 근사: 우리 차로 측 roadmark_speed_30 표식부터 해제표시/다음 교차로까지.
        보호구역(붉은 노면) 좌표는 제공되지 않으므로 표식 위치로 근사한다. 저속은 감점이 아니다."""
        caps = []
        seen = set()
        for i, q in enumerate(self.p):
            if q.in_junction or q.road in seen:
                continue
            seen.add(q.road)
            blk = self._blocks[q.road]
            s_first, s_last = self.p[blk[0]].s, self.p[blk[-1]].s
            lo, hi = min(s_first, s_last), max(s_first, s_last)
            d = 1.0 if s_last >= s_first else -1.0
            ends = net.limit_end_marks(q.road)
            for s_m, t_m in net.speed30_marks(q.road):
                if not (lo - 5.0 <= s_m <= hi + 5.0):
                    continue
                if (t_m < 0) != (q.lane < 0):          # 반대편 차로 표식
                    continue
                c0 = self.cum_at(q.road, s_m)
                # 종료: 진행방향 앞의 해제표시, 없으면 다음 교차로 진입
                s_end = None
                cand = [s for s in ends if (s - s_m) * d > 0]
                if cand:
                    s_end = min(cand, key=lambda s: abs(s - s_m))
                c1 = self.cum_at(q.road, s_end) if s_end is not None else None
                if c1 is None:
                    nxt = [J.entry_cum for J in self.junctions if J.entry_cum > c0]
                    c1 = min(nxt) if nxt else self.length
                if c0 is not None and c1 > c0:
                    caps.append((c0, c1, v30))
        return caps

    def summary(self) -> str:
        js = ", ".join(f"{J.road}({'좌' if J.turn == 1 else '우' if J.turn == 2 else '직'}"
                       f"{'' if J.s_line is not None else ',정지선없음'})" for J in self.junctions)
        return (f"경로 {self.n}점 {self.length:.0f} m, 교차로 {len(self.junctions)}개 [{js}], "
                f"30km/h 근사구간 {len(self.caps)}개")
