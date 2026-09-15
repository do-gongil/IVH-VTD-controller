#!/usr/bin/env python3
"""경유지(CSV) → OpenDRIVE 도로망을 따르는 조밀한 주행 경로.

대회 경로 형식 (Redmine 뉴스 [22], [30]):
  - 첫 지점 = 출발, 마지막 = 종료, 그 사이는 (교차로 진입, 교차로 진출) 쌍.
  - 경유지는 대략적 위치이며 반드시 밟을 필요 없다. 차로값도 아니다.
  - 경로는 최단거리 기준으로 선정되었다.

방법
  1) 경유지를 도로에 투영하되, 교차로 안에서는 연결도로가 여러 개 겹치므로
     후보를 전부 보고 "교차로 밖 도로 우선 + 다음 경유지 방향과 일치하는 진행방향"으로 고른다.
  2) 도로 그래프(road link + junction connection)에서 경유지 순서대로 Dijkstra.
     진행방향 판정이 틀렸을 때를 대비해 방향을 뒤집어 재시도한다.
  3) 차로 중심선을 2 m 간격으로 샘플링. 교차로 통과는 connectingRoad + laneLink 로 결정.

출력 점: (x, y, heading, road_id, lane_id, s, in_junction)
"""
from __future__ import annotations

import csv
import heapq
import math
import sys
from dataclasses import dataclass

from .odr import RoadNetwork, Road

STEP_M = 2.0
# 실선 위 차로변경을 피해 블렌드를 앞당긴다. 긴급 차단용 스위치.
AVOID_SOLID_LANE_CHANGE = True

# 교차로 진입 차로 보정(2패스). 회귀 비교·긴급 차단용 스위치.
ENTRY_LANE_FIX = __import__('os').environ.get('HLFMA_ENTRY_FIX', '1') != '0'
DETOUR_WARN = 1.6      # 직선거리 대비 이 배수를 넘으면 우회 경고


@dataclass
class PathPoint:
    x: float
    y: float
    heading: float
    road: str
    lane: int
    s: float
    in_junction: bool


@dataclass
class Loc:
    road: Road
    s: float
    lane: int
    dir: int          # +1: s 증가 방향(우측 차로 id<0), -1: 반대
    t: float
    dist: float


# ---------------------------------------------------------------- 그래프
class RoadGraph:
    def __init__(self, net: RoadNetwork):
        self.net = net

    @staticmethod
    def _dir_from_contact(contact: str) -> int:
        return +1 if contact == "start" else -1

    def successors(self, node: tuple[str, int]):
        rid, d = node
        rd = self.net.roads[rid]
        link = rd.succ if d == +1 else rd.pred
        if link is None:
            return
        etype, eid, contact = link
        if etype == "road":
            nxt = self.net.roads.get(eid)
            if nxt is not None:
                yield (eid, self._dir_from_contact(contact)), nxt.length, None
        elif etype == "junction":
            for c in self.net.junctions.get(eid, []):
                if c["incomingRoad"] != rid:
                    continue
                cr = self.net.roads.get(c["connectingRoad"])
                if cr is None:
                    continue
                yield (c["connectingRoad"], self._dir_from_contact(c["contactPoint"])), cr.length, c

    def shortest(self, start, goal):
        if start == goal:
            return [(start, None)], 0.0
        dist = {start: 0.0}
        prev: dict = {}
        pq = [(0.0, start)]
        seen = set()
        while pq:
            dcur, node = heapq.heappop(pq)
            if node in seen:
                continue
            seen.add(node)
            if node == goal:
                break
            for nxt, w, conn in self.successors(node):
                nd = dcur + w
                if nd < dist.get(nxt, float("inf")):
                    dist[nxt] = nd
                    prev[nxt] = (node, conn)
                    heapq.heappush(pq, (nd, nxt))
        if goal not in prev:
            return None, float("inf")
        out, n = [], goal
        while n != start:
            p, conn = prev[n]
            out.append((n, conn))
            n = p
        out.append((start, None))
        out.reverse()
        return out, dist[goal]


# ---------------------------------------------------------------- 차로 도우미
def _lanes_for_dir(rd: Road, s: float, d: int) -> list[int]:
    return sorted([i for i in rd.driving_lanes(s) if (i < 0) == (d == +1)], key=abs)


def _nearest_lane(cands: list[int], want: int | None) -> int | None:
    if not cands:
        return None
    if want is None:
        return cands[0]
    return min(cands, key=lambda i: (abs(abs(i) - abs(want)), abs(i)))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------- 계획기
class RoutePlanner:
    def __init__(self, net: RoadNetwork):
        self.net = net
        self.g = RoadGraph(net)

    # ---- 경유지 투영 ----
    def locate(self, x: float, y: float, hint: float | None, prefer_non_junction: bool = True) -> list[Loc]:
        """후보 Loc 목록 (좋은 순). hint = 원하는 진행방위[rad] (다음 경유지 방향)."""
        cands = self.net.project_all(x, y, max_dist=14.0)
        if not cands:
            # 광폭도로(주행차로 10개)의 바깥 차로는 참조선에서 최대 18 m 떨어져 있다.
            # 14 m 안에 후보가 없다고 계획 전체를 실패시키지 말고 반경을 넓혀 재시도한다.
            cands = self.net.project_all(x, y, max_dist=25.0)
        if not cands:
            raise ValueError(f"({x:.1f},{y:.1f}) 근처 25 m 에 도로 없음")
        # 교차로 진입·진출점은 경계에 찍혀 안쪽 연결도로(여러 개가 겹침)에 투영되기 쉽다.
        # 교차로 밖 도로가 12 m 안에 있으면 그것만 쓴다 → 어느 연결도로를 탈지는
        # Dijkstra 가 접근로→진출로 최단으로 정한다.
        if prefer_non_junction:
            nonj = [c for c in cands if c[0].junction == "-1" and c[3] <= 18.0 and c[0].sections]
            if nonj:
                cands = nonj
        locs = []
        for rd, s, t, dist in cands:
            _, _, _, h = rd.pose_at(s)
            for d in (+1, -1):
                lanes = _lanes_for_dir(rd, s, d)
                if not lanes:
                    continue
                hd = h if d == +1 else _wrap(h + math.pi)
                align = 1.0 if hint is None else math.cos(_wrap(hd - hint))
                if hint is not None and align < 0.0:          # 반대방향은 버린다
                    continue
                lane = min(lanes, key=lambda i: abs(rd.lane_center_t(s, i) - t))
                score = dist + (6.0 if (prefer_non_junction and rd.junction != "-1") else 0.0) \
                        + (1.0 - align) * 8.0
                locs.append((score, Loc(rd, s, lane, d, t, dist)))
        if not locs:
            if hint is None:
                # 힌트가 이미 없는데도 비었다면 재시도해도 비어 있다 (무한 재귀 방지).
                raise ValueError(f"({x:.1f},{y:.1f}) 근처에 주행차로 없음")
            # 방향 힌트를 무시하고 재시도
            return self.locate(x, y, None, prefer_non_junction)
        locs.sort(key=lambda z: z[0])
        return [l for _, l in locs]

    def plan(self, waypoints: list[tuple[float, float]], verbose: bool = True) -> list[PathPoint]:
        n = len(waypoints)
        hints = []
        for i, (x, y) in enumerate(waypoints):
            if i < n - 1:
                nx, ny = waypoints[i + 1]
            else:
                nx, ny = x, y
                x, y = waypoints[i - 1]
            hints.append(math.atan2(ny - y, nx - x))
        cand_lists = [self.locate(x, y, hints[i]) for i, (x, y) in enumerate(waypoints)]

        cur = cand_lists[0][0]
        cur_lane = cur.lane
        self.locs = [cur]
        chains: list[list] = []

        def segment(frm: Loc, frm_lane: int, to: Loc):
            chain, _ = self.g.shortest((frm.road.id, frm.dir), (to.road.id, to.dir))
            if chain is None:
                return None
            if len(chain) == 1 and ((to.s - frm.s) * frm.dir < -1.0):
                return None                                     # 같은 도로에서 뒤로 가는 후보
            try:
                seg = self._sample_chain(chain, frm.s, to.s, frm_lane, quiet=True)
            except (RuntimeError, KeyError):
                return None
            return path_length(seg), chain, seg

        for i in range(n - 1):
            best = None
            straight = math.hypot(waypoints[i + 1][0] - waypoints[i][0], waypoints[i + 1][1] - waypoints[i][1])
            # 다음 경유지 후보마다 실제 경로를 샘플링해 길이를 재고, 그 다음 구간까지
            # 한 단계 내다본 합이 가장 짧은 후보를 택한다 (교차로 안 경유지가 엉뚱한
            # 연결도로에 붙어 다음 구간이 크게 우회하는 것을 막는다).
            for nxt in cand_lists[i + 1][:6]:
                r = segment(cur, cur_lane, nxt)
                if r is None:
                    continue
                length, chain, seg = r
                look = 0.0
                if i + 2 < n:
                    look_best = None
                    for nn in cand_lists[i + 2][:4]:
                        r2 = segment(nxt, seg[-1].lane, nn)
                        if r2 is not None and (look_best is None or r2[0] < look_best):
                            look_best = r2[0]
                    look = look_best if look_best is not None else 1e6
                total = length + look
                if best is None or total < best[0]:
                    best = (total, length, chain, nxt, seg)
            if best is None:
                raise PlanError(f"경유지 {i+1}→{i+2}: 경로 없음 (road {cur.road.id} → 후보 "
                                f"{[c.road.id for c in cand_lists[i+1][:6]]})")
            _, length, chain, nxt, seg = best
            if verbose:
                flag = "  ← 우회 의심" if length > DETOUR_WARN * straight + 30 else ""
                print(f"  구간 {i+1}→{i+2}: 직선 {straight:6.1f} m  도로경로 {length:6.1f} m  "
                      f"{cur.road.id}→{nxt.road.id} ({len(chain)}개 도로){flag}")
            chains.append(chain)
            cur_lane = seg[-1].lane if seg else nxt.lane        # 후보 평가용 차로 연속
            cur = nxt
            self.locs.append(cur)

        # ---- 전역 체인 병합 후 한 번에 샘플링 ----
        # 경유지(교차로 진입점)가 도로 끝에 찍히면 구간 단위 샘플링에서는 접근로가
        # "다음이 교차로"임을 알 수 없어 사전 차로변경이 빠진다(예제 경로 6 m 점프의 원인).
        # 구간 경계에서 같은 도로가 이어지면 하나로 합친다.
        merged: list = []
        for ch in chains:
            for node, conn in ch:
                if merged and merged[-1][0] == node:
                    continue
                merged.append((node, conn))
        self.chain = merged
        path = self._sample_chain(merged, self.locs[0].s, self.locs[-1].s, self.locs[0].lane)
        return finalize_path(path, self.net)

    @staticmethod
    def _entry_lanes(rd: Road, s_end: float, cands_end: list[int], conn: dict,
                     nrd: Road, ns0: float) -> list[int]:
        """rd 끝에서 교차로 연결도로 nrd 로 들어갈 수 있는 rd 의 차로 목록.

        laneLink 의 from 차로 중 기하학적으로 실제 이어지는 것만. 하나도 안 맞으면
        (맵의 링크 기하 불일치) 번호만 보고 고른다.
        """
        good = []
        for f, t in conn["laneLinks"]:
            if f not in cands_end:
                continue
            gap = _lane_gap(rd, s_end, f, nrd, ns0, t)
            if gap is not None and gap < 0.5:
                good.append(f)
        if not good:
            good = [f for f, _ in conn["laneLinks"] if f in cands_end]
        return good

    # ---- 샘플링 ----
    def _sample_chain(self, chain, s_start, s_end, lane_start, lane_goal=None, quiet: bool = False) -> list[PathPoint]:
        """도로 연쇄를 차로 중심선으로 샘플링.

        차로 연속성: 이 맵은 lane <link> 가 비어 있어 번호로 이어 붙이면 회전 포켓 차로 등에서
        수 m 횡 점프가 생긴다. 직전 점을 새 도로에 투영해 기하학적으로 가장 가까운 차로를 잇는다.
        교차로 진입: 다음 connection 의 laneLink 'from' 차로로 접근로 마지막 40 m 에서
        부드럽게(smoothstep) 차로변경한다. 경유지의 차로는 공지대로 강제하지 않는다.
        """
        pts: list[PathPoint] = []
        lane = lane_start
        last_xy = None
        for k, ((rid, d), conn) in enumerate(chain):
            rd = self.net.roads[rid]
            first, last = (k == 0), (k == len(chain) - 1)
            if d == +1:
                sa = s_start if first else 0.0
                sb = s_end if last else rd.length
            else:
                sa = s_start if first else rd.length
                sb = s_end if last else 0.0
            cands = _lanes_for_dir(rd, (sa + sb) / 2, d) or _lanes_for_dir(rd, sa, d)
            if not cands:
                raise PlanError(f"road {rid}: 진행방향 {d} 에 주행차로 없음")

            # ---- 이 도로에서 탈 차로 ----
            if conn is not None and k > 0:                         # 교차로 연결도로: laneLink
                prd, pd = self.net.roads[chain[k - 1][0][0]], chain[k - 1][0][1]
                s_prev_end = prd.length if pd == +1 else 0.0
                # 접근로 끝 차로 f ↔ 연결도로 시작 차로 t 가 기하학적으로 이어지는 링크만
                good = []
                for f, t in conn["laneLinks"]:
                    gap = _lane_gap(prd, s_prev_end, f, rd, sa, t)
                    if gap is not None and gap < 0.5:
                        good.append((f, t, gap))
                pick = [t for f, t, _ in good if f == lane]
                if not pick and good:
                    pick = [min(good, key=lambda g: abs(abs(g[0]) - abs(lane)))[1]]
                if not pick:                                       # 링크가 안 맞으면 기하로 직접
                    pick = [min(cands, key=lambda t: _lane_gap(prd, s_prev_end, lane, rd, sa, t) or 1e9)]
                lane = pick[0] if pick[0] in cands else _nearest_lane(cands, lane)
            elif not first and last_xy is not None:                # 일반 도로: 기하 연속
                lane = _lane_by_xy(rd, sa, cands, last_xy)
            if lane not in cands:
                lane = _nearest_lane(cands, lane)

            # ---- 다음이 교차로 진입이면 그 connection 의 from 차로로 미리 변경 ----
            target = lane
            entry_forced = False          # 교차로가 요구한 차로인가 (번호 비교로 판단하면 안 된다)
            if k + 1 < len(chain) and chain[k + 1][1] is not None:
                nconn = chain[k + 1][1]
                nrd = self.net.roads[chain[k + 1][0][0]]
                ns0 = 0.0 if chain[k + 1][0][1] == +1 else nrd.length
                cands_end = _lanes_for_dir(rd, sb, d) or cands
                good = self._entry_lanes(rd, sb, cands_end, nconn, nrd, ns0)
                if good:
                    target = min(good, key=lambda f: (abs(abs(f) - abs(lane)), abs(f)))
                    entry_forced = True
            elif k + 1 < len(chain):
                # ---- 일반 도로 연결: 현 차로가 다음 도로로 이어지지 않으면 미리 합류 ----
                # 차로 수가 줄어드는 구간(예: 교차로 연결도로 3차로 → 일반도로 2차로)에서
                # 번호만 보고 이어붙이면 경계에서 1~2 m 횡 점프가 생기고, finalize_path 의
                # 이음새 재보간이 그걸 펴면서 주행영역 밖으로 부푼다. 이어지는 차로로
                # 블렌드해 정상적인 차로변경으로 만든다.
                nrd, nd = self.net.roads[chain[k + 1][0][0]], chain[k + 1][0][1]
                ns0 = 0.0 if nd == +1 else nrd.length
                ncands = _lanes_for_dir(nrd, ns0, nd)
                cands_end = _lanes_for_dir(rd, sb, d) or cands

                def _gap_to_next(f: int) -> float:
                    return min((_lane_gap(rd, sb, f, nrd, ns0, t) or 1e9) for t in ncands)

                if ncands and cands_end and _gap_to_next(lane) > 0.5:
                    good = [f for f in cands_end if _gap_to_next(f) < 0.5]
                    if good:
                        target = min(good, key=lambda f: (abs(abs(f) - abs(lane)), abs(f)))

                # ---- 2 단계 선행: 다음 도로 뒤가 교차로면 그 교차로 차로로 지금부터 ----
                # 접근로가 짧으면(route_example 첫 교차로: 52 m) 거기서만 바꾸기엔 늦다.
                # 다음 도로가 교차로에서 요구하는 from 차로를 먼저 구하고, 그 차로로
                # 기하학적으로 이어지는 현 도로의 차로를 target 으로 잡아 미리 붙는다.
                if k + 2 < len(chain) and chain[k + 2][1] is not None and ncands and cands_end:
                    ns1 = nrd.length if nd == +1 else 0.0          # 다음 도로의 끝(교차로 쪽)
                    rd2 = self.net.roads[chain[k + 2][0][0]]
                    s2 = 0.0 if chain[k + 2][0][1] == +1 else rd2.length
                    ncands_end = _lanes_for_dir(nrd, ns1, nd) or ncands
                    need = self._entry_lanes(nrd, ns1, ncands_end, chain[k + 2][1], rd2, s2)
                    if need:
                        # 현 도로 끝에서 need 중 하나로 곧장 이어지는 차로들
                        feed = [f for f in cands_end
                                if any((_lane_gap(rd, sb, f, nrd, ns0, t) or 1e9) < 0.5 for t in need)]
                        if feed and target not in feed:
                            target = min(feed, key=lambda f: (abs(abs(f) - abs(lane)), abs(f)))
            span = abs(sb - sa)
            n_change = abs(abs(target) - abs(lane))
            blend = 0.0
            if target != lane:
                # 교차로 진입 차로변경은 **일찍** 끝내야 한다. 45 km/h 에서 40 m 는
                # 3.2 s 뿐이라 뒷차 흐름을 끊는다. 아래 avail 로 포켓 차로가 실제로
                # 존재하는 구간까지만 제한되므로 넉넉히 잡아도 안전하다.
                blend = min(max(70.0, 45.0 * n_change), 150.0, span * 0.9)
                # 포켓 차로(예: 우회전 −4)는 마지막 laneSection 에만 있을 수 있다.
                # target 차로가 존재하는 구간 길이로 블렌드를 제한한다.
                avail = 0.0
                for sec in rd.sections:
                    s_lo, s_hi = sec.s, rd.length
                    nxt_secs = [q.s for q in rd.sections if q.s > sec.s]
                    if nxt_secs:
                        s_hi = min(nxt_secs)
                    if sec.lane(target) is not None and sec.lane(target).type == "driving":
                        lo, hi = max(s_lo, min(sa, sb)), min(s_hi, max(sa, sb))
                        # 도로 끝(교차로 쪽)에 붙은 구간만 센다
                        end_s = sb
                        if (d == +1 and abs(hi - end_s) < 1e-6) or (d == -1 and abs(lo - end_s) < 1e-6):
                            avail = max(avail, hi - lo)
                        elif avail > 0.0:
                            avail += max(0.0, hi - lo)
                if avail > 0.0:
                    blend = min(blend, avail * 0.95)
                if blend < 15.0 * n_change and not quiet:
                    print(f"  경고: road {rid} 에서 {n_change}차로 변경 여유 {blend:.0f} m 부족")

            def _blend_len(nc: int) -> float:
                """nc 차로 변경에 쓸 블렌드 길이. target 차로가 실제 존재하는 구간까지만."""
                b = min(max(70.0, 45.0 * nc), 150.0, span * 0.9)
                av = 0.0
                for sec in rd.sections:
                    s_lo, s_hi = sec.s, rd.length
                    nxt = [q.s for q in rd.sections if q.s > sec.s]
                    if nxt:
                        s_hi = min(nxt)
                    if sec.lane(target) is not None and sec.lane(target).type == "driving":
                        lo, hi = max(s_lo, min(sa, sb)), min(s_hi, max(sa, sb))
                        end_s = sb
                        if (d == +1 and abs(hi - end_s) < 1e-6) or (d == -1 and abs(lo - end_s) < 1e-6):
                            av = max(av, hi - lo)
                        elif av > 0.0:
                            av += max(0.0, hi - lo)
                return min(b, av * 0.95) if av > 0.0 else b

            # ---- 샘플링 ----
            # 곡률이 큰 구간은 촘촘히 샘플링한다. MAX_TURN_DEG 는 '스텝당' 한계라
            # STEP_M 고정으로는 급커브(교차로 연결로)에서 스텝당 회전이 한계를 넘어
            # finalize_path 가 스스로 만든 경로를 거부한다.
            _, _, _, _h0 = rd.pose_at(sa)
            _, _, _, _h1 = rd.pose_at(sb)
            _need = int(math.ceil(abs(_wrap(_h1 - _h0)) / math.radians(MAX_TURN_DEG * 0.6))) + 1
            npts = max(2, int(span / STEP_M) + 1, int(math.ceil(span / (MAX_GAP - 0.2))) + 1, _need)

            def _sample(blend_len: float):
                """이 도로를 blend_len 만큼 target 으로 블렌드하며 샘플링. (점들, 끝 차로)"""
                out: list[PathPoint] = []
                lane_l = lane
                sec_prev = None
                t_prev = None
                t_adj, adj_left = 0.0, 0.0        # 구간 경계 횡 어긋남을 15 m 에 걸쳐 흡수
                bs = None                         # 블렌드가 실제로 시작된 s
                for j in range(npts):
                    sv = sa + (sb - sa) * j / (npts - 1)
                    sec = rd.section_at(sv)
                    here = _lanes_for_dir(rd, sv, d)
                    if sec is not sec_prev and sec_prev is not None and t_prev is not None:
                        # laneSection 경계에서 차로 번호가 밀린다. 번호를 유지하면 물리적으로
                        # 옆 차선으로 끌려가므로 **직전 횡위치에 가장 가까운 차로**로 다시 잡는다.
                        if here and (bs is None or lane_l not in here):
                            lane_l = min(here, key=lambda i: abs(rd.lane_center_t(sv, i) - t_prev))
                        if here and lane_l in here and bs is None:
                            jump = t_prev - rd.lane_center_t(sv, lane_l)
                            if abs(jump) > 0.3:
                                t_adj, adj_left = jump, 15.0
                    sec_prev = sec
                    t0 = rd.lane_center_t(sv, lane_l)
                    t, lane_here = t0, lane_l
                    if blend_len > 0.0 and target in here and abs(sb - sv) < blend_len:
                        if bs is None:
                            bs = sv                # target 차로가 존재하는 첫 지점부터
                        frac = abs(sv - bs) / max(abs(sb - bs), 1e-6)
                        w = frac * frac * (3.0 - 2.0 * frac)          # smoothstep
                        t1 = rd.lane_center_t(sv, target)
                        t = t0 + (t1 - t0) * w
                        if w > 0.5:
                            lane_here = target
                    if adj_left > 0.0:
                        t += t_adj * (adj_left / 15.0)
                        adj_left = max(0.0, adj_left - STEP_M)
                    x, y, _, hh = rd.pose_at(sv, t)
                    if d == -1:
                        hh = _wrap(hh + math.pi)
                    out.append(PathPoint(x, y, hh, rid, lane_here, sv, rd.junction != "-1"))
                    t_prev = t
                return out, (out[-1].lane if out else lane_l)

            seg, lane_end = _sample(blend)
            # 교차로가 진입 차로를 지정했는데 실제로 그 차로로 끝나지 않았다면 다시 그린다.
            # 차로 번호는 laneSection 마다 밀리므로(road 429: 진입부 [1,2,3] → 교차로쪽
            # [2,3,4,5,6]) 도로 진입 시점의 번호로 blend 를 판단하면 빗나간다. 번호를
            # 예측하지 않고 **실제 샘플 결과**를 보고 한 번만 다시 그린다.
            if (ENTRY_LANE_FIX and entry_forced and lane_end != target
                    and target in (_lanes_for_dir(rd, sb, d) or [])):
                nc = max(1, abs(abs(target) - abs(lane_end)))
                b2 = _blend_len(nc)
                if b2 > 5.0:
                    seg2, lane_end2 = _sample(b2)
                    if lane_end2 == target:
                        if not quiet:
                            print(f"  교차로 진입 차로 보정: road {rid} 끝차로 {lane_end}→{target} "
                                  f"({b2:.0f} m 에 걸쳐)")
                        seg, lane_end, blend = seg2, lane_end2, b2

            # 실선 위 차로변경 회피 (채점 항목). 차로가 바뀌는 지점의 경계선이 실선이면
            # 블렌드를 늘려 **더 일찍** 옮긴다 — 실선은 보통 교차로 쪽에만 깔려 있다.
            # 늘려도 안 되면 원래 것을 쓴다(차로 자체는 맞춰야 회전을 한다).
            def _solid_cross(sq) -> bool:
                for a, b in zip(sq, sq[1:]):
                    if a.lane != b.lane and a.road == b.road:
                        if "solid" in rd.boundary_mark(b.s, a.lane, b.lane):
                            return True
                return False

            if AVOID_SOLID_LANE_CHANGE and blend > 0.0 and _solid_cross(seg):
                cap = min(span * 0.95, 220.0)
                for mult in (1.4, 1.8, 2.4):
                    b3 = min(blend * mult, cap)
                    if b3 <= blend + 1.0:
                        break
                    seg3, lane_end3 = _sample(b3)
                    if lane_end3 == lane_end and not _solid_cross(seg3):
                        if not quiet:
                            print(f"  실선 회피: road {rid} 차로변경을 {blend:.0f}→{b3:.0f} m 로 앞당김")
                        seg, blend = seg3, b3
                        break
            pts.extend(seg)
            if seg:
                last_xy = (seg[-1].x, seg[-1].y)
            lane = lane_end
        return pts


class PlanError(RuntimeError):
    """경로를 만들 수 없거나 연속성 검증에 실패했을 때."""


def _lane_gap(rd_a: Road, s_a: float, lane_a: int, rd_b: Road, s_b: float, lane_b: int) -> float | None:
    """도로 a 의 (s_a, lane_a) 차로 중심 ↔ 도로 b 의 (s_b, lane_b) 차로 중심 거리."""
    try:
        xa, ya, _, _ = rd_a.pose_at(s_a, rd_a.lane_center_t(s_a, lane_a))
        xb, yb, _, _ = rd_b.pose_at(s_b, rd_b.lane_center_t(s_b, lane_b))
    except KeyError:
        return None
    return math.hypot(xb - xa, yb - ya)


def _lane_by_xy(rd: Road, s: float, cands: list[int], xy: tuple[float, float]) -> int:
    """(x, y) 에 가장 가까운 차로 중심을 가진 차로."""
    px, py, _, h = rd.pose_at(s)
    t_prev = -math.sin(h) * (xy[0] - px) + math.cos(h) * (xy[1] - py)

    def tc(i):
        try:
            return rd.lane_center_t(s, i)
        except KeyError:
            return 1e9
    return min(cands, key=lambda i: abs(tc(i) - t_prev))


def on_drivable(net: RoadNetwork, p: PathPoint, tol: float = 0.3) -> bool:
    """점이 자기 도로에서 같은 진행방향(차로 부호) 주행차로들의 합집합 안에 있는가."""
    rd = net.roads[p.road]
    sec = rd.section_at(p.s)
    if sec is None:
        return False
    px, py, _, h = rd.pose_at(p.s)
    t = -math.sin(h) * (p.x - px) + math.cos(h) * (p.y - py)
    lo, hi = float("inf"), float("-inf")
    for lid in rd.driving_lanes(p.s):
        if (lid < 0) != (p.lane < 0):
            continue
        try:
            tc = rd.lane_center_t(p.s, lid)
        except KeyError:
            continue
        w = sec.lane(lid).width(p.s - sec.s)
        lo, hi = min(lo, tc - w / 2), max(hi, tc + w / 2)
    return lo - tol <= t <= hi + tol


MAX_GAP = 2.5            # 인접 점 간 허용 거리 [m]
MAX_TURN_DEG = 35.0      # 인접 점 간 허용 heading 변화 (교차로 연결도로 각도 수용)
SEAM_TURN_DEG = 18.0     # 이 이상이면 이음새로 보고 재보간
SEAM_BACK_M = 20.0        # 이음새를 되짚어 다시 그릴 거리. 20 m 였을 때는
                         # 교차로 정지선 한참 전부터 경로가 돌기 시작해서
                         # 차가 차로 안에서 미리 틀어진 채 진입했다.


def finalize_path(pts: list[PathPoint], net=None) -> list[PathPoint]:
    """중복 제거 → heading 재계산 → 교차로 회전 지연 → 이음새 재보간 → 연속성 검증."""
    if len(pts) < 2:
        raise PlanError("경로 점이 2개 미만")
    out = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p.x - out[-1].x, p.y - out[-1].y) >= 0.3:
            out.append(p)

    def reheading(lo: int, hi: int) -> None:
        for i in range(max(lo, 0), min(hi, len(out))):
            a = out[max(i - 1, 0)]
            b = out[min(i + 1, len(out) - 1)]
            if a is not b:
                out[i].heading = math.atan2(b.y - a.y, b.x - a.x)

    reheading(0, len(out))

    i = 1
    while i < len(out):
        a, b = out[i - 1], out[i]
        gap = math.hypot(b.x - a.x, b.y - a.y)
        turn = abs(_wrap(b.heading - a.heading))
        if gap > MAX_GAP or turn > math.radians(SEAM_TURN_DEG):
            # 이음새 앞 SEAM_BACK_M 를 Hermite 로 다시 그린다 (양끝 heading 을 접선으로)
            j = i - 1
            back = 0.0
            while j > 0 and back < SEAM_BACK_M:
                back += math.hypot(out[j].x - out[j - 1].x, out[j].y - out[j - 1].y)
                j -= 1
            p0, p1 = out[j], out[i]
            old = out[j:i + 1]                                   # 속성(도로·차로·교차로) 보존용
            L = math.hypot(p1.x - p0.x, p1.y - p0.y)
            T = 0.6 * L                                          # 접선 크기: 곡선 바깥 부풀림 억제
            n = max(2, int(L / STEP_M) + 1, int(math.ceil(L / (MAX_GAP - 0.2))) + 1)
            new = []
            for k in range(1, n):
                u = k / n
                h00 = 2 * u ** 3 - 3 * u ** 2 + 1
                h10 = u ** 3 - 2 * u ** 2 + u
                h01 = -2 * u ** 3 + 3 * u ** 2
                h11 = u ** 3 - u ** 2
                x = h00 * p0.x + h10 * T * math.cos(p0.heading) + h01 * p1.x + h11 * T * math.cos(p1.heading)
                y = h00 * p0.y + h10 * T * math.sin(p0.heading) + h01 * p1.y + h11 * T * math.sin(p1.heading)
                # 도로/차로/교차로 표시는 가장 가까운 원래 점에서 가져온다 (라벨 소실 방지)
                src = min(old, key=lambda q: (q.x - x) ** 2 + (q.y - y) ** 2)
                new.append(PathPoint(x, y, 0.0, src.road, src.lane, src.s, src.in_junction))
            out[j + 1:i] = new
            reheading(j, j + len(new) + 2)
            i = j + len(new) + 2
            continue
        i += 1

    for k in range(1, len(out)):
        a, b = out[k - 1], out[k]
        gap = math.hypot(b.x - a.x, b.y - a.y)
        turn = math.degrees(abs(_wrap(b.heading - a.heading)))
        if gap > MAX_GAP + 0.2 or turn > MAX_TURN_DEG:
            raise PlanError(f"경로 연속성 실패: 점 {k} (road {b.road}) gap={gap:.2f} m, heading 점프={turn:.1f}°")
    return out


# ---------------------------------------------------------------- 유틸
def load_waypoints(path: str) -> list[tuple[float, float]]:
    with open(path, newline="", encoding="utf-8") as f:
        return [(float(r["x"]), float(r["y"])) for r in csv.DictReader(f)]


def path_length(path: list[PathPoint]) -> float:
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(path, path[1:]))


def write_path_csv(path: list[PathPoint], out: str) -> None:
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "heading", "road", "lane", "s", "junction"])
        for p in path:
            w.writerow([f"{p.x:.3f}", f"{p.y:.3f}", f"{p.heading:.5f}", p.road, p.lane, f"{p.s:.2f}", int(p.in_junction)])


def read_path_csv(path: str) -> list[PathPoint]:
    out = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            out.append(PathPoint(float(r["x"]), float(r["y"]), float(r["heading"]), r["road"],
                                 int(r["lane"]), float(r["s"]), r["junction"] == "1"))
    return out


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("사용: python3 planner.py route.csv [out.csv]"); sys.exit(1)
    net = RoadNetwork(str(__import__("pathlib").Path(__file__).resolve().parent.parent / "data" / "HL_FMA_VTD_LivingLab.xodr"))
    rp = RoutePlanner(net)
    wps = load_waypoints(sys.argv[1])
    path = rp.plan(wps)
    for i, l in enumerate(rp.locs):
        print(f"  wp{i+1:<2} → road {l.road.id:>5} s={l.s:6.1f} lane {l.lane:+d} dir {l.dir:+d} dist {l.dist:4.1f}{' J' if l.road.junction != '-1' else ''}")
    roads = []
    for p in path:
        if not roads or roads[-1] != p.road:
            roads.append(p.road)
    straight = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(wps, wps[1:]))
    print(f"\n경로 점 {len(path)}개, 길이 {path_length(path):.1f} m (경유지 직선합 {straight:.1f} m), 도로 {len(roads)}개")
    print("  " + " → ".join(roads))
    # 검증: 각 점이 자기 도로(p.road)의 같은 진행방향 주행차로 영역 안에 있는지
    # (차로변경 블렌드 중엔 두 차로 사이에 있으므로 "차로 하나" 가 아니라 "주행 가능 영역" 으로 본다)
    from collections import Counter
    off = Counter(p.road for p in path if not on_drivable(net, p))
    print(f"주행영역 밖 점: {sum(off.values())}/{len(path)}" +
          (f"  (도로별: {dict(off.most_common(6))})" if off else ""))
    # 횡 점프 검사: 인접 점 간 거리가 STEP 의 2배를 넘는 곳
    jumps = [(i, math.hypot(b.x - a.x, b.y - a.y)) for i, (a, b) in enumerate(zip(path, path[1:]))
             if math.hypot(b.x - a.x, b.y - a.y) > STEP_M * 2.2]
    print(f"인접점 간격 {STEP_M*2.2:.1f} m 초과(점프) {len(jumps)}곳" +
          (f": {[(i, round(d,1), path[i].road) for i, d in jumps[:6]]}" if jumps else ""))
    if len(sys.argv) > 2:
        write_path_csv(path, sys.argv[2]); print("저장:", sys.argv[2])