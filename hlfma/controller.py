"""주행 제어기 — 상태기계 + 순수추종 + 규칙 기반 속도 + 워치독.

상태: DRIVE → (TL_STOP ↔ DRIVE) → FINISHING → DONE, 어디서든 RECOVER 로.
모든 결정은 GT 데이터(ego/objects/trafficLight)와 계획 경로만으로 한다.
"""
from __future__ import annotations

import math

from .config import RunConfig
from .hlvtd import (TL_GREEN, TL_YELLOW, TL_RED, TL_FLASH, TL_GO, TL_NAME,
                    TURN_LEFT, TURN_OFF, TURN_RIGHT)
from .perception import (relate, lead_vehicle, pedestrians, blocking_cones,
                         find_cone_line, side_of_line)
from .route import Route, wrap

AVOID_MARGIN = 0.5       # 콘 옆을 지날 때 남길 여유 [m].
                         # blocking_cones 의 margin(0.4) 보다 커야 한다 — 작으면 오프셋을
                         # 다 넣은 뒤에도 같은 콘이 계속 '차로를 막음' 으로 잡혀
                         # 2 m/s 서행이 안 풀린다 (실측: -2.2 m 회피 후에도 서행 유지).
AVOID_RATE = 1.0         # 횡 오프셋 변화율 [m/s] — 급조향 방지
AVOID_SPEED = 6.0        # 회피 기동 중 속도 상한 [m/s]
OVERTAKE_WAIT_S = 8.0    # 선행차가 이만큼 계속 막아야 '정차 차량'으로 본다
OVERTAKE_GAP = 15.0      # 이 거리 안에 멈춰 있어야 막힌 것으로 센다 [m]
JERK_MAX = 6.0           # 가속도 변화 상한 [m/s^3]. 급가감속을 막는다
CONE_HALF = 0.23         # 라바콘 반폭 [m]


class Controller:
    def __init__(self, route: Route, cfg: RunConfig, log=None):
        self.r = route
        self.cfg = cfg
        self.c = cfg.control
        self.veh = cfg.vehicle
        self.log = log
        self.state = "DRIVE"
        self.idx = 0
        self.cum = 0.0            # 연속 누적거리 (r.cum[idx] 는 2 m 양자화라 정지·종료 판정에 못 쓴다)
        self.prev = None
        self.v = 0.0
        self.steer_prev = 0.0
        self.respawns = 0
        self.grace = 0.0
        self.t = 0.0
        # 워치독
        self.wd_cum = 0.0
        self.wd_t = 0.0
        self.stuck_level = 0
        self.push_left = 0.0
        # 종료
        self.finish_line = None
        self.finish_side = None
        self.goal_cum = route.length + 2.0
        self.done_t = 0.0
        self.notes: list[str] = []
        self.junction_green = False   # 진입하려는(또는 통과 중인) 교차로가 녹색인가
        self.tl_done_id = 0           # 교차로 루프가 이미 처리를 끝낸 신호 id
        # --- 상황판단 계층 상태 (매 프레임 뒤집히지 않게 값을 들고 간다) ---
        self.seen_junc = set()        # 이미 '인지' 로그를 남긴 교차로
        self.lane_now = None          # 직전 프레임의 경로 차로 (변경 시작/완료 판정)
        self.lane_changing = False
        self.stop_reason = ""        # 지금 멈춰 있는 이유 (전이 시에만 로그)
        self.block_s = 0.0            # 선행차가 '계속' 길을 막은 시간 [s]
        self.ot_id = 0                # 추월 중인 대상 객체 id (0 = 없음)
        self.ot_off = 0.0             # 그때 쓴 횡 오프셋
        self.queued = False           # 신호/교차로 때문에 선 줄인가
        self.ped_stall_s = 0.0        # 보행자 때문에 멈춰 있는 시간 [s]
        self.accel_prev = 0.0
        self.prep_junc = None         # 준비 중인 회전 교차로 (로그 1회용)
        self.aligned_junc = None      # '정렬 유지' 로그를 남긴 교차로
        self.avoid = 0.0              # 현재 적용 중인 횡회피 오프셋 [m], 좌측 +
        self.avoid_prev = 0.0         # 직전 프레임 값 (지시등 방향 판정)
        self.status = ""

    # ------------------------------------------------------------ 보조
    def stop_speed(self, dist: float, a: float | None = None) -> float:
        """dist 앞에서 정지하기 위한 속도 상한. 가장 빡빡한 정지거리를 선행보상용으로 기록한다.
        a 를 낮게 주면 같은 거리에서 상한이 낮아진다 = 더 일찍 감속한다 (보행자용)."""
        self.stop_dist = min(self.stop_dist, dist)
        return math.sqrt(max(0.0, 2.0 * (self.c.a_brake if a is None else a) * dist))

    def _event(self, text: str) -> None:
        print(f"  [{self.t:6.1f}s] {text}")
        if self.log:
            self.log.event(text)

    # ------------------------------------------------------------ 위치/속도
    def localize(self, x: float, y: float) -> None:
        c = self.c
        if self.prev is not None:
            jump = math.hypot(x - self.prev[0], y - self.prev[1])
            if jump > c.respawn_jump:
                self.respawns += 1
                self.idx = self.r.nearest(x, y, 0, self.r.n)
                self.v = 0.0
                self.steer_prev = 0.0
                self.grace = 1.0
                for J in self.r.junctions:
                    J.done = self.idx >= J.entry
                    J.tl_latched = -1
                    J.stopped_s = 0.0
                self.cum = self.r.cum_xy(x, y, self.idx)
                self._event(f"리스폰 {self.respawns}: {jump:.1f} m 점프 → idx {self.idx} (state {self.state}→DRIVE)")
                self.state = "DRIVE" if self.r.length - self.cum > 40.0 else "FINISHING"
                self.wd_cum, self.wd_t, self.stuck_level = self.cum, self.t, 0
            elif jump <= c.teleport_sample:
                self.v = 0.7 * self.v + 0.3 * (jump / c.dt)
        self.prev = (x, y)
        self.idx = self.r.nearest(x, y, self.idx, 60)
        self.cum = self.r.cum_xy(x, y, self.idx)

    def watchdog(self, v_t: float) -> None:
        """진행이 없으면 단계적으로 제약을 푼다 (무조작 요건)."""
        c = self.c
        cum = self.cum
        if cum - self.wd_cum > 1.0 or v_t < 0.5 or self.state in ("TL_STOP", "DONE"):
            self.wd_cum, self.wd_t = cum, self.t
            if self.state != "TL_STOP":
                self.stuck_level = 0
            return
        dt_stuck = self.t - self.wd_t
        if dt_stuck > c.stuck_l3_s and self.stuck_level < 3:
            self.stuck_level = 3
            self._event("워치독 L3: 미확인 신호 대기 해제")
        elif dt_stuck > c.stuck_l2_s and self.stuck_level < 2:
            self.stuck_level = 2
            self.idx = self.r.nearest(self.prev[0], self.prev[1], 0, self.r.n)
            self._event(f"워치독 L2: 전체 재위치 → idx {self.idx}")
        elif dt_stuck > c.stuck_l1_s and self.stuck_level < 1:
            self.stuck_level = 1
            self.push_left = 3.0
            self._event("워치독 L1: 3 s 동안 저속 전진 시도")

    # ------------------------------------------------------------ 횡회피
    def _drivable(self, cones, off: float) -> bool:
        """오프셋 경로가 장애물 구간 내내 주행 차로 위인지.

        net.project_lane 은 쓰지 않는다 — 도로가 겹친 곳(road 3195 옆의 2063/2075)에서
        참조선이 더 가까운 남의 도로로 투영돼 멀쩡한 경로점도 None 이 나온다.
        경로점이 자기 road/lane/s 를 들고 있으니 그 도로의 차로 구성으로 직접 판정한다.
        """
        r = self.r
        lo = max(0, min(q.idx for q in cones) - 3)
        hi = min(r.n, max(q.idx for q in cones) + 4)
        for k in range(lo, hi):
            p = r.p[k]
            rd = r.net.roads.get(p.road)
            if rd is None:
                return False
            sec = rd.section_at(p.s)
            if sec is None:
                return False
            try:
                t = rd.lane_center_t(p.s, p.lane) + off      # 오프셋 후 횡위치
            except KeyError:
                return False
            ok = False
            for lid in rd.driving_lanes(p.s):
                ln = sec.lane(lid)
                if ln is None:
                    continue
                w = ln.width(p.s - sec.s)
                # 같은 진행방향 차로만 (부호가 같아야 한다 — 대향차로로 넘어가면 안 된다)
                if lid * p.lane > 0 and abs(t - rd.lane_center_t(p.s, lid)) <= w / 2.0:
                    ok = True
                    break
            if not ok:
                return False
        return True

    def _lane_at(self, p, off: float) -> int | None:
        """경로점 p 에서 횡오프셋 off 를 준 위치가 어느 주행 차로인지. 없으면 None."""
        rd = self.r.net.roads.get(p.road)
        sec = rd.section_at(p.s) if rd else None
        if sec is None:
            return None
        try:
            t = rd.lane_center_t(p.s, p.lane) + off
        except KeyError:
            return None
        for lid in rd.driving_lanes(p.s):
            ln = sec.lane(lid)
            if ln is None:
                continue
            if abs(t - rd.lane_center_t(p.s, lid)) <= ln.width(p.s - sec.s) / 2.0:
                return lid
        return None

    def _crosses_solid(self, cones, off: float) -> bool:
        """그 오프셋으로 비키면 실선을 넘는가. 채점 항목 '실선 차로변경 금지'.
        넘더라도 대안이 없으면 쓰지만, 점선 쪽을 먼저 시도한다."""
        r = self.r
        lo = max(0, min(q.idx for q in cones) - 3)
        hi = min(r.n, max(q.idx for q in cones) + 4)
        for k in range(lo, hi):
            p = r.p[k]
            tgt = self._lane_at(p, off)
            if tgt is None or tgt == p.lane:
                continue
            rd = r.net.roads.get(p.road)
            if rd and "solid" in rd.boundary_mark(p.s, p.lane, tgt):
                return True
        return False

    def avoid_offset(self, rels) -> float:
        """장애물을 비켜갈 횡 오프셋[m]. 0 이면 회피 불필요 또는 불가.

        차로 안(±0.6 m)으로는 못 비키는 경우가 많아(차로 3.15 m, 차폭 1.89 m) 옆 차로까지
        허용한다. 경계가 실선이면 project_lane 이 아니라 주행 차로 여부만 보므로, 실선
        구분은 하지 않는다 — 점선/실선 판정이 필요해지면 여기에 붙인다.
        """
        # 미리 비킨다: 정지거리의 2 배쯤 앞에서 시작해야 뒷차·교통 흐름을 덜 건드린다.
        # RDB 가 80 m 까지 주므로 그 안에서 속도에 맞춰 늘린다 (45 km/h → 72 m).
        # 하한 65 m: 아래 '콘 서행'(2 m/s) 판정이 60 m 앞부터 걸린다. 회피 판정이 40 m 만
        # 보면 20 m 를 2 m/s 로 기어간 뒤에야 비켜, 장애물 1개당 10 s 를 버린다(실측).
        # 서행이 걸리기 전에 차로를 옮기도록 두 창을 맞춘다.
        see = min(80.0, max(65.0, self.v * self.v / self.c.a_brake + 10.0))
        cones = [q for q in blocking_cones(rels, self.veh.obb_half_width, ahead=see)
                 if -3.0 <= q.along <= see]
        if not cones:
            return 0.0
        half = max(CONE_HALF, max(q.ob["width"] / 2.0 for q in cones))
        clear = self.veh.obb_half_width + half + AVOID_MARGIN
        cands = (max(q.lat_s for q in cones) + clear,      # 좌측으로 비키기
                 min(q.lat_s for q in cones) - clear)      # 우측으로 비키기
        # 실선을 넘는 쪽은 뒤로 미룬다 (채점: 실선 차로변경 금지). 대안이 없으면 쓴다 —
        # 정적 장애물 충돌보다는 낫고, 어차피 안 비키면 서행으로 들이받는다.
        for off in sorted(cands, key=lambda o: (self._crosses_solid(cones, o), abs(o))):
            if self._drivable(cones, off) and self._side_clear(rels, off, skip=cones):
                if self._crosses_solid(cones, off):
                    self.notes.append("실선회피")
                return off
        return 0.0

    def _overtake_want(self, rels, lead) -> float:
        """추월 목표 오프셋. 한 번 시작하면 대상을 지날 때까지 유지한다(래치).

        래치가 없으면 비키자마자 lead_vehicle 시야(±1.7 m)에서 대상이 빠져 want 가 0 이
        되고, 매 프레임 좌우로 흔들린다.
        """
        if self.ot_id:
            tgt = next((q for q in rels if q.ob["id"] == self.ot_id), None)
            if tgt is not None and tgt.along > -(self.veh.rear_to_rear + 2.0):
                return self.ot_off                      # 아직 못 지났다 → 유지
            self._event(f"추월 완료: 객체 {self.ot_id} 통과, 원래 경로 복귀")
            self.ot_id, self.ot_off = 0, 0.0
            return 0.0
        off = self._overtake_offset(rels, lead[1] if lead else None, self.queued)
        if off != 0.0 and lead is not None:
            self.ot_id, self.ot_off = lead[1].ob["id"], off
            self._event(f"추월 시작: 객체 {self.ot_id} 가 {self.block_s:.0f}s 정차, "
                        f"횡 {off:+.1f} m 로 비켜감 (옆·후방 확인 완료)")
        return off

    def _overtake_offset(self, rels, lead_rel, queued: bool) -> float:
        """계속 길을 막고 선 차를 비켜갈 횡 오프셋[m]. 0 이면 하지 않는다.

        판단 순서 (하나라도 막히면 0 = 기다린다):
          1) 신호·교차로 때문에 선 줄인가(queued) → 추월하지 않는다. 정체·신호 대기 차를
             밀고 나가면 규정 위반이고, 어차피 곧 움직인다.
          2) 같은 자리에서 block_s 이상 막고 있는가 → 일시 정차가 아니라 '고장/주차'로 본다.
          3) 비킬 자리가 주행 차로 위인가(_drivable) + 옆·뒤가 비어 있는가(_side_clear).
             _side_clear 는 후방 상대속도까지 본다. 후방 객체는 실측으로 60 m 까지 온다.
        관측이 모자라면(옆 차로 판정 실패) 0 을 돌려 기다린다.
        """
        if queued or lead_rel is None:
            return 0.0
        if self.block_s < OVERTAKE_WAIT_S:
            return 0.0
        clear = self.veh.obb_half_width + lead_rel.ob["width"] / 2.0 + AVOID_MARGIN
        cands = (lead_rel.lat_s + clear, lead_rel.lat_s - clear)
        for off in sorted(cands, key=abs):
            if self._drivable([lead_rel], off) and self._side_clear(rels, off, skip=[lead_rel]):
                return off
        return 0.0

    def _side_clear(self, rels, off: float, skip=()) -> bool:
        """비켜갈 자리가 비어 있는지.

        옆 차로에 차가 있는데 밀고 들어가면 안 된다. 뒤쪽은 **상대속도**로 본다 —
        나보다 빠른 차가 뒤에 있으면 3 초 안에 따라붙는 거리까지 비워져 있어야 한다.
        """
        span = self.veh.obb_half_width + 1.0
        skip_ids = {q.ob["id"] for q in skip}
        for q in rels:
            if q.kind == "cone" or q.ob["id"] in skip_ids:
                continue
            if abs(q.lat_s - off) >= span + q.ob["width"] / 2.0:
                continue
            back = -(10.0 + max(0.0, q.ob["speed"] - self.v) * 3.0)
            if back <= q.along <= 45.0:
                return False
        return True

    # ------------------------------------------------------------ 메인
    def step(self, x: float, y: float, h: float, tl: tuple[int, int], objs: list[dict]):
        c, r = self.c, self.r
        self.t += c.dt
        self.notes = []
        self.stop_dist = float("inf")
        self.localize(x, y)
        i = self.idx
        v = self.v
        e_lat = r.lateral_error(x, y, i)
        remaining = r.length - self.cum
        rels = relate(r, i, self.cum, objs)

        # ---- 횡회피/추월: 목표 오프셋을 속도제한으로 따라간다 ----
        # lead 를 여기서 한 번만 구해 아래 객체 절과 같이 쓴다.
        lead = lead_vehicle(rels, rear_to_front=self.veh.rear_to_front)
        want = self.avoid_offset(rels)
        if want == 0.0:
            want = self._overtake_want(rels, lead)
        lim = AVOID_RATE * c.dt
        self.avoid_prev = self.avoid
        self.avoid += max(-lim, min(lim, want - self.avoid))
        if abs(self.avoid) < 0.02:
            self.avoid = 0.0
        # 오프셋 주행 중에는 그만큼 벗어나 있는 게 정상이다 (RECOVER 오발동 방지)
        e_lat -= self.avoid

        if self.state == "DONE":
            self.status = "DONE"
            return 0.0, -2.0, TURN_OFF, e_lat

        # ---- 조향: 순수추종 ----
        ld = max(c.look_min, min(c.look_max, c.look_base + c.look_k * v))
        kk = r.max_kappa(i, max(8.0, ld))
        if kk > 1e-3:
            ld = max(c.look_min_curve, 0.6 * v, min(ld, math.sqrt(2.0 * c.look_err / kk)))
        tgt = r.p[r.index_ahead(i, ld)]
        tx = tgt.x - self.avoid * math.sin(tgt.heading)
        ty = tgt.y + self.avoid * math.cos(tgt.heading)
        alpha = wrap(math.atan2(ty - y, tx - x) - h)
        steer = math.atan2(2.0 * self.veh.wheelbase * math.sin(alpha), ld)
        lim = c.steer_max_fast if v > 10.0 else c.steer_max
        steer = max(-lim, min(lim, steer))
        rate = c.steer_rate * c.dt
        steer = max(self.steer_prev - rate, min(self.steer_prev + rate, steer))
        self.steer_prev = steer

        # ---- 속도 상한 (완주 우선: 순항 고정) ----
        v_t = min(self.cfg.cruise, self.cfg.speed_limit)
        v_t = min(v_t, r.cap_at(self.cum, v_t))                      # 30 km/h 근사 구간
        # 곡선 감속에 필요한 거리만큼 앞을 본다. 순항 45 km/h 에서 급커브(R 16 m,
        # 상한 5.4 m/s)로 줄이려면 25 m 로는 모자란다(필요 25.4 m).
        k = r.max_kappa(i, max(25.0, v * v / (2.0 * c.a_brake) + 8.0))
        if k > 1e-4:
            v_t = min(v_t, math.sqrt(c.a_lat / k))
        # ---- 회전 준비: 차로변경 시작점까지 v_prep 로 내려가는 속도 계획 ----
        # 곡률 감속은 완만한 차로변경 곡선엔 걸리지 않아 순항 45 km/h 그대로 차로를
        # 옮기고 정지선 직전에야 줄였다(실측: 정지선 67 m 전 53 km/h). 회전 교차로를
        # prep_horizon 부터 보고, 첫 차로변경 지점(없으면 정지선 40 m 전) 에서 v_prep 가
        # 되도록 a_prep 로 거꾸로 그린 프로파일을 상한으로 건다.
        Jn = next((J for J in r.junctions
                   if not J.done and J.turn != TURN_OFF and 0.0 < J.entry_cum - self.cum < c.prep_horizon), None)
        if Jn is not None and self.state != "RECOVER":
            lane0, lc_cum = r.p[i].lane, None
            for k in range(i, Jn.entry):
                if r.p[k].lane != lane0:
                    lc_cum = r.cum[k]
                    break
            tgt_cum = lc_cum if lc_cum is not None else Jn.stop_cum - 40.0
            d_prep = max(0.0, tgt_cum - self.cum - c.prep_margin)
            v_cap = math.sqrt(c.v_prep ** 2 + 2.0 * c.a_prep * d_prep)
            if v_cap < v_t:
                v_t = v_cap
                self.notes.append(f"회전준비 {v_cap:.1f}")
            if self.prep_junc is not Jn:
                self.prep_junc = Jn
                lane_goal = r.p[Jn.entry - 1].lane
                n_lc = sum(1 for k in range(i + 1, Jn.entry) if r.p[k].lane != r.p[k - 1].lane)
                self._event(f"회전 예정 확인: {['','좌회전','우회전'][Jn.turn]} 교차로 "
                            f"{Jn.entry_cum - self.cum:.0f} m, 정지선 {Jn.stop_cum - self.cum:.0f} m, "
                            f"차로 {lane0}→{lane_goal} (변경 {n_lc}회, 첫 변경점 "
                            f"{(tgt_cum - self.cum):.0f} m 앞), 속도 {v:.1f}→{c.v_prep:.1f} m/s")
            # 목표 차로 진입 + 정렬 확인 (위치 기준: 경로 차로 일치 & 횡오차 작음)
            if (self.aligned_junc is not Jn and r.p[i].lane == r.p[Jn.entry - 1].lane
                    and not self.lane_changing and abs(e_lat) < 0.25):
                self.aligned_junc = Jn
                self._event(f"차로 정렬 완료: 차로 {r.p[i].lane} 중앙(e={e_lat:+.2f} m), "
                            f"정지선 {Jn.stop_cum - self.cum:.0f} m 전, v={v:.1f} m/s — 이 상태로 접근")
        if r.p[i].in_junction or r.p[r.index_ahead(i, 8.0)].in_junction:
            # 녹색이면 서행이 오히려 흐름을 막는다. 신호 상태는 아래 루프에서 직전
            # 프레임에 갱신된 값을 쓴다(50 ms 지연은 무시 가능). 곡선 상한은 그대로
            # 적용되므로 회전 경로에서는 여전히 5~6 m/s 로 묶인다.
            v_t = min(v_t, c.v_junction_green if self.junction_green else c.v_junction)
        if abs(e_lat) > c.lat_err_recover:
            if self.state not in ("RECOVER", "FINISHING"):
                self.state = "RECOVER"
                self._event(f"횡오차 {e_lat:+.1f} m → RECOVER")
        elif self.state == "RECOVER" and abs(e_lat) < 0.8:
            self.state = "DRIVE"
            self._event("RECOVER → DRIVE")
        if abs(e_lat) > c.lat_err_slow or self.state == "RECOVER":
            v_t = min(v_t, 3.0 if abs(e_lat) > c.lat_err_slow else v_t)
            if self.state == "RECOVER":
                v_t = min(v_t, 2.0)
        if self.grace > 0.0:
            self.grace -= c.dt
            v_t = 0.0

        # ---- 교차로 / 신호 ----
        signal = TURN_OFF
        hard_stop_cum = None
        tl_handled = False
        for J in r.junctions:
            if J.done:
                continue
            if i >= J.entry:
                if not J.done and J.turn != TURN_OFF:
                    self._event(f"회전 시작: {['직진','좌회전','우회전'][J.turn]} 교차로 진입 "
                                f"(신호 {'녹색' if J.tl_latched in TL_GO else TL_NAME.get(J.tl_latched, '없음')})")
                J.done = True
                # 교차로를 빠져나갈 때까지 진입 시점의 녹색 판정을 유지한다.
                # (done 이 되면 아래 신호 분기를 안 타므로 여기서 확정해 둔다)
                self.junction_green = (J.tl_latched in TL_GO)
                if tl[0] != 0:
                    self.tl_done_id = tl[0]
                if self.state == "TL_STOP":
                    self.state = "DRIVE"
                continue
            d_entry = J.entry_cum - self.cum
            d_stop = J.stop_cum - self.cum
            if d_entry > 80.0:
                break
            if id(J) not in self.seen_junc:
                self.seen_junc.add(id(J))
                self._event(f"교차로 인지: {['직진','좌회전','우회전'][J.turn]} "
                            f"진입 {d_entry:.0f} m, 정지선 {d_stop:.0f} m, "
                            f"정지선{'있음' if J.s_line is not None else '없음(추정)'}")
            if d_entry <= c.signal_lead and J.turn != TURN_OFF:
                signal = J.turn
            tl_id, st = tl
            if tl_id != 0:
                J.tl_latched = st                                    # 변화는 즉시 반영
            elif J.tl_wait_s >= c.tl_unknown_wait_s:
                # 신호 정보가 끊긴(id=0) 채 래치만 남으면 영원히 대기한다. TL_STOP 에서는
                # 워치독도 리셋되어 탈출구가 없으므로, 대기 상한을 넘으면 래치를 푼다
                # → 아래 st == 0 분기(비신호 교차로: 서행·양보)로 안전하게 진행한다.
                J.tl_latched = -1
            st = J.tl_latched if J.tl_latched >= 0 else 0
            name = TL_NAME.get(st, str(st))
            at_line = d_stop < 0.4
            self.junction_green = (st in TL_GO)
            if st in TL_GO:
                self.notes.append("녹색")
                if self.state == "TL_STOP":
                    self.state = "DRIVE"
            elif st == 0:
                # 신호 정보 없음 = 비신호 교차로: 서행, 접근 차량 있으면 양보
                if d_entry < 25.0:
                    v_t = min(v_t, c.v_junction)
                    yield_car = [q for q in rels if q.kind == "car" and abs(q.along - d_entry) < 25.0
                                 and q.lat < 15.0 and q.closing > 0.5]
                    if yield_car:
                        v_t = min(v_t, self.stop_speed(max(0.0, d_stop)))
                        self.notes.append(f"비신호 양보 {len(yield_car)}대")
                    else:
                        self.notes.append("비신호 서행")
            elif st == TL_FLASH or (st == 4 and (J.tl_wait_s >= c.tl_unknown_wait_s or self.stuck_level >= 3)):
                # 적색점멸: 정지선 일시정지(1 s) 후 서행 통과
                if J.stopped_s < 1.0:
                    v_t = min(v_t, self.stop_speed(max(0.0, d_stop)))
                    if at_line and v < 0.3:
                        J.stopped_s += c.dt
                    self.notes.append(f"점멸 일시정지 {J.stopped_s:.1f}s")
                else:
                    v_t = min(v_t, 3.0)
                    self.notes.append("점멸 통과")
                    if self.state == "TL_STOP":
                        self.state = "DRIVE"
            elif st == TL_YELLOW and d_stop < (v * v) / (2.0 * 3.0):
                self.notes.append("황색 커밋(정지불가)")
            else:
                # 적색 / 황색(정지 가능) / 미확인 코드: 정지선 앞 정지
                v_t = min(v_t, self.stop_speed(max(0.0, d_stop)))
                hard_stop_cum = J.stop_cum
                if at_line and v < 0.3:
                    J.tl_wait_s += c.dt
                    self.state = "TL_STOP"
                self.notes.append(f"{name} 정지선 {d_stop:.1f}m")
            tl_handled = True
            break

        # ---- 미등록 교차로의 신호 ----
        # VTD 는 교차로 접근 도로에서만 신호를 준다(PROTOCOL.md). 즉 tl_id != 0 인데
        # 위 루프가 처리하지 못했다면, 계획기가 교차로로 잡지 못한 신호 교차로가
        # 앞에 있다는 뜻이다 (실측: 경로 정지선 12개 중 교차로 등록은 4개뿐).
        # 그대로 두면 적색을 그냥 통과한다 — 실주행에서 3곳에서 발생했다.
        if tl[0] != 0 and not tl_handled and tl[0] != self.tl_done_id:
            st = tl[1]
            if st not in TL_GO and st != TL_FLASH:
                ahead = [c0 for c0 in r.stop_cums if c0 - self.cum > -0.5]
                if ahead and ahead[0] - self.cum < 60.0:
                    d_stop = ahead[0] - self.cum
                    v_t = min(v_t, self.stop_speed(max(0.0, d_stop)))
                    hard_stop_cum = ahead[0]
                    if d_stop < 0.4 and v < 0.3:
                        self.state = "TL_STOP"
                    self.notes.append(f"{TL_NAME.get(st, st)} 정지선(미등록) {d_stop:.1f}m")
        elif self.state == "TL_STOP" and (tl[0] == 0 or tl[1] in TL_GO):
            self.state = "DRIVE"

        # ---- 객체 ----
        # 신호·교차로 때문에 선 줄인가? 그렇다면 추월 대상이 아니다. (다음 프레임 판단에 쓴다)
        self.queued = (hard_stop_cum is not None and hard_stop_cum - self.cum < 45.0) or \
                      self.state == "TL_STOP" or not self.junction_green
        if lead is not None and lead[1].ob["speed"] < 0.3 and lead[0] < OVERTAKE_GAP and v < 0.6:
            self.block_s += c.dt
        else:
            self.block_s = 0.0
        if lead is not None:
            gap, q = lead
            d_des = 6.0 + 2.0 * v
            if gap < 4.0:
                v_t = 0.0
            elif gap < d_des + 12.0:
                v_t = min(v_t, max(0.0, q.ob["speed"] + (gap - d_des) * 0.5))
            self.notes.append(f"선행차 {gap:.1f}m v={q.ob['speed']:.1f}")
        if self.avoid != 0.0:
            v_t = min(v_t, AVOID_SPEED)
            self.notes.append(f"회피 {self.avoid:+.1f}m")
        still = blocking_cones(rels, self.veh.obb_half_width, offset=self.avoid)
        if still:
            # 비킬 수 없는 콘. **멈추지 않는다** — 완주 실패가 접촉보다 큰 손실이고
            # v_t<0.5 면 워치독도 리셋되어 영구 정지가 된다. 서행으로 통과한다.
            v_t = min(v_t, 2.0)
            self.notes.append(f"콘 {still[0].along:.1f}m 서행")
        # 탐지거리는 정지거리보다 넉넉해야 한다. RDB 가 80 m 까지 주므로 그 안에서
        # 속도에 맞춰 늘린다 (45 km/h → 57 m).
        see = min(80.0, max(30.0, v * v / (2.0 * c.a_brake_ped) + 2.0 * self.veh.rear_to_front + 10.0))
        # 보행자 때문에 너무 오래 서 있으면(인도에 가만히 선 사람) 서행 통과로 내린다.
        # 그대로 두면 v_t<0.5 라 워치독도 리셋되어 영구 정지가 된다(실측: pretest 경로
        # idx 163 에서 측방 3.5 m 에 서 있는 보행자 앞에 무한 정차).
        creep = self.ped_stall_s > c.ped_stall_s
        ped_blocking = False
        for q in pedestrians(rels, ahead=see):
            # 정지 대상: ① 내 진로 위(ped_corridor) ② 경로 쪽으로 다가오는 중.
            # 인도에 **서 있는** 사람(정지·측방 여유)은 정지가 아니라 감속 대상이다.
            moving = q.ob["speed"] > 0.3
            in_path = q.lat <= c.ped_corridor
            approaching = q.closing > 0.3 and q.lat <= 8.0
            if (in_path or approaching or (moving and q.lat <= 3.5)) and not creep:
                # along 은 후륜축(기준점) 기준이다. 앞 범퍼까지 3.8 m 를 빼지 않으면
                # 실제 여유가 1.2 m 밖에 안 남는다 (정지선 계산과 같은 보정).
                d_ped = q.along - self.veh.rear_to_front          # 앞 범퍼 기준 거리
                v_t = min(v_t, self.stop_speed(max(0.0, d_ped - c.ped_gap), c.a_brake_ped))
                if d_ped < c.ped_stop_zone:
                    v_t = 0.0                                    # 완전 정지, 지나갈 때까지
                    ped_blocking = True
                    self.notes.append(f"보행자 정지 {d_ped:.1f}m(측 {q.lat:.1f}m)")
                else:
                    self.notes.append(f"보행자 감속 {d_ped:.1f}m(측 {q.lat:.1f}m)")
            elif q.lat <= 8.0:
                # 진로 밖에 서 있거나 멀어지는 중: 갑자기 들어와도 설 수 있게 낮춰만 둔다.
                # creep 중이면 더 낮춰(2 m/s) 사람 옆을 기어서 지난다.
                v_t = min(v_t, 2.0 if creep else 6.0)
                self.notes.append(f"보행자 {'서행통과' if creep else '주의'} {q.lat:.1f}m")
        if ped_blocking and v < 0.5:
            self.ped_stall_s += c.dt
            if self.ped_stall_s > c.ped_stall_s and self.ped_stall_s - c.dt <= c.ped_stall_s:
                self._event(f"보행자 정지 {c.ped_stall_s:.0f}s 초과 — 서행 통과로 전환 "
                            f"(진로 밖에 정지한 보행자로 판단)")
        elif not ped_blocking:
            if self.ped_stall_s > 0.0:
                self.ped_stall_s = 0.0

        # ---- 종료 ----
        if remaining < 40.0 and self.state in ("DRIVE", "RECOVER"):
            self.state = "FINISHING"
            self._event("FINISHING: 종료선 탐색")
        if self.state == "FINISHING":
            if self.finish_line is None:
                line = find_cone_line(rels)
                if line is not None:
                    self.finish_line = line
                    mid = ((line[0][0] + line[1][0]) / 2, (line[0][1] + line[1][1]) / 2)
                    k2 = r.nearest(mid[0], mid[1], max(0, i - 5), 60)
                    self.goal_cum = r.cum_xy(mid[0], mid[1], k2) + 1.5
                    self.finish_side = side_of_line(line, x, y)
                    self._event(f"라바콘 종료선 검출 → 목표 누적거리 {self.goal_cum:.1f} m")
            d_goal = self.goal_cum - self.cum
            if d_goal < 20.0:
                v_t = min(v_t, c.v_finish)
            v_t = min(v_t, self.stop_speed(max(0.0, d_goal)))
            passed = False
            if self.finish_line is not None and self.finish_side is not None:
                s_now = side_of_line(self.finish_line, x, y)
                passed = (s_now * self.finish_side < 0) and d_goal < 3.0
            if passed or (d_goal < 0.6 and v < 0.25):
                self.state = "DONE"
                self._event(f"완주 (goal {self.goal_cum:.1f} m, 리스폰 {self.respawns}회, {self.t:.0f}s)")

        # ---- 회피 지시등: 횡으로 움직이는 동안 켠다 (도로교통법 38조) ----
        d_off = self.avoid - self.avoid_prev
        if signal == TURN_OFF and abs(d_off) > 1e-3:
            signal = TURN_LEFT if d_off > 0 else TURN_RIGHT

        # ---- 차로변경: 지시등 + 시작/완료 로그 ----
        # 12 m 앞만 보면 이미 차로를 넘는 중에 켜진다. signal_lead(30 m, 도로교통법 38조)
        # 만큼 앞을 보고 미리 켠 뒤, 실제로 목표 차로에 들어갈 때까지 유지한다.
        j2 = r.index_ahead(i, c.signal_lead)
        lane_ahead = r.p[j2].lane if r.p[j2].road == r.p[i].road else r.p[i].lane
        if self.lane_now is not None and r.p[i].lane != self.lane_now:
            self._event(f"차로변경 완료: {self.lane_now} → {r.p[i].lane} (e={e_lat:+.2f} m, v={v:.1f})")
            self.lane_changing = False
        self.lane_now = r.p[i].lane
        if lane_ahead != r.p[i].lane:
            if not self.lane_changing:
                d_lc = r.cum[j2] - self.cum
                self._event(f"차로변경 시작: {r.p[i].lane} → {lane_ahead} ({d_lc:.0f} m 앞)")
                self.lane_changing = True
            if signal == TURN_OFF:
                a, b = r.p[i].lane, lane_ahead
                signal = TURN_LEFT if abs(b) < abs(a) else TURN_RIGHT   # 참조선 쪽(안쪽) = 좌
        elif self.lane_changing and lane_ahead == r.p[i].lane:
            self.lane_changing = False

        # ---- 워치독 ----
        self.watchdog(v_t)
        if self.push_left > 0.0:
            self.push_left -= c.dt
            hard_ok = (lead is None or lead[0] > 4.0) and \
                      (hard_stop_cum is None or hard_stop_cum - self.cum > 2.0)
            if hard_ok:
                v_t = max(v_t, 1.5)
                self.notes.append("워치독 전진")

        # ---- 가속도 명령 ----
        v_t = max(0.0, min(v_t, self.cfg.speed_limit))
        accel = c.kp_v * (v_t - v)
        if v * v > 2.0 * c.a_brake * max(self.stop_dist, 0.0) and v > 0.3:
            # 프로파일보다 빠르다 = 지금 감속해야 한다. P 제어만 쓰면 a_brake/kp_v 만큼
            # 뒤처져 따라가 정지선을 넘으므로, 실제로 필요한 감속도를 직접 명령한다.
            accel = min(accel, -(v * v) / (2.0 * max(self.stop_dist, 0.05)))
        if v_t < 0.1 and v < 0.6:
            accel = -2.0
        accel = max(c.accel_min, min(c.accel_max, accel))
        if v > self.cfg.speed_limit:
            accel = min(accel, -1.0)                 # 제한 50 km/h 초과는 즉시 잡는다 (감점 항목)
        # 저크 제한: 급가감속을 막는다. 단 '지금 세워야 하는' 강한 제동은 예외로 둔다
        # (보행자·정지선 안전 기능을 약화시키지 않는다).
        if accel > -c.a_brake_hard:
            lim = JERK_MAX * c.dt
            accel = max(self.accel_prev - lim, min(self.accel_prev + lim, accel))
        self.accel_prev = accel

        # 정지 이유: 전이될 때만 남긴다 (매 프레임 로그는 읽을 수 없다)
        reason = ""
        if v_t < 0.3:
            for key in ("보행자", "정지선", "선행차", "콘", "비신호", "종료", "회피", "점멸"):
                hit = next((n for n in self.notes if key in n), None)
                if hit:
                    reason = hit
                    break
            reason = reason or self.state
        # 전이 판정은 '종류'로 한다 — 거리 숫자까지 비교하면 매 프레임 새 이유가 된다
        kind = reason.split("(")[0].split()[0] if reason else ""
        if kind != self.stop_reason:
            if reason:
                self._event(f"정지 판단: {reason} (v={v:.1f})")
            elif self.stop_reason:
                self._event(f"정지 해제: {self.stop_reason} → 출발 (v={v:.1f})")
            self.stop_reason = kind
        sig = {0: " ", 1: "◀", 2: "▶"}[signal]
        self.status = (f"{self.state:9s} idx {i}/{r.n} v={v:4.1f}→{v_t:4.1f} e={e_lat:+.2f} "
                       f"steer={steer:+.2f} {sig} " + " ".join(self.notes))
        return steer, accel, signal, e_lat
