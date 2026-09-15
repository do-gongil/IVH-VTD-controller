"""정지 정밀도 자체점검 — 정지선 침범과 종료 미판정 회귀 방지.

경로 누적거리를 2 m 간격 인덱스(r.cum[idx])로만 알면 (1) 정지선을 ~2 m 넘고
(2) 마지막 인덱스에서 포화해 완주 판정이 서지 않는다. 둘 다 실차에서 관측됐다.

실행: cd controller && python3 -m unittest tests.test_controller_stop -v
"""
from __future__ import annotations

import math
import unittest

from hlfma.config import RunConfig
from hlfma.odr import RoadNetwork
from hlfma.planner import RoutePlanner, load_waypoints
from hlfma.route import Route
from hlfma.controller import Controller
from hlfma.hlvtd import TL_RED, TL_GREEN, TL_YELLOW

ROUTE = "../redmine/attachments/news29_route_pretest.csv"


def obj_at(r: Route, cum: float, lat: float, size, speed: float = 0.0,
           oid: int = 800, heading_off: float = 0.0):
    """경로 cum·횡거리 lat 지점의 객체 하나 (패킷 objects[] 형식). size=(L,W,H)."""
    k = r.index_at_cum(max(0.0, min(cum, r.length)))
    p = r.p[k]
    ds = cum - r.cum[k]
    nx, ny = -math.sin(p.heading), math.cos(p.heading)      # 경로 좌측
    x = p.x + math.cos(p.heading) * ds + nx * lat
    y = p.y + math.sin(p.heading) * ds + ny * lat
    L, W, H = size
    return {"slot": 0, "id": oid, "x": x, "y": y, "z": 0.0,
            "heading": p.heading + heading_off, "speed": speed,
            "length": L, "width": W, "height": H}


CAR = (4.848, 1.886, 1.507)
PED = (0.600, 0.700, 1.800)


def cone_pair(r: Route, cum: float, gap: float = 5.0):
    """경로 cum 지점에 좌우로 gap 만큼 벌어진 라바콘 2개 (패킷 객체 형식)."""
    k = r.index_at_cum(min(cum, r.length))
    p = r.p[k]
    ds = cum - r.cum[k]
    bx, by = p.x + math.cos(p.heading) * ds, p.y + math.sin(p.heading) * ds
    nx, ny = -math.sin(p.heading), math.cos(p.heading)      # 경로 좌측 단위벡터
    out = []
    for sgn, oid in ((+1, 901), (-1, 902)):
        out.append({"slot": oid - 900, "id": oid,
                    "x": bx + nx * sgn * gap / 2, "y": by + ny * sgn * gap / 2, "z": 0.0,
                    "heading": p.heading, "speed": 0.0,
                    "length": 0.3, "width": 0.3, "height": 0.32})   # RdMiscPylon03-32cm
    return out


def drive(ctl: Controller, r: Route, tl_state: int, max_s: float = 400.0, objs_at=None):
    """계획 경로를 그대로 따라가는 이상적 차량으로 제어기를 돌린다 (조향은 완벽하다고 가정).
    후륜축 누적거리 s 를 accel 명령으로 적분한다. → (정지한 누적거리, DONE 여부)"""
    cfg, c = ctl.cfg, ctl.c
    s, v = 0.0, 0.0
    for _ in range(int(max_s / c.dt)):
        k = r.index_at_cum(min(s, r.length))
        p = r.p[k]
        # s 를 정확히 재현하는 (x, y): 경로 점에서 접선방향으로 나머지를 밀어준다
        ds = s - r.cum[k]
        x = p.x + math.cos(p.heading) * ds
        y = p.y + math.sin(p.heading) * ds
        tl = (159, tl_state)
        objs = objs_at(s) if objs_at else []
        _steer, accel, _sig, _e = ctl.step(x, y, p.heading, tl, objs)
        ctl.v = v                       # 위치추정 대신 참값 속도를 준다
        v = max(0.0, v + accel * c.dt)
        s += v * c.dt
        if ctl.state == "DONE":
            return s, True
        if v < 1e-3 and ctl.state == "TL_STOP":
            return s, False
    return s, False


class StopAccuracy(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cfg = RunConfig.load("hlfma.json")
        net = RoadNetwork(cfg.xodr)
        path = RoutePlanner(net).plan(load_waypoints(ROUTE), verbose=False)
        cls.cfg, cls.net, cls.path = cfg, net, path

    def _route(self):
        return Route(self.path, self.net, self.cfg.vehicle, self.cfg.control, self.cfg.speed_zone30)

    def test_stop_line_not_crossed(self):
        """적색에 정지선을 넘지 않고, 규정 여유(2 m) 안에 선다."""
        r = self._route()
        ctl = Controller(r, self.cfg)
        s, _done = drive(ctl, r, TL_RED)
        J = r.junctions[0]
        line = r.cum_at(J.road, J.s_line)
        bumper = s + self.cfg.vehicle.rear_to_front
        gap = line - bumper
        self.assertGreaterEqual(gap, 0.0, f"정지선 {-gap:.2f} m 침범 (범퍼 {bumper:.2f} > 정지선 {line:.2f})")
        self.assertLessEqual(gap, 2.0, f"정지선에서 {gap:.2f} m 미달 (규정 2 m 이내)")

    def test_yellow_commit_is_not_reversed(self):
        """정지불가로 황색 커밋한 뒤에는 정지선을 넘어도 결정을 뒤집지 않는다.

        뒤집히면 8 m/s 로 정지선을 넘은 차가 급제동해 교차로 한복판에 선다 (실차 관측).
        """
        r = self._route()
        ctl = Controller(r, self.cfg)
        J = r.junctions[0]
        # 정지불가 거리까지 접근한 상태를 만든다: d_stop 이 v^2/6 보다 작아지는 지점
        v = 8.0
        s = J.stop_cum - (v * v / 6.0) * 0.5          # 커밋 영역 한가운데
        seen = []
        for _ in range(120):
            k = r.index_at_cum(min(s, r.length))
            p = r.p[k]
            ds = s - r.cum[k]
            x, y = p.x + math.cos(p.heading) * ds, p.y + math.sin(p.heading) * ds
            ctl.v = v
            _st, accel, _sg, _e = ctl.step(x, y, p.heading, (159, TL_YELLOW), [])
            seen.append(" ".join(ctl.notes))
            s += v * self.cfg.control.dt
            if s > J.entry_cum + 5.0:
                break
        committed = [n for n in seen if "커밋" in n]
        stopped = [n for n in seen if "정지선" in n]
        self.assertTrue(committed, "황색 커밋이 한 번도 서지 않았다")
        self.assertFalse(stopped, f"커밋 후 정지로 뒤집혔다: {stopped[:3]}")

    def test_stale_light_does_not_deadlock(self):
        """신호 정보가 끊긴(id=0) 채 래치만 남아도 대기 상한 뒤에는 진행한다."""
        r = self._route()
        ctl = Controller(r, self.cfg)
        J = r.junctions[0]
        J.tl_latched = TL_RED
        J.tl_wait_s = self.cfg.control.tl_unknown_wait_s + 1.0
        k = r.index_at_cum(J.stop_cum)
        p = r.p[k]
        ctl.step(p.x, p.y, p.heading, (0, 0), [])      # tl_id = 0 (정보 없음)
        self.assertEqual(J.tl_latched, -1, "끊긴 신호의 래치가 풀리지 않았다 → 영구 대기")

    def test_finish_reaches_done(self):
        """녹색으로 끝까지 가면 DONE 에 도달한다 (종료점에서 무한 선회하지 않는다)."""
        r = self._route()
        ctl = Controller(r, self.cfg)
        s, done = drive(ctl, r, TL_GREEN)
        self.assertTrue(done, f"완주 판정 실패: state={ctl.state} s={s:.1f}/{r.length:.1f}")
        self.assertLess(abs(s - ctl.goal_cum), 2.0, f"종료 위치 오차 {s - ctl.goal_cum:+.2f} m")


class ObjectAvoidance(unittest.TestCase):
    """선행차 추종·보행자 정지 — 실주행에서 한 번도 발동하지 않아 별도 검증한다."""

    @classmethod
    def setUpClass(cls):
        cfg = RunConfig.load("hlfma.json")
        net = RoadNetwork(cfg.xodr)
        cls.cfg, cls.net = cfg, net
        cls.path = RoutePlanner(net).plan(load_waypoints(ROUTE), verbose=False)

    def _route(self):
        return Route(self.path, self.net, self.cfg.vehicle, self.cfg.control, self.cfg.speed_zone30)

    def test_stops_behind_stopped_car(self):
        """내 차로에 선 앞차를 들이받지 않는다."""
        r = self._route()
        ctl = Controller(r, self.cfg)
        car_cum = 200.0
        s, _ = drive(ctl, r, TL_GREEN, max_s=120.0,
                     objs_at=lambda _s: [obj_at(r, car_cum, 0.0, CAR, 0.0)])
        gap = (car_cum - CAR[0] / 2.0) - (s + self.cfg.vehicle.rear_to_front)   # 범퍼 간격
        self.assertGreater(gap, 1.0, f"추돌 위험: 범퍼 간격 {gap:.2f} m")
        self.assertLess(gap, 20.0, f"앞차에서 {gap:.2f} m 나 떨어져 멈췄다 (과잉 정지)")

    def test_follows_moving_car(self):
        """주행 중인 앞차를 안전 간격으로 따라간다 (멈춰서지 않는다)."""
        r = self._route()
        ctl = Controller(r, self.cfg)
        lead = {"cum": 60.0}
        def objs(_s):
            lead["cum"] += 4.0 * self.cfg.control.dt        # 앞차 4 m/s 순항
            return [obj_at(r, lead["cum"], 0.0, CAR, 4.0)]
        s, _ = drive(ctl, r, TL_GREEN, max_s=90.0, objs_at=objs)
        gap = (lead["cum"] - CAR[0] / 2.0) - (s + self.cfg.vehicle.rear_to_front)
        self.assertGreater(gap, 0.0, f"추돌 (간격 {gap:.1f} m)")
        self.assertGreater(s, 100.0, f"앞차를 따라가지 못하고 {s:.1f} m 에서 정체")

    def test_stops_for_crossing_pedestrian(self):
        """경로를 막고 선 보행자 앞에서 여유를 두고 정지한다."""
        r = self._route()
        ctl = Controller(r, self.cfg)
        ped_cum = 150.0
        s, _ = drive(ctl, r, TL_GREEN, max_s=120.0,
                     objs_at=lambda _s: [obj_at(r, ped_cum, 0.0, PED, 0.0, oid=901)])
        # 여유는 후륜축이 아니라 '앞 범퍼' 기준으로 봐야 한다 (기준점이 후륜축이므로)
        clear = ped_cum - (s + self.cfg.vehicle.rear_to_front)
        self.assertGreater(clear, 1.5, f"보행자 앞 여유 {clear:.2f} m — 너무 가깝다")
        self.assertLess(clear, 12.0, f"보행자에서 {clear:.2f} m 나 떨어져 멈췄다 (과잉 정지)")

    def test_ignores_pedestrian_on_sidewalk(self):
        """차로 밖(정지·비접근) 보행자에는 서지 않는다 — 오검출은 완주 실패로 이어진다."""
        r = self._route()
        ctl = Controller(r, self.cfg)
        s, done = drive(ctl, r, TL_GREEN,
                        objs_at=lambda _s: [obj_at(r, 150.0, 4.0, PED, 0.0, oid=902)])
        self.assertTrue(done, f"인도 보행자 때문에 완주하지 못했다: {s:.1f} m, state={ctl.state}")


class ConeFinishLine(unittest.TestCase):
    """라바콘 종료선 검출 경로 (실주행 검증은 콘 미배치라 폴백 경로만 확인됨)."""

    @classmethod
    def setUpClass(cls):
        cfg = RunConfig.load("hlfma.json")
        net = RoadNetwork(cfg.xodr)
        cls.cfg, cls.net = cfg, net
        cls.path = RoutePlanner(net).plan(load_waypoints(ROUTE), verbose=False)

    def _route(self):
        return Route(self.path, self.net, self.cfg.vehicle, self.cfg.control, self.cfg.speed_zone30)

    def test_stops_at_cone_line(self):
        """콘 종료선이 마지막 경유지보다 앞에 있으면 그 선에서 멈춘다."""
        r = self._route()
        ctl = Controller(r, self.cfg)
        cone_cum = r.length - 12.0                      # 경로 끝보다 12 m 앞
        s, done = drive(ctl, r, TL_GREEN, objs_at=lambda _s: cone_pair(r, cone_cum))
        self.assertTrue(done, f"콘 종료선에서 완주 판정 실패: state={ctl.state}")
        self.assertIsNotNone(ctl.finish_line, "콘 종료선을 검출하지 못했다")
        # 콘 선 ±2 m 안에서 멈춰야 한다 (goal 은 선 + 1.5 m 로 잡힌다)
        # 콘 선 중점을 경로에 연속 투영하므로 goal 은 cone_cum + 1.5 와 거의 같아야 한다.
        # r.cum[k2] 를 쓰면 경로 샘플 간격(2 m)만큼 어긋난다.
        self.assertLess(abs(ctl.goal_cum - (cone_cum + 1.5)), 0.3,
                        f"목표 {ctl.goal_cum:.2f} m, 기대 {cone_cum + 1.5:.2f} m")
        # 완주 판정은 콘 선 통과(side_of_line 부호 변화)로 선다 → 선 직후에 멈춘다.
        self.assertGreaterEqual(s, cone_cum, f"콘 선({cone_cum:.2f} m)을 넘지 않고 {s:.2f} m 에서 멈췄다")
        self.assertLess(s - cone_cum, 2.0, f"콘 선을 {s - cone_cum:.2f} m 지나쳐 멈췄다")
        # 마지막 경유지(+2 m)가 아니라 콘 선을 썼는지 확인 — 12 m 차이가 나야 한다
        self.assertLess(s, r.length, f"콘을 무시하고 경로 끝({r.length:.1f} m)까지 갔다")

    def test_ignores_wrong_sized_objects(self):
        """차량·보행자는 종료선으로 오인하지 않는다."""
        r = self._route()
        ctl = Controller(r, self.cfg)
        def fake(_s):
            objs = cone_pair(r, r.length - 12.0)
            for o in objs:                              # 콘 크기를 보행자로 바꾼다
                o["length"], o["width"], o["height"] = 0.6, 0.7, 1.8
            return objs
        drive(ctl, r, TL_GREEN, objs_at=fake)
        self.assertIsNone(ctl.finish_line, "보행자 크기 객체를 종료선으로 오인했다")


if __name__ == "__main__":
    unittest.main()
