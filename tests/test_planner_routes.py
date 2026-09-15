"""경로계획 연속성 테스트 — 예제 + 사전테스트 2종.

실행: cd controller && python3 -m unittest tests.test_planner_routes -v
"""
from __future__ import annotations

import math
import unittest
from pathlib import Path

from hlfma.config import DEFAULT_XODR
from hlfma.odr import RoadNetwork
from hlfma.planner import RoutePlanner, load_waypoints, path_length, MAX_GAP, MAX_TURN_DEG, on_drivable

ROOT = Path(__file__).resolve().parent.parent.parent      # ~/Documents/HL_FMA
ROUTES = {
    "route_example": ROOT / "route_example.csv",
    "pretest": ROOT / "redmine/attachments/news29_route_pretest.csv",
    "pretest_2": ROOT / "redmine/attachments/news29_route_pretest_2.csv",
}
# tools/sweep_routes.py 가 찾아낸 실패 경로. 근본원인 1건당 1개만 승격한다.
ROUTES.update({f.stem: f for f in sorted((Path(__file__).parent / "fixtures").glob("*.csv"))})


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class PlannerRoutes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.net = RoadNetwork(DEFAULT_XODR)
        cls.rp = RoutePlanner(cls.net)

    def _check(self, name: str):
        wps = load_waypoints(str(ROUTES[name]))
        path = self.rp.plan(wps, verbose=False)
        straight = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(wps, wps[1:]))
        length = path_length(path)
        gaps = [math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(path, path[1:])]
        turns = [math.degrees(abs(wrap(b.heading - a.heading))) for a, b in zip(path, path[1:])]
        self.assertGreater(len(path), 50, name)
        self.assertLessEqual(max(gaps), MAX_GAP + 0.2, f"{name}: 최대 간격 {max(gaps):.2f}")
        self.assertGreaterEqual(min(gaps), 0.3, f"{name}: 중복 점")
        self.assertLessEqual(max(turns), MAX_TURN_DEG, f"{name}: heading 점프 {max(turns):.1f}°")
        self.assertLessEqual(length, 1.15 * straight + 40, f"{name}: 우회 {length:.0f} vs 직선 {straight:.0f}")
        # 각 점이 자기 도로의 같은 방향 주행영역(차로 합집합) 안에 있는지 — 반대차로/보도 침범 검출
        # 교차로 연결도로는 차로가 좁고(≈1.9 m) 이음새 보정으로 0.5 m 안팎 바깥으로 갈 수 있어 0.8 m 허용
        off = [p for p in path if not on_drivable(self.net, p, tol=0.8)]
        self.assertLessEqual(len(off), 2, f"{name}: 주행영역 밖 점 {len(off)}/{len(path)} "
                                          f"{[(p.road, round(p.s,1)) for p in off[:5]]}")
        # 교차로 진입마다 접근로에 정지선이 있는지 (보고용)
        entries = [i for i in range(1, len(path)) if path[i].in_junction and not path[i - 1].in_junction]
        missing = [path[i - 1].road for i in entries if not self.net.stop_lines(path[i - 1].road)]
        print(f"\n  {name}: {len(path)}점 {length:.0f} m (직선 {straight:.0f}), 교차로 {len(entries)}개, "
              f"정지선 없는 접근로 {missing}")

    def test_regress_curvature_1139(self):
        """급커브 교차로 연결로에서 스텝당 회전이 MAX_TURN_DEG 를 넘던 사례."""
        self._check("regress_curvature_1139")

    def test_route_example(self):
        self._check("route_example")

    def test_pretest(self):
        self._check("pretest")

    def test_pretest_2(self):
        self._check("pretest_2")


if __name__ == "__main__":
    unittest.main()
