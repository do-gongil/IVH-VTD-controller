"""정적 장애물(구루마) 회피 관련 회귀 — 실제로 물렸던 것들만.

대회 영상(fma.mp4)에서 정적 장애물이 라바콘이 아니라 구루마(WheelBarrow01,
1.01 x 2.06 x 0.85)임을 확인했고, 채점표에 '정적 장애물 충돌' 과
'실선 차로변경 금지' 가 있다.
"""
from __future__ import annotations

import unittest

from hlfma.config import RunConfig
from hlfma.controller import AVOID_MARGIN, CONE_HALF
from hlfma.odr import RoadNetwork
from hlfma.perception import blocking_cones, classify, is_marker

BARROW = {"length": 1.01, "width": 2.06, "height": 0.85, "speed": 0.0}   # WheelBarrow01 실측
PYLON = {"length": 0.15, "width": 0.46, "height": 0.61, "speed": 0.0}    # 라바콘 실측
PED = {"length": 0.60, "width": 0.70, "height": 1.80, "speed": 0.0}      # 보행자 실측(정지)
CAR = {"length": 4.85, "width": 1.89, "height": 1.51, "speed": 0.0}      # 아이오닉6 실측(정차)


class TestClassify(unittest.TestCase):
    def test_barrow_is_static_obstacle(self):
        """구루마는 길이 1.01 m 라 옛 콘 조건(<1.0)을 0.01 m 차이로 못 넘겼다 →
        car 로 분류돼 8 s 대기 후 추월 판정으로 갔고, 장애물 자신이 옆자리를 막아
        영구 정차가 됐다. 정적 장애물로 분류돼야 미리 비켜간다."""
        self.assertEqual(classify(BARROW), "cone")

    def test_standing_pedestrian_is_not_obstacle(self):
        """서 있는 보행자를 정적 장애물로 보면 2 m/s 로 들이받고 지나간다."""
        self.assertEqual(classify(PED), "ped")

    def test_stopped_car_is_not_obstacle(self):
        self.assertEqual(classify(CAR), "car")

    def test_only_real_cones_mark_the_finish(self):
        """종료선 판정은 진짜 라바콘만. 구루마가 콘으로 분류돼도 짝을 이루면 안 된다."""
        self.assertTrue(is_marker(PYLON))
        self.assertFalse(is_marker(BARROW))


class TestAvoidMargin(unittest.TestCase):
    def test_avoid_margin_clears_blocking_margin(self):
        """회피 여유가 '막고 있음' 판정 여유(0.4)보다 작으면, 오프셋을 다 넣은 뒤에도
        같은 장애물이 계속 막은 것으로 잡혀 2 m/s 서행이 안 풀린다 (실측)."""
        self.assertGreater(AVOID_MARGIN, 0.4)

    def test_barrow_width_used_not_cone_half(self):
        """라바콘 반폭(0.23)으로 비키면 구루마(반폭 1.03) 를 스친다."""
        self.assertGreater(BARROW["width"] / 2.0, CONE_HALF)

    def test_offset_clears_barrow(self):
        """계산한 오프셋으로 비키면 더 이상 '막고 있음' 이 아니어야 한다."""
        from hlfma.perception import Rel
        half = RunConfig().vehicle.obb_half_width
        ob = dict(BARROW, x=0.0, y=0.0, heading=0.0)
        rel = Rel(ob, classify(ob), 30.0, 0.0, 0.0, 0, 0.0)
        self.assertEqual(len(blocking_cones([rel], half)), 1)          # 안 비키면 막힌다
        off = half + max(CONE_HALF, ob["width"] / 2.0) + AVOID_MARGIN
        self.assertEqual(blocking_cones([rel], half, offset=-off), [])  # 비키면 풀린다


class TestBoundaryMark(unittest.TestCase):
    """실선 차로변경 금지 판정의 근거 데이터."""

    @classmethod
    def setUpClass(cls):
        cls.net = RoadNetwork(RunConfig().xodr)

    def test_reads_real_map(self):
        rd = self.net.roads["429"]
        self.assertEqual(rd.boundary_mark(60.0, 2, 3), "broken")
        self.assertEqual(rd.boundary_mark(120.0, 2, 3), "solid")

    def test_opposite_direction_is_solid(self):
        """반대방향 차로 사이는 중앙선 — 넘으면 중앙선 침범이다."""
        rd = self.net.roads["429"]
        self.assertEqual(rd.boundary_mark(60.0, 1, -1), "solid")


if __name__ == "__main__":
    unittest.main()
