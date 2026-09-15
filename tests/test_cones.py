"""라바콘 판별: 종료선 게이트는 통과, 차로를 막은 콘은 정지.

실제 사고 데이터(v7 주행, cum 741 m 지점 콘 3개)를 그대로 넣어 회귀로 잡는다.
"""
from __future__ import annotations

import math
import unittest

from hlfma.perception import Rel, blocking_cones, classify

CONE = {"length": 0.15, "width": 0.46, "height": 0.61, "speed": 0.0, "heading": 0.0}
HALF_W = 0.943


def rel(along: float, lat_s: float, ob: dict | None = None) -> Rel:
    o = dict(ob or CONE, x=0.0, y=0.0)
    return Rel(o, classify(o), along, abs(lat_s), lat_s, 0, 0.0)


class TestBlockingCones(unittest.TestCase):
    def test_classify_cone(self):
        self.assertEqual(classify(dict(CONE, x=0.0, y=0.0)), "cone")

    def test_finish_gate_is_passable(self):
        """종료선: 경로 양옆 하나씩 (간격 2.4 m) → 막힌 것으로 보면 완주 못 한다."""
        gate = [rel(20.0, +1.2), rel(20.0, -1.2)]
        self.assertEqual(blocking_cones(gate, HALF_W), [])

    def test_narrow_gate_still_passable(self):
        """규격 최소 간격 2.0 m 게이트도 통과 대상이어야 한다."""
        gate = [rel(15.0, +1.0), rel(15.0, -1.0)]
        self.assertEqual(blocking_cones(gate, HALF_W), [])

    def test_lone_cone_in_lane_blocks(self):
        """실제 사고: 경로 중심 -0.22 m 에 홀로 선 콘."""
        got = blocking_cones([rel(12.0, -0.22)], HALF_W)
        self.assertEqual(len(got), 1)

    def test_same_side_cluster_blocks(self):
        """v7 실측: lat -0.22 / -1.27 / -2.67, 전부 같은 쪽 → 게이트가 아니다."""
        cones = [rel(12.0, -0.22), rel(13.7, -1.27), rel(12.0, -2.67)]
        got = blocking_cones(cones, HALF_W)
        # -0.22 는 정면. -1.27 은 차량 반폭 0.943 + 콘 반폭 0.23 을 빼면 여유 10 cm
        # 뿐이라 함께 잡는다. -2.67 은 충분히 벗어나 무시한다.
        self.assertEqual([round(c.lat_s, 2) for c in got], [-0.22, -1.27])

    def test_cone_outside_vehicle_width_ignored(self):
        self.assertEqual(blocking_cones([rel(12.0, +2.0)], HALF_W), [])

    def test_cone_behind_ignored(self):
        self.assertEqual(blocking_cones([rel(-5.0, 0.0)], HALF_W), [])

    def test_stop_distance_is_reachable(self):
        """40 m 앞에서 잡으면 45 km/h 에서도 a_brake 2.5 로 설 수 있어야 한다."""
        from hlfma.config import RunConfig
        cfg = RunConfig()
        v, c = cfg.cruise, cfg.control
        need = v * v / (2 * c.a_brake) + cfg.vehicle.rear_to_front + 1.0
        self.assertLess(need, 40.0, f"콘 탐지거리 40 m 로는 {need:.1f} m 가 필요해 못 선다")


if __name__ == "__main__":
    unittest.main()
