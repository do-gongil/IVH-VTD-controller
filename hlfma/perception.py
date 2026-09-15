"""GT objects[] 해석 — 분류, 경로 기준 상대 위치, 선행차/보행자/콘 종료선.

객체 타입은 오지 않는다(공지). 크기·속도로 분류한다:
  콘(라바콘)  : height < 1.0 and length < 1.0
  보행자      : height >= 1.2 and length < 1.5
  차량        : 그 외
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def is_marker(ob: dict) -> bool:
    """진짜 라바콘(종료선 표시용). 실측 0.15 x 0.46 x 0.61."""
    return ob["height"] < 1.0 and ob["length"] < 1.0


def classify(ob: dict) -> str:
    if is_marker(ob):
        return "cone"
    # 정지한 소형 물체 = 정적 장애물. 콘과 같은 취급(미리 비켜가고, 못 비키면 서행 통과)
    # 을 받아야 한다. car 로 두면 8 s 대기 후 추월 판정으로 가는데, 장애물 자신이
    # _side_clear 를 막아 영구 정차가 된다.
    # 구루마(WheelBarrow01) 1.01 x 2.06 x 0.85 가 기준. 보행자(h 1.8)·승용차(l 4.4+)는 제외.
    if ob["speed"] < 0.3 and ob["length"] <= 2.5 and ob["height"] < 1.4:
        return "cone"
    if ob["height"] >= 1.2 and ob["length"] < 1.5:
        return "ped"
    return "car"


@dataclass
class Rel:
    ob: dict
    kind: str
    along: float      # 경로 진행 방향 거리 [m] (음수 = 뒤)
    lat: float        # 경로 중심선에서 횡거리 [m] (절대값)
    lat_s: float      # 같은 값, 부호 있음 (경로 좌측이 +)
    idx: int          # 가장 가까운 경로 인덱스
    closing: float    # 경로 쪽으로 다가오는 속도 성분 [m/s] (양수 = 접근)


def relate(route, i: int, cum_now: float, objs: list[dict], ahead: float = 80.0) -> list[Rel]:
    """objects 를 경로 기준 (along, lat) 로. route 는 hlfma.route.Route.
    cum_now 는 자차의 연속 누적거리 (route.cum[i] 는 2 m 양자화라 거리 판정이 거칠어진다)."""
    out = []
    for ob in objs:
        k = route.nearest(ob["x"], ob["y"], max(0, i - 10), int(ahead / 2.0) + 20)
        p = route.p[k]
        along = route.cum[k] - cum_now
        dx, dy = ob["x"] - p.x, ob["y"] - p.y
        # 경로 좌측이 양(+)
        lat_signed = -math.sin(p.heading) * dx + math.cos(p.heading) * dy
        lat = abs(lat_signed)
        # 접근 속도: 객체 속도벡터의 (경로 중심 방향) 성분
        vx, vy = ob["speed"] * math.cos(ob["heading"]), ob["speed"] * math.sin(ob["heading"])
        toward = (-dx, -dy)
        n = math.hypot(*toward) or 1.0
        closing = (vx * toward[0] + vy * toward[1]) / n
        out.append(Rel(ob, classify(ob), along, lat, lat_signed, k, closing))
    return out


def lead_vehicle(rels: list[Rel], lat_max: float = 1.7, ahead: float = 70.0, rear_to_front: float = 3.808):
    """내 경로 위 가장 가까운 전방 차량 → (gap[m], Rel). gap 은 범퍼 간 거리."""
    best = None
    for r in rels:
        if r.kind != "car" or r.along < -1.0 or r.along > ahead or r.lat > lat_max:
            continue
        gap = r.along - r.ob["length"] / 2.0 - rear_to_front
        if best is None or gap < best[0]:
            best = (gap, r)
    return best


def pedestrians(rels: list[Rel], ahead: float = 60.0) -> list[Rel]:
    """전방 보행자. ahead 는 정지거리보다 넉넉해야 한다 — 45 km/h 의 정지거리가
    37 m 라 예전 기본값 30 m 로는 보이는 순간 이미 늦는다. RDB 는 80 m 까지 준다."""
    return [r for r in rels if r.kind == "ped" and -2.0 <= r.along <= ahead]


def blocking_cones(rels: list[Rel], half_width: float = 0.943, ahead: float = 60.0,
                   margin: float = 0.4, offset: float = 0.0) -> list[Rel]:
    """내 차로를 막고 선 라바콘. 종료선 콘(양옆에 하나씩)은 제외한다.

    콘은 종료선 표시로도 쓰여 사이로 통과해야 하므로 무조건 세우면 완주를 못 한다.
    구분 기준은 **쌍**이다: 종방향 8 m 안에 반대편(lat 부호 반대) 콘이 있으면 게이트로
    보고 통과, 없으면 차로를 막은 장애물로 본다.
    """
    cones = [r for r in rels if r.kind == "cone" and -1.0 <= r.along <= ahead]
    out = []
    for c in cones:
        side = c.lat_s
        if abs(side - offset) > half_width + c.ob["width"] / 2.0 + margin:
            continue                                   # 차폭 밖 — 스치지 않는다
        paired = any(o is not c and abs(o.along - c.along) < 8.0
                     and o.lat_s * side < 0 for o in cones)
        if not paired:
            out.append(c)
    return out



def find_cone_line(rels: list[Rel], within: float = 35.0):
    """종료선: 정지된 콘 2개, 간격 2~8 m, 전방 within m 안. → ((x1,y1),(x2,y2)) 또는 None."""
    cones = [r for r in rels if is_marker(r.ob) and abs(r.ob["speed"]) < 0.3 and -5.0 <= r.along <= within]
    best = None
    for a in range(len(cones)):
        for b in range(a + 1, len(cones)):
            oa, ob_ = cones[a].ob, cones[b].ob
            d = math.hypot(oa["x"] - ob_["x"], oa["y"] - ob_["y"])
            if 2.0 <= d <= 8.0:
                mid_along = (cones[a].along + cones[b].along) / 2.0
                if best is None or abs(mid_along) < abs(best[0]):
                    best = (mid_along, ((oa["x"], oa["y"]), (ob_["x"], ob_["y"])))
    return best[1] if best else None


def side_of_line(line, x: float, y: float) -> float:
    """선분 (p1→p2) 기준 점의 부호 있는 거리. 통과 판정은 부호 변화로."""
    (x1, y1), (x2, y2) = line
    dx, dy = x2 - x1, y2 - y1
    n = math.hypot(dx, dy) or 1.0
    return ((x - x1) * dy - (y - y1) * dx) / n
