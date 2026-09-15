"""기준 시나리오에 NPC 차량·보행자를 추가한 새 시나리오를 만든다.

  python3 tools/make_scenario.py <경로.csv> [출력파일명]

객체를 절대좌표가 아니라 **계획 경로 위 누적거리(cum)** 로 지정한다. 경로가 바뀌어도
아래 표의 숫자를 그대로 쓸 수 있고, 차로 중심에 정확히 놓인다.
기준 시나리오(BASE)의 기존 객체(Oncoming1/Npc1/Npc2/Ped1)는 건드리지 않는다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from hlfma.config import RunConfig
from hlfma.odr import RoadNetwork
from hlfma.planner import RoutePlanner, load_waypoints
from hlfma.route import Route

VTD_ROOT = Path("/home/dap/Hexagon/VTD.2025.2")
SCEN_DIR = VTD_ROOT / "Data/Projects/SampleProject/Scenarios"
BASE = SCEN_DIR / "HL_FMA_VTD_LivingLab_news29_route_pretest_2_test.xml"

# ---------------------------------------------------------------- 배치표
# 모든 NPC 는 자차가 발동반경 안에 들어왔을 때 움직이기 시작한다. 시작하자마자
# 달리게 하면 자차가 도착할 즈음엔 이미 멀리 가 있어 만나지 않는다.

# 선행차: 경로 위 cum[m] 에 차로 중심으로 놓고, 목표속도로 자율주행시킨다.
#         자차 순항 8 m/s 보다 느려야 따라붙어 추종 로직이 걸린다.
CARS = [
    # 이름,     cum,   횡오프셋, 목표속도, 발동반경
    ("Lead1",   430.0,  0.0,     4.0,      50.0),
]

# 교차 차량: 교차 도로에서 교차로를 향해 접근 → 교차로 양보/서행 분기 시험.
# 교차 도로는 `--list` 로 확인한다(경로에 포함되지 않은 incoming 도로여야 한다).
# 발동반경은 자차~교차로 거리 기준이다. 반경/자차속도 ≈ 교차로까지거리/NPC속도 로 맞춘다.
CROSS_CARS = [
    # 이름,      교차로 인덱스, 교차 도로, 교차로까지 거리, 목표속도, 발동반경
    ("Cross1",   3,             "128",     25.0,            6.0,      60.0),
    ("Cross2",   2,             "174",     20.0,            5.0,      75.0),
]

# 보행자: cum 지점에서 도로를 가로지른다. 시작/끝 횡거리는 **도로 가장자리에서 자동**으로
#         잡는다 — 도로 폭이 지점마다 달라서(Ped1 지점은 10 m, Ped3 지점은 20 m) 고정값을
#         쓰면 인도가 아니라 차로 한복판에서 시작하거나 다 건너기 전에 멈춘다.
#         side = -1 이면 경로 우측(인도)에서 출발, +1 이면 좌측에서 출발.
# 기준 시나리오의 Ped1 도 여기서 다시 만든다. MovingObjectsControl 을 통째로 새로 쓰기
# 때문이다(기준 파일의 보행자 블록은 PathShape 순서·Weight 누락으로 걷지 않는다).
PEDS = [
    # 이름,   cum,   side, 보행속도, 발동반경(None = 자동)
    ("Ped1",  311.0, -1,   1.4,      None),
    ("Ped2",  480.0, -1,   1.4,      None),
    # cum 660 은 편도 25 m 짜리 넓은 도로다. 좌측에서 출발시키면 우리 차로까지 19 s 가
    # 걸려 자차가 먼저 지나간다 → 우측(-1) 출발.
    ("Ped3",  660.0, -1,   1.4,      None),
]

# --stress : 보행자·차량을 촘촘히 깐 시험용. 경로 752 m 를 대략 80 m 간격으로 채운다.
# 대회 본선보다 훨씬 빡세게 잡아 제어기 한계를 본다 (완주 실패해도 정보가 남는다).
STRESS_CARS = [
    ("SLead1",  150.0, 0.0, 3.5, 60.0),
    ("SLead2",  300.0, 0.0, 4.0, 60.0),
    ("SLead3",  560.0, 0.0, 3.0, 60.0),
    ("SLead4",  700.0, 0.0, 4.0, 60.0),
]
STRESS_CROSS = [
    ("SCross0", 0, None, 22.0, 6.0, 60.0),
    ("SCross1", 1, None, 22.0, 5.0, 70.0),
]
STRESS_PEDS = [
    # SPed1(cum 120) 제거: 왕복 모드에서 자차 출발 직후 구간에 계속 걸쳐 있어
    #   차가 출발하자마자 멈춰 서는 게 반복됐다. 시험 가치보다 방해가 크다.
    # SPed2(cum 250) 제거: 첫 교차로 출구 한복판을 가로지른다 — 횡단보도가 아니라
    #   교차로 안이라 현실성이 없다.
    ("SPed3",  400.0, -1, 1.6, None),
    ("SPed4",  545.0, -1, 1.4, None),
    ("SPed5",  620.0, -1, 1.3, None),
    ("SPed6",  720.0, -1, 1.5, None),
]

PED_WAYPOINTS = 5          # 스플라인은 3 점이면 자주 뭉갠다
PED_MARGIN = 2.0           # 도로 가장자리에서 이만큼 밖(인도)에서 출발/도착
PED_MAX_CROSS_M = 16.0     # 이보다 넓은 도로를 가로지르는 보행자는 만들지 않는다.
                           # 편도 25 m 간선을 무단횡단하는 사람은 현실에 없고,
                           # 그런 배치는 제어기를 시험하는 게 아니라 괴롭힐 뿐이다.
EGO_SPEED = 12.5           # 발동반경 자동계산용 자차 접근속도 [m/s] — cfg.cruise 와 맞춘다

CAR_TYPE = "HyundaiIoniq6_23_White"
PED_TYPE = ("male_adult", "Christian")


def e(v: float) -> str:
    """VTD 시나리오의 숫자 표기(부동소수 지수형)."""
    return "%.6e" % v


def lat_point(p, lat: float) -> tuple[float, float]:
    """경로점 p 에서 횡거리 lat 만큼 떨어진 좌표. 좌측이 +(perception.py 와 같은 부호)."""
    return p.x - lat * math.sin(p.heading), p.y + lat * math.cos(p.heading)


def road_edges(net: RoadNetwork, p, reach: float = 25.0, step: float = 0.5) -> tuple[float, float]:
    """경로점 p 에서 좌·우로 훑어 주행 차로가 끊기는 횡거리 → (우측 edge, 좌측 edge).

    차로 수가 지점마다 다르므로(편도 1차로 ~ 편도 5차로) 실측해야 한다.
    """
    out = []
    for sign in (-1.0, +1.0):
        edge = 0.0
        lat = 0.0
        while lat < reach:
            lat += step
            x, y = lat_point(p, sign * lat)
            if net.project_lane(x, y) is None:
                break
            edge = sign * lat
        out.append(edge)
    return out[0], out[1]


def zof(net: RoadNetwork, x: float, y: float, fallback: float) -> float:
    """절대 고도. 도로에서 멀면(인도 바깥) 조회가 실패하므로 반드시 지면값을 폴백으로 준다 —
    0 을 넣으면 이 맵(지면 0~79 m)에서는 객체가 땅속 수십 m 로 들어간다."""
    got = net.elevation_at(x, y)
    if got is None:
        print(f"  ! 고도 조회 실패 ({x:.1f},{y:.1f}) → 경로점 고도 {fallback:.1f} m 사용")
        return fallback
    return got[0]


def player_xml(name: str, x: float, y: float, z: float, hdg: float, speed: float) -> str:
    return f"""        <Player>
            <Description Driver="DefaultDriver" Control="internal" AdaptDriverToVehicleType="true" Type="{CAR_TYPE}" Name="{name}" />
            <Init>
                <Speed Value="{e(speed)}" />
                <PosAbsolute X="{e(x)}" Y="{e(y)}" Z="{e(z)}" Direction="{e(hdg)}" AlignToRoad="true" />
            </Init>
        </Player>
"""


def actions_xml(name: str, trig: tuple[float, float], radius: float, target: float) -> str:
    """자차가 trig 반경 안에 들어오면 목표속도로 달리기 시작한다.

    Force 로 기본 드라이버의 희망속도를 덮어써야 순항 8 m/s 인 자차가 따라붙는다.
    <Autonomous> 는 넣지 않는다 — Control="internal" 인 NPC 는 이미 교통 AI 가 몰고 있어
    불필요하고, 넣으면 Ego 까지 ghostdriver(교통 AI)로 넘어가 외부 제어가 무시된다.
    """
    return f"""        <PlayerActions Player="{name}">
            <Action Name="cruise">
                <PosAbsolute CounterID="" CounterComp="COMP_EQ" Radius="{e(radius)}" X="{e(trig[0])}" Y="{e(trig[1])}" NetDist="false" CounterVal="0" Pivot="Ego" />
                <SpeedChange Rate="{e(2.0)}" Target="{e(target)}" Force="true" ExecutionTimes="1" ActiveOnEnter="true" DelayTime="0.0000000000000000e+00" />
            </Action>
        </PlayerActions>
"""


def pathshape_xml(shape_id: int, pts: list[tuple[float, float, float]], hdg: float) -> str:
    """보행 경로. 배포판의 작동하는 시나리오와 속성·순서·값을 그대로 맞춘다:

      - polyline  : 직선 횡단에 스플라인은 불필요하고, 일직선 점들은 스플라인 피팅이
                    뭉개져 보행자가 제자리걸음만 한다 (배포판 PassingAnimal/Parking 도 polyline)
      - Z         : **절대 고도**다. 지면 오프셋이 아니다. 배포판 예제가 전부 Z=0 인 것은
                    그 맵의 지면이 z=0 이기 때문이고, 이 맵은 지면이 0~79 m 다.
                    0 을 넣으면 보행자가 지면 40 m 아래로 사라진다.
      - Weight    : 없으면 경로 길이가 0 이 된다
      - Yaw       : 점마다 다음 점을 향하는 방향
    """
    wps = []
    for j, (px, py, pz) in enumerate(pts):
        k = min(j + 1, len(pts) - 1)
        yaw = hdg if k == j else math.atan2(pts[k][1] - py, pts[k][0] - px)
        wps.append(
            f"""
            <Waypoint X="{e(px)}" Y="{e(py)}" Options="0x00000000" Z="{e(pz)}" """
            f"""Weight="{e(1.0)}" Yaw="{e(yaw % (2 * math.pi))}" """
            f"""Pitch="0.0000000000000000e+00" Roll="0.0000000000000000e+00"/>""")
    return f"""        <PathShape ShapeId="{shape_id}" ShapeType="polyline" Closed="false" Name="Cross{shape_id}">{"".join(wps)}
        </PathShape>
"""


def character_xml(name: str, pt: tuple[float, float, float], hdg: float) -> str:
    ctype, appear = PED_TYPE
    x, y, z = pt
    return f"""        <Character CharacterType="{ctype}" Class="pedestrian" Appearance="{appear}" Name="{name}">
            <StartPosAbs X="{e(x)}" Y="{e(y)}" Z="{e(z)}" Direction="{e(hdg % (2 * math.pi))}" />
        </Character>
"""


def char_actions_xml(name: str, shape_id: int, speed: float,
                     radius: float, trig: tuple[float, float], now: bool = False) -> str:
    """now=True 면 자차와 무관하게 시작 즉시 걷는다 (배포판 예제와 같은 자기참조 트리거)."""
    if now:
        trigger = (f"""<PosRelative CounterID="" CounterComp="COMP_EQ" NetDist="false" """
                   f"""Distance="{e(1.0)}" CounterVal="0" Pivot="{name}"/>""")
    else:
        trigger = (f"""<PosAbsolute CounterID="" CounterComp="COMP_EQ" Radius="{e(radius)}" """
                   f"""X="{e(trig[0])}" Y="{e(trig[1])}" NetDist="false" CounterVal="0" Pivot="Ego"/>""")
    return f"""        <CharacterActions Character="{name}">
            <Action Name="">
                {trigger}
                <EditorPos Radius="{e(5.0)}" X="{e(trig[0])}" Y="{e(trig[1])}"/>
                <Motion Move="walk" Rate="0.0000000000000000e+00" Speed="{e(speed)}" Force="true" ExecutionTimes="1" ActiveOnEnter="true" DelayTime="0.0000000000000000e+00"/>
                <CharacterPath Loop="{'true' if now else 'false'}" PathShape="{shape_id}" ExecutionTimes="1" ActiveOnEnter="true" DelayTime="0.0000000000000000e+00" Beam="true" ClampToGround="true"/>
            </Action>
        </CharacterActions>
"""


def cross_car_pose(net: RoadNetwork, road_id: str, target: tuple[float, float], back: float):
    """교차 도로 위에서 target(교차로) 을 향하고 back[m] 뒤에 있는 지점 → (x, y, z, hdg).

    도로의 두 진행 방향 중 교차로에 가까워지는 쪽을 고른다. 차로 중심으로 오프셋한다.
    """
    rd = net.roads[road_id]
    tx, ty = target
    d = [math.hypot(x - tx, y - ty) for x, y in zip(rd.X, rd.Y)]
    s_near = rd.S[min(range(len(d)), key=lambda i: d[i])]
    # 교차로에 붙은 쪽은 도로의 한쪽 끝이다. 그 끝을 향해 달리는 방향이 '접근'이다.
    fwd = +1 if s_near > (rd.S[0] + rd.S[-1]) / 2.0 else -1   # +1 = s 증가 방향 주행
    s = min(max(s_near - fwd * back, rd.S[0] + 1.0), rd.S[-1] - 1.0)
    k = min(range(len(rd.S)), key=lambda i: abs(rd.S[i] - s))
    hdg = rd.H[k] if fwd > 0 else rd.H[k] + math.pi
    # OpenDRIVE: 음수 차선이 s 증가 방향, 양수 차선이 s 감소 방향
    lanes = [l for l in rd.driving_lanes(s) if (l < 0) == (fwd > 0)]
    if not lanes:
        raise SystemExit(f"교차 도로 {road_id} s={s:.1f} 에 진행방향 차로가 없다")
    lane = min(lanes, key=abs)                        # 중앙선에 가장 가까운 1차로
    t = rd.lane_center_t(s, lane)
    x, y, z, _ = rd.pose_at(s, t)
    return x, y, z, hdg


def check(net: RoadNetwork, r: Route) -> None:
    """배치 기하가 맞는지 확인한다 — 부호 하나 틀리면 객체가 반대편 인도에 선다."""
    from hlfma.perception import relate

    p = r.p[r.index_at_cum(200.0)]
    for lat in (-7.0, 3.0):
        x, y = lat_point(p, lat)
        ob = {"x": x, "y": y, "height": 1.8, "length": 0.6, "speed": 0.0, "heading": 0.0}
        got = relate(r, r.index_at_cum(200.0), 200.0, [ob])[0]
        assert abs(got.lat - abs(lat)) < 0.3, f"횡거리 불일치: {got.lat:.2f} != {abs(lat)}"
        assert abs(got.along) < 2.0, f"종거리가 0 이 아니다: {got.along:.2f}"
        assert got.kind == "ped", f"분류 오류: {got.kind}"

    for _, ji, road_id, back, _, _ in CROSS_CARS:
        q = r.p[r.index_at_cum(r.junctions[ji].entry_cum)]
        x, y, _, hdg = cross_car_pose(net, road_id, (q.x, q.y), back)
        to_j = math.atan2(q.y - y, q.x - x)
        off = abs(math.atan2(math.sin(hdg - to_j), math.cos(hdg - to_j)))
        assert off < math.pi / 2, f"교차차 {road_id} 가 교차로 반대쪽을 본다 ({math.degrees(off):.0f}°)"

    print("자체검사 통과: 횡거리 부호·교차차 진행방향")


def main() -> None:
    csv = sys.argv[1] if len(sys.argv) > 1 else "../redmine/attachments/news29_route_pretest_2.csv"
    args = [a for a in sys.argv[2:] if not a.startswith("--")]
    out_name = args[0] if args else BASE.stem + ("_pedtest.xml" if "--test-peds" in sys.argv else "_dense.xml")

    cfg = RunConfig()
    net = RoadNetwork(cfg.xodr)
    path = RoutePlanner(net).plan(load_waypoints(csv), verbose=False)
    r = Route(path, net, cfg.vehicle, cfg.control)
    print(f"경로 {r.length:.1f} m, 교차로 {len(r.junctions)}개")

    if "--list" in sys.argv:
        route_roads = {p.road for p in r.p}
        for n, J in enumerate(r.junctions):
            q = r.p[r.index_at_cum(J.entry_cum)]
            jid = net.roads[q.road].junction
            print(f"\n[{n}] entry_cum={J.entry_cum:6.1f} junction={jid} ({q.x:.1f},{q.y:.1f})")
            for rid in sorted({c["incomingRoad"] for c in net.junctions.get(jid, [])}):
                rd = net.roads.get(rid)
                if rd is None:
                    continue
                d = min(math.hypot(x - q.x, y - q.y) for x, y in zip(rd.X, rd.Y))
                tag = "경로" if rid in route_roads else "교차 후보"
                print(f"    road {rid:>5}  길이 {rd.S[-1]:6.1f} m  거리 {d:5.1f} m  {tag}")
        return

    check(net, r)
    players, actions, chars = [], [], []

    cars, cross, peds = list(CARS), list(CROSS_CARS), list(PEDS)
    if "--stress" in sys.argv:
        cars += STRESS_CARS
        peds += STRESS_PEDS
        # 교차 차량은 교차 도로 id 를 자동으로 고른다(경로에 없는 incoming 도로)
        for name, ji, _none, back, target, radius in STRESS_CROSS:
            if ji >= len(r.junctions):
                continue
            q = r.p[r.index_at_cum(r.junctions[ji].entry_cum)]
            jid = net.roads[q.road].junction
            route_roads = {p.road for p in r.p}
            cands = [rid for rid in sorted({c["incomingRoad"] for c in net.junctions.get(jid, [])})
                     if rid not in route_roads and rid in net.roads]
            if cands:
                cross.append((name, ji, cands[0], back, target, radius))
        print(f"[stress] 선행차 {len(cars)}대, 교차차량 {len(cross)}대, 보행자 {len(peds)}명")

    for name, cum, off, target, radius in cars:
        p = r.p[r.index_at_cum(cum)]
        x, y = lat_point(p, off)
        z = zof(net, x, y, zof(net, p.x, p.y, 0.0))
        players.append(player_xml(name, x, y, z, p.heading, 0.0))
        actions.append(actions_xml(name, (x, y), radius, target))
        print(f"  선행차 {name:8s} cum {cum:6.1f} ({x:8.2f},{y:8.2f}) v={target} m/s 발동 {radius:.0f} m")

    for name, ji, road_id, back, target, radius in cross:
        if ji >= len(r.junctions):
            print(f"  ! {name}: 교차로 {ji} 없음 — 건너뜀")
            continue
        q = r.p[r.index_at_cum(r.junctions[ji].entry_cum)]
        x, y, z, hdg = cross_car_pose(net, road_id, (q.x, q.y), back)
        players.append(player_xml(name, x, y, z, hdg, 0.0))
        actions.append(actions_xml(name, (q.x, q.y), radius, target))   # 발동 기준은 교차로
        d = math.hypot(x - q.x, y - q.y)
        print(f"  교차차 {name:8s} road {road_id} ({x:8.2f},{y:8.2f}) 교차로까지 {d:.1f} m "
              f"→ {d / target:.1f}s / 자차 {radius / 6.0:.1f}s")

    # --test-peds: 보행자가 걷는지만 15 초 안에 판정한다. 자차 출발점 20 m 앞에 한 명을
    # 놓고 시작 즉시 왕복시킨다 — 주행도 제어기도 필요 없이 눈으로 바로 확인된다.
    test_peds = "--test-peds" in sys.argv
    if test_peds:
        peds = [("PedTest", 20.0, -1, 1.4, None)]

    walk_loop = "--stress" in sys.argv or "--walk-loop" in sys.argv
    shapes, characters, char_acts = [], [], []
    for n, (name, cum, side, speed, radius) in enumerate(peds):
        p = r.p[r.index_at_cum(cum)]
        right, left = road_edges(net, p)
        z_ref = zof(net, p.x, p.y, 0.0)      # 경로 중심 고도 = 폴백 기준
        l0 = (right - PED_MARGIN) if side < 0 else (left + PED_MARGIN)
        l1 = (left + PED_MARGIN) if side < 0 else (right - PED_MARGIN)
        if radius is None:
            # 자차가 도착하는 순간 보행자가 자차 차로(lat 0)에 있도록 맞춘다
            want = EGO_SPEED * abs(l0) / speed
            radius = max(15.0, min(80.0, want))
            if abs(radius - want) > 1.0:
                print(f"  ! {name}: 발동반경 {want:.0f} m 가 필요한데 {radius:.0f} m 로 잘렸다 "
                      f"— 자차와 보행자가 만나지 않는다. side 를 뒤집거나 cum 을 옮겨라")
        if abs(l1 - l0) > PED_MAX_CROSS_M:
            print(f"  - {name} 제외: 횡단거리 {abs(l1 - l0):.1f} m (> {PED_MAX_CROSS_M:.0f} m) "
                  f"— 간선도로 무단횡단은 비현실적")
            continue
        pts = []
        for j in range(PED_WAYPOINTS):
            lat = l0 + (l1 - l0) * j / (PED_WAYPOINTS - 1)
            x, y = lat_point(p, lat)
            pts.append((x, y, zof(net, x, y, z_ref)))
        walk_h = math.atan2(pts[-1][1] - pts[0][1], pts[-1][0] - pts[0][0])
        trig = lat_point(p, 0.0)
        sid = 900 + n
        shapes.append(pathshape_xml(sid, pts, walk_h))
        characters.append(character_xml(name, pts[0], walk_h))
        # --stress 에서는 자차 도착을 기다리지 않고 처음부터 왕복시킨다.
        # 발동형은 '자차가 오면 딱 맞춰 건너는' 유리한 타이밍만 시험하게 된다 —
        # 왕복이면 매 주행마다 다른 위상에서 마주쳐 우연한 통과를 걸러낸다.
        char_acts.append(char_actions_xml(name, sid, speed, radius, trig,
                                         now=test_peds or walk_loop))
        print(f"  보행자 {name:8s} cum {cum:6.1f} lat {l0:+.1f}→{l1:+.1f} "
              f"{math.dist(pts[0][:2], pts[-1][:2]):.1f} m "
              f"{'시작 즉시 왕복' if test_peds else '발동 %.0f m' % radius} (도로 {right:+.1f}~{left:+.1f})")
    # 작동하는 배포판 시나리오와 같은 순서: PathShape 전부 → Character 전부 → Actions 전부
    chars = shapes + characters + char_acts

    src = BASE.read_text(encoding="utf-8")
    anchor_p = '        <PlayerActions Player="Ego" />\n'
    i0 = src.find("    <MovingObjectsControl>")
    i1 = src.find("</MovingObjectsControl>")
    if anchor_p not in src or i0 < 0 or i1 < 0:
        raise SystemExit("기준 시나리오 구조가 예상과 다르다 — 앵커를 찾지 못했다")
    # NPC 액션을 Ego 의 빈 PlayerActions **앞**에 둔다. 뒤에 두면 VTD 가 액션을 Ego 에
    # 붙여 Ego 가 교통 AI 로 넘어간다(외부 제어 무시).
    src = src.replace(anchor_p, "".join(players) + "".join(actions) + anchor_p, 1)
    # 보행자는 덧붙이지 않고 블록째 새로 쓴다. 기준 파일의 순서·Weight 누락까지 고쳐야
    # 기존 Ped1 도 실제로 걷는다.
    i0, i1 = src.find("    <MovingObjectsControl>"), src.find("</MovingObjectsControl>")
    src = src[:i0] + "    <MovingObjectsControl>\n" + "".join(chars) + "    " + src[i1:]

    out = SCEN_DIR / out_name
    out.write_text(src, encoding="utf-8")
    print(f"\n생성: {out}")
    print(f"실행: python3 tools/simctl.py load {out_name}")


if __name__ == "__main__":
    main()
