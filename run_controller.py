#!/usr/bin/env python3
"""HL FMA 대회 제어기 — 단일 진입점.

    python3 run_controller.py <route.csv> --host <VTD_PC_IP> [--cruise 8] [--plan-only]

대회 당일 절차: 랜선 연결 → USB 에서 route.csv 복사 → (--plan-only 로 경로 확인) →
시뮬레이션 시작 → 이 프로그램 실행 → 이후 조작 없음 (완주 후에도 종료하지 않는다).
"""
from __future__ import annotations

import math
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from hlfma.config import RunConfig
from hlfma.hlvtd import HlVtdClient, ego_pose, objects, traffic_light, TURN_OFF
from hlfma.odr import RoadNetwork
from hlfma.planner import RoutePlanner, load_waypoints, PlanError
from hlfma.route import Route
from hlfma.controller import Controller
from hlfma.runlog import RunLog


def main() -> int:
    ap = argparse.ArgumentParser(description="HL FMA 자율주행 제어기")
    ap.add_argument("route", help="경유지 CSV (seq,x,y)")
    ap.add_argument("--host", default=None)
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--cruise", type=float, default=None, help="순항 속도 m/s")
    ap.add_argument("--config", default=str(Path(__file__).resolve().parent / "hlfma.json"))
    ap.add_argument("--plan-only", action="store_true")
    ap.add_argument("--no-log", action="store_true")
    args = ap.parse_args()

    cfg = RunConfig.load(args.config)
    if args.host:
        cfg.host = args.host
    if args.port:
        cfg.port = args.port
    if args.cruise:
        cfg.cruise = args.cruise

    t0 = time.monotonic()
    net = RoadNetwork(cfg.xodr)
    wps = load_waypoints(args.route)
    try:
        path = RoutePlanner(net).plan(wps, verbose=True)
    except (PlanError, ValueError, KeyError, RecursionError) as e:
        # locate() 는 ValueError 를 던진다 (planner.py). PlanError 만 잡으면 당일
        # 트레이스백으로 죽어 "경로계획 실패" 안내조차 못 남긴다.
        print("경로계획 실패:", e)
        return 2
    route = Route(path, net, cfg.vehicle, cfg.control, cfg.speed_zone30)
    print(f"{route.summary()}  (계획 {time.monotonic() - t0:.1f}s)")
    if args.plan_only:
        return 0

    log = RunLog(cfg.log_dir, meta={"route": str(Path(args.route).resolve()), "config": cfg.dump()},
                 enabled=not args.no_log)
    log.save_path(path)
    ctl = Controller(route, cfg, log)

    client = HlVtdClient(cfg.host, cfg.port, timeout=10.0, first_timeout=cfg.first_packet_s)
    print(f"[접속 시도] {cfg.host}:{cfg.port} (최대 {cfg.connect_timeout_s:.0f}s)")
    client.connect_retry(cfg.connect_timeout_s)
    print("[접속] 첫 데이터 대기 중 (시뮬레이션 시작 후 최대 %.0fs)" % cfg.first_packet_s)

    # 첫 패킷도 루프와 같은 방식으로 재시도한다. 여기서 죽으면 "제어기 실행 후 무조작"
    # 절차가 깨진다 (운영측 시뮬 시작 지연·재시작 시 첫 데이터가 늦게 온다).
    while True:
        try:
            pkt = client.step(0.0, 0.0, TURN_OFF)
            break
        except OSError as e:
            print(f"[첫 데이터 없음] {e} → 재접속 후 재시도")
            log.event(f"첫 데이터 대기 실패: {e}")
            client.close(send_stop=False)
            client.connect_retry(cfg.connect_timeout_s)
    print("[데이터 수신] 주행 시작")

    # 경로 CSV 와 시뮬레이션 시나리오가 안 맞으면(파일명 착오) 자차가 경로에서 수백 m
    # 떨어진 채 시작한다. 그대로 두면 RECOVER 로 들어가 조향을 끝까지 꺾고 엉뚱하게
    # 출발하므로, 사람이 알아챌 수 있게 크게 알린다. 중단하지는 않는다 — 오탐이 나도
    # "제어기 실행 후 무조작" 절차를 깨면 안 된다.
    x0, y0, _z0, _h0 = ego_pose(pkt)
    d0 = math.hypot(x0 - route.p[0].x, y0 - route.p[0].y)
    if d0 > 50.0:
        msg = (f"자차({x0:.1f}, {y0:.1f}) 가 경로 시작점({route.p[0].x:.1f}, {route.p[0].y:.1f}) 에서 "
               f"{d0:.0f} m 떨어져 있다 — 경로 CSV 와 시나리오가 다른 것 같다")
        print("\n" + "!" * 78 + f"\n!! {msg}\n" + "!" * 78 + "\n")
        log.event(msg)
    elif d0 > 1.0:
        # 경유지는 "대략적 위치, 차로값 아님"(PROTOCOL.md). 1 번 경유지가 찍힌 차로에서
        # 출발하면 자차가 다른 차로에 있을 때 시작하자마자 불필요한 차로변경을 한다.
        # 자차 실제 위치를 출발점으로 다시 계획해 **현재 차로 그대로** 출발시킨다.
        try:
            path2 = RoutePlanner(net).plan([(x0, y0)] + wps[1:], verbose=False)
            route = Route(path2, net, cfg.vehicle, cfg.control, cfg.speed_zone30)
            ctl = Controller(route, cfg, log)
            log.save_path(path2)
            print(f"[재계획] 자차 위치에서 출발 (경유지 1번과 {d0:.1f} m 차이) — {route.summary()}")
            log.event(f"자차 위치 재계획: 경유지1과 {d0:.1f} m 차이")
        except (PlanError, ValueError, KeyError, RecursionError) as e:
            print(f"[재계획 실패] {e} — 원래 경로로 진행")
            log.event(f"재계획 실패: {e}")

    n = 0
    try:
        while True:
            x, y, _z, h = ego_pose(pkt)
            tl = traffic_light(pkt)
            objs = objects(pkt)
            steer, accel, signal, e_lat = ctl.step(x, y, h, tl, objs)
            log.frame(x, y, h, ctl.v, ctl.idx, e_lat, steer, accel, signal, tl[0], tl[1], ctl.state,
                      " ".join(ctl.notes), objs)
            try:
                pkt = client.step(steer, accel, signal)
            except (ConnectionError, OSError) as e:
                print(f"[연결 끊김] {e} → 재접속")
                log.event(f"연결 끊김: {e}")
                client.close(send_stop=False)
                client.connect_retry(cfg.connect_timeout_s)
                pkt = client.step(0.0, -2.0, TURN_OFF)
                continue
            n += 1
            if n % 40 == 0:
                print(f"  ({x:7.1f},{y:7.1f}) {ctl.status}")
    except KeyboardInterrupt:
        print("\n[중단] 정지 명령 송신")
        try:
            for _ in range(20):
                client.step(0.0, -4.0, TURN_OFF)
        except Exception:
            pass
    finally:
        log.close()
        client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
