#!/usr/bin/env python3
"""계획기 견고성 스윕 — 대회형 합성 경로를 대량으로 돌려 하드 실패를 찾는다.

당일 경로는 미지수다. 계획기가 예외로 죽으면 제어기가 종료되어 점수 0 이므로,
지도 전역에 대회와 같은 형태(2 + 2N)의 경로를 만들어 미리 터뜨려 본다.

방법
  1) 주행차로 위 무작위 두 점 a, b 로 기준 경로를 계획한다.
  2) 기준 경로의 교차로 진입·진출 지점을 뽑아 대회형 경유지(2 + 2N)를 만든다.
  3) 각 교차로 경유지를 같은 방향 '다른 차로 중심' 으로 옮긴다.
     (뉴스 [30]: "좌회전 하는 구간에서 3차로로 경유지가 제공될수도있습니다")
  4) 그 경유지로 다시 계획해 검사한다.

핵심 성질: 1)에서 경로 존재가 증명된 상태이므로, 그 경로 위에서 뽑은 경유지로
4)가 실패하면 확정적으로 계획기 결함이다.

사용:
    python3 tools/sweep_routes.py --n 1000 --seed 20260912
"""
from __future__ import annotations
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent.parent))

import argparse
import collections
import contextlib
import csv
import io
import math
import random
import re
import time
from pathlib import Path

from hlfma.config import DEFAULT_XODR
from hlfma.odr import RoadNetwork
from hlfma import planner as P
from hlfma.planner import RoutePlanner, on_drivable, path_length, MAX_GAP, MAX_TURN_DEG, SEAM_TURN_DEG


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# 같은 결함이 수십 개 서명으로 쪼개지지 않도록 좌표·번호를 전부 지운다.
# 점 번호와 경유지 번호를 빠뜨리면 결함 수를 10배 과소평가한다.
_NORM = ((r"\(-?[\d.]+,-?[\d.]+\)", "(X,Y)"), (r"점 \d+", "점 N"),
         (r"경유지 \d+→\d+", "경유지 i→i+1"), (r"road \d+", "road R"),
         (r"\[[^\]]*\]", "[...]"), (r"-?\d+\.\d+", "N"))


def signature(e: BaseException) -> str:
    m = str(e)
    for pat, rep in _NORM:
        m = re.sub(pat, rep, m)
    return f"{type(e).__name__}: {m[:70]}"


def count_seams(pts) -> int:
    """finalize_path 가 재보간해야 했던 이음새 수. 상류 불연속의 냄새."""
    n = 0
    for a, b in zip(pts, pts[1:]):
        gap = math.hypot(b.x - a.x, b.y - a.y)
        if gap > MAX_GAP or abs(wrap(b.heading - a.heading)) > math.radians(SEAM_TURN_DEG):
            n += 1
    return n


def check_invariants(net, path, wps) -> list[str]:
    """tests/test_planner_routes.py 와 같은 불변식. 위반 목록을 돌려준다."""
    bad = []
    gaps = [math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(path, path[1:])]
    turns = [math.degrees(abs(wrap(b.heading - a.heading))) for a, b in zip(path, path[1:])]
    if len(path) <= 50:
        bad.append(f"점 부족 {len(path)}")
    if gaps and max(gaps) > MAX_GAP + 0.2:
        bad.append(f"최대 간격 {max(gaps):.2f} m")
    if gaps and min(gaps) < 0.3:
        bad.append(f"중복 점 {min(gaps):.2f} m")
    if turns and max(turns) > MAX_TURN_DEG:
        bad.append(f"heading 점프 {max(turns):.1f}°")
    off = [p for p in path if not on_drivable(net, p, tol=0.8)]
    if len(off) > 2:
        bad.append(f"주행영역 밖 {len(off)}/{len(path)} (road {off[0].road})")
    return bad


def lane_jitter(rnd, net, p, lon: float):
    """경유지를 같은 진행방향의 다른 차로 중심으로 옮긴다 (+ 선택적 종방향 흔들기)."""
    rd = net.roads[p.road]
    same = [l for l in rd.driving_lanes(p.s) if (l < 0) == (p.lane < 0)]
    try:
        x, y, _, _ = rd.pose_at(p.s, rd.lane_center_t(p.s, rnd.choice(same) if same else p.lane))
    except (KeyError, IndexError):
        x, y = p.x, p.y
    if lon:
        d = rnd.uniform(-lon, lon)
        x += math.cos(p.heading) * d
        y += math.sin(p.heading) * d
    return (x, y)


def dump_csv(out_dir: Path, tag: str, wps) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    f = out_dir / f"{tag}.csv"
    with open(f, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["seq", "x", "y"])
        for i, (x, y) in enumerate(wps, 1):
            w.writerow([i, f"{x:.3f}", f"{y:.3f}"])
    return f


def main() -> int:
    ap = argparse.ArgumentParser(description="계획기 견고성 스윕")
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=20260912)
    ap.add_argument("--out", default="sweep_fail")
    ap.add_argument("--min-dist", type=float, default=150.0)
    ap.add_argument("--lon-jitter", type=float, default=0.0, help="종방향 ±m (가혹 변형)")
    ap.add_argument("--max-junctions", type=int, default=0,
                    help="교차로 수 상한 (대회 경로는 3~4개). 0=제한 없음")
    a = ap.parse_args()

    net = RoadNetwork(DEFAULT_XODR)
    rp = RoutePlanner(net)
    rnd = random.Random(a.seed)
    roads = [r for r in net.roads.values()
             if r.junction == "-1" and r.sections and len(r.S) > 4 and r.driving_lanes(r.length / 2)]

    # finalize_path 를 감싸 재보간 전 점을 관측한다 (hlfma/ 는 건드리지 않는다)
    pre = {}
    _orig = P.finalize_path
    def spy(pts, *a, **kw):          # finalize_path(pts, net) 로 인자가 늘어도 견디게
        pre["pts"] = list(pts)
        return _orig(pts, *a, **kw)
    P.finalize_path = spy

    def rand_pt():
        rd = rnd.choice(roads)
        s = rnd.uniform(3.0, max(3.5, rd.length - 3.0))
        return rd.pose_at(s, rd.lane_center_t(s, rnd.choice(rd.driving_lanes(s))))[:2]

    out = Path(a.out)
    defect = collections.Counter(); lowpri = collections.Counter(); noise = collections.Counter()
    inv = collections.Counter(); rep_ex = {}
    ok = skipped = seam_routes = warn_routes = 0
    t0 = time.time()

    for _ in range(a.n):
        p_a, p_b = rand_pt(), rand_pt()
        if math.hypot(p_b[0] - p_a[0], p_b[1] - p_a[1]) < a.min_dist:
            skipped += 1
            continue
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                base = rp.plan([p_a, p_b], verbose=False)
        except Exception as e:                                  # 기준 계획 실패 = 대회 조건 아님
            (noise if "경로 없음" in str(e) else lowpri)[signature(e)] += 1
            continue

        marks = [i for i in range(1, len(base)) if base[i].in_junction != base[i - 1].in_junction]
        if a.max_junctions and len(marks) > 2 * a.max_junctions:
            skipped += 1                       # 대회 형태를 벗어난 장거리 순환 경로
            continue
        wps = [p_a] + [lane_jitter(rnd, net, base[i], a.lon_jitter) for i in marks] + [p_b]

        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                path = rp.plan(wps, verbose=False)
        except Exception as e:
            sg = signature(e)
            defect[sg] += 1
            if sg not in rep_ex:
                rep_ex[sg] = dump_csv(out, f"replan_{len(rep_ex):03d}", wps)
            continue

        bad = check_invariants(net, path, wps)
        if bad:
            sg = "INVARIANT: " + "; ".join(re.sub(r"(?<!road )\b[\d.]+", "N", b) for b in bad)[:70]
            inv[sg] += 1
            if sg not in rep_ex:
                rep_ex[sg] = dump_csv(out, f"inv_{len(rep_ex):03d}", wps)
        else:
            ok += 1
        if "pts" in pre and count_seams(pre["pts"]):
            seam_routes += 1
        if "경고" in buf.getvalue():
            warn_routes += 1

    P.finalize_path = _orig
    valid = ok + sum(defect.values()) + sum(inv.values())
    dt = time.time() - t0
    print(f"\n시행 {a.n}  유효 {valid}  성공 {ok}  (버림 {skipped}, {dt:.0f}초, seed={a.seed})\n")

    def show(title, ctr, note="", dumps=False):
        if not ctr:
            print(f"[{title}]  없음"); return
        print(f"[{title}]  {sum(ctr.values())} 건 / 서명 {len(ctr)} 종 {note}")
        for k, v in ctr.most_common():
            print(f"   {v:4d}  {k}")
            if dumps and k in rep_ex:
                print(f"         대표 → {rep_ex[k]}")
        print()

    show("DEFECT-replan", defect, "← 최우선: 대회와 같은 경유지 밀도", dumps=True)
    show("DEFECT-invariant", inv, dumps=True)
    show("LOWPRI-base", lowpri, "← 경유지 2개 장거리 구간. 대회 조건 아님, 참고용")
    show("NOISE-unreachable", noise, "← 무작위 쌍이 실제로 도달 불가")
    print(f"진단: 이음새 재보간이 일어난 경로 {seam_routes}, 차로변경 여유 경고 {warn_routes}")
    return 1 if (defect or inv) else 0


if __name__ == "__main__":
    _sys.exit(main())
