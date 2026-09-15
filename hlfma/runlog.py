"""주행 로그 — 사후 분석용. 매 프레임 CSV + 주기적 objects JSONL + 계획 경로 CSV.

로그 디렉터리: logs/<YYYYmmdd_HHMMSS>/
  frames.csv   t,x,y,h,v,idx,e_lat,steer,accel,signal,tl_id,tl_state,state,notes
  objects.jsonl {"t":..,"objs":[...]}  (10 프레임마다)
  path.csv     계획 경로
  meta.json    설정·경로 파일·시작 시각
"""
from __future__ import annotations

import csv
import json
import time
from pathlib import Path


class RunLog:
    def __init__(self, log_dir: str | Path, meta: dict | None = None, enabled: bool = True):
        self.enabled = enabled
        if not enabled:
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.dir = Path(log_dir) / stamp
        self.dir.mkdir(parents=True, exist_ok=True)
        self._f = open(self.dir / "frames.csv", "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(["t", "x", "y", "h", "v", "idx", "e_lat", "steer", "accel",
                          "signal", "tl_id", "tl_state", "state", "notes"])
        self._o = open(self.dir / "objects.jsonl", "w", encoding="utf-8")
        self._n = 0
        self.t0 = time.monotonic()
        if meta:
            (self.dir / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")

    def frame(self, x, y, h, v, idx, e_lat, steer, accel, signal, tl_id, tl_state, state, notes, objs=None):
        if not self.enabled:
            return
        t = time.monotonic() - self.t0
        self._w.writerow([f"{t:.2f}", f"{x:.3f}", f"{y:.3f}", f"{h:.4f}", f"{v:.2f}", idx, f"{e_lat:.2f}",
                          f"{steer:.3f}", f"{accel:.2f}", signal, tl_id, tl_state, state, notes])
        self._n += 1
        if objs is not None and self._n % 10 == 0:
            slim = [{k: (round(v_, 3) if isinstance(v_, float) else v_) for k, v_ in o.items()}
                    for o in objs]
            self._o.write(json.dumps({"t": round(t, 2), "objs": slim}) + "\n")
        if self._n % 100 == 0:
            self._f.flush(); self._o.flush()

    def save_path(self, path) -> None:
        if not self.enabled:
            return
        with open(self.dir / "path.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["x", "y", "heading", "road", "lane", "s", "junction"])
            for p in path:
                w.writerow([f"{p.x:.3f}", f"{p.y:.3f}", f"{p.heading:.5f}", p.road, p.lane, f"{p.s:.2f}", int(p.in_junction)])

    def event(self, text: str) -> None:
        if not self.enabled:
            return
        with open(self.dir / "events.log", "a", encoding="utf-8") as f:
            f.write(f"{time.monotonic() - self.t0:8.2f}  {text}\n")

    def close(self) -> None:
        if not self.enabled:
            return
        self._f.close(); self._o.close()
