#!/usr/bin/env python3
"""VTD 시뮬레이션 제어 (SCP) — 시나리오 교체·리셋을 안전하게.

실측으로 확정한 절차:
  - 실행 중인 인스턴스에 LoadScenario 만 보내면 무시된다. 반드시 Stop 을 먼저 보낸다.
  - Apply 이후 TaskControl 이 "InitDone" 을 찍을 때까지 4~30 s 걸린다. 그 전에
    9910 에 붙으면 접속은 되지만 데이터가 오지 않는다. 고정 sleep 대신 로그를 폴링한다.
  - Stop → Start 만으로는 시나리오가 다시 설정되지 않는다 (VtGui: set scenario file ...).

사용:
    python3 simctl.py load  시나리오.xml     # Stop → Load → Apply → Start, InitDone 대기
    python3 simctl.py reset                 # 현재 시나리오를 처음부터 (Stop → Start)
    python3 simctl.py stop
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

VTD_ROOT = Path("/home/dap/Hexagon/VTD.2025.2")
SCP_GEN = VTD_ROOT / "Runtime/Tools/ScpGenerator/scpGenerator"
SCP_PORT = 48179
TC_LOG = Path("/tmp/taskRec_TaskControl.txt")
MM_LOG = Path("/tmp/taskRec_ModuleManager.txt")


def require_vtd() -> None:
    """TaskControl SCP 포트(48179)가 열려 있어야 SCP 명령이 통한다. 아니면 즉시 안내하고 종료."""
    import socket
    with socket.socket() as s:
        s.settimeout(1.0)
        if s.connect_ex(("127.0.0.1", SCP_PORT)) != 0:
            sys.exit("VTD 가 실행 중이 아니다 (SCP 48179 닫힘). 먼저 실행할 것:\n"
                     "  ~/Hexagon/VTD.2025.2/bin/vtdStop.sh\n"
                     "  DISPLAY=:5 /opt/VirtualGL/bin/vglrun -d egl ~/Hexagon/VTD.2025.2/bin/vtdStart.sh &\n"
                     "  (LICENSING_STATE: 1(OK) 확인 후 30초 뒤 다시 load)")


def scp(cmd: str, timeout: float = 25) -> None:
    subprocess.run([str(SCP_GEN), "-w", "-p", str(SCP_PORT), "-i", cmd],
                   capture_output=True, timeout=timeout)


def _count(path: Path, needle: str) -> int:
    try:
        return path.read_text(encoding="utf-8", errors="ignore").count(needle)
    except OSError:
        return 0


def wait_init_done(before: int, timeout: float = 90) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        if _count(TC_LOG, "InitDone") > before:
            return True
        time.sleep(1.0)
    return False


def wait_frames(timeout: float = 30) -> bool:
    """ModuleManager 로그가 커지면(=RDB 프레임 수신) 시뮬레이션이 도는 것."""
    try:
        a = MM_LOG.stat().st_size
    except OSError:
        return False
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(1.5)
        try:
            if MM_LOG.stat().st_size > a + 200:
                return True
        except OSError:
            pass
    return False


def load(scenario: str | Path) -> None:
    sc = Path(scenario).resolve()
    if not sc.is_file():
        sys.exit(f"시나리오 없음: {sc}")
    before = _count(TC_LOG, "InitDone")
    print("Stop"); scp("<SimCtrl><Stop /></SimCtrl>"); time.sleep(4)
    print(f"Load {sc.name}"); scp(f'<SimCtrl><LoadScenario filename="{sc}" /></SimCtrl>'); time.sleep(5)
    print("Apply"); scp("<SimCtrl><Apply /></SimCtrl>"); time.sleep(5)
    print("Start"); scp("<SimCtrl><Start /></SimCtrl>")
    t0 = time.time()
    ok = wait_init_done(before)
    print(f"InitDone {'OK' if ok else '타임아웃'} ({time.time() - t0:.0f}s)")
    fr = wait_frames()
    print(f"프레임 진행 {'OK' if fr else '없음 — Ego 배치(Z/도로) 또는 IG 상태를 확인할 것'}")
    tc = TC_LOG.read_text(encoding="utf-8", errors="ignore")
    i = tc.rfind("scenario file is <")
    if i >= 0:
        print("TaskControl 시나리오:", tc[i + 18:tc.find(">", i)].rsplit("/", 1)[-1])


def reset() -> None:
    print("Stop"); scp("<SimCtrl><Stop /></SimCtrl>"); time.sleep(4)
    print("Start"); scp("<SimCtrl><Start /></SimCtrl>")
    print(f"프레임 진행 {'OK' if wait_frames() else '없음'}")


def stop() -> None:
    scp("<SimCtrl><Stop /></SimCtrl>"); print("Stop 전송")


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("load", "reset", "stop"):
        print(__doc__); sys.exit(1)
    require_vtd()
    if sys.argv[1] == "load":
        load(sys.argv[2])
    elif sys.argv[1] == "reset":
        reset()
    else:
        stop()
