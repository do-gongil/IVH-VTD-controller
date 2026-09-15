#!/usr/bin/env python3
"""HLVTD 통신 계층.

ModuleManager 가 기동 시 출력하는 규격 (2026-09-08 실측 확인):
    Protocol: headerless   DataPktSize: 1109 B   CtrlPktSize: 9 B

제어 패킷 (client -> sim), 9 B, little-endian:
    steering    float32   조향각 [rad]   (+ 우, - 좌)
    targetAccel float32   목표 가속도 [m/s^2]  (음수 = 감속)
    turnSignal  uint8     0=OFF 1=LEFT 2=RIGHT

데이터 패킷 (sim -> client), 1109 B, little-endian, 헤더 없음:
    offset  0  float32  자차 X [m]        <- 실측 확인
    offset  4  float32  자차 Y [m]        <- 실측 확인
    offset  8  float32  자차 Z [m] 고도    <- 실측 확인 (리빙랩 지형 약 42 m)
    offset 12  float32  heading [rad]     <- 실측 확인 (이동방향과 0.005 rad 이내 일치)
    offset 16  float32  pitch [rad]       <- 추정
    offset 20  float32  roll  [rad]       <- 추정
    즉 선두 24 B 는 (x, y, z, h, p, r) 구조다.
    나머지(오브젝트 30개·신호등 5 B)는 아래 각 절에 실측 확정해 두었다.

통신은 lock-step 이다. 제어 1개를 보내면 데이터 1개가 온다 (실측 303:303, 20 Hz).
따라서 send -> recv 를 한 쌍으로 돌리면 자연히 시뮬레이터 주기에 동기화된다.
"""
from __future__ import annotations

import socket
import struct

DATA_SIZE = 1109
CTRL_FMT = "<ffB"

# 모든 좌표의 기준점은 egoReferencePoint = "rear_axle_center" (hl_vtd_config.json).

TURN_OFF, TURN_LEFT, TURN_RIGHT = 0, 1, 2

STEER_LIMIT = 0.75     # rad. 전송 단 클램프 — config.Control.steer_max(0.65) 보다 넉넉히.
ACCEL_MAX = 3.0
ACCEL_MIN = -6.0


class HlVtdClient:
    """TCP 9910 통합 DATA/CONTROL 채널."""

    def __init__(self, host: str = "127.0.0.1", port: int = 9910,
                 timeout: float = 10.0, first_timeout: float = 45.0):
        # first_timeout: 첫 데이터 패킷 대기. 시뮬레이터는 시나리오 Apply→InitDone 까지
        # 25~30 s 가 걸릴 수 있고 그동안은 접속만 되고 데이터가 오지 않는다.
        # 대회 절차(시뮬 시작 → 제어기 시작)에서도 같은 대기가 필요하다.
        self.host, self.port, self.timeout = host, port, timeout
        self.first_timeout = first_timeout
        self.sock: socket.socket | None = None
        self._got_first = False

    def connect(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(self.first_timeout)
        s.connect((self.host, self.port))
        self.sock = s
        self._got_first = False

    def connect_retry(self, total_s: float = 600.0, interval_s: float = 1.0) -> None:
        """대회 절차상 제어기를 시뮬레이터보다 먼저 켜도 되도록, 접속 거부를 재시도한다."""
        import time
        t0 = time.monotonic()
        n = 0
        while True:
            try:
                self.connect()
                if n:
                    print(f"[접속 성공] {n}회 재시도 후")
                return
            except OSError as e:
                n += 1
                if time.monotonic() - t0 > total_s:
                    raise ConnectionError(f"{self.host}:{self.port} 접속 실패 ({total_s:.0f}s 초과): {e}")
                if n == 1 or n % 15 == 0:
                    print(f"  접속 대기 중… ({e.__class__.__name__}, {n}회)")
                time.sleep(interval_s)

    def close(self, send_stop: bool = True) -> None:
        if self.sock:
            if send_stop:
                try:
                    # 안전을 위해 마지막에 정지 명령을 남긴다
                    self.send(0.0, ACCEL_MIN, TURN_OFF)
                except OSError:
                    pass
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def send(self, steering: float, accel: float, signal: int = TURN_OFF) -> None:
        steering = max(-STEER_LIMIT, min(STEER_LIMIT, steering))
        accel = max(ACCEL_MIN, min(ACCEL_MAX, accel))
        self.sock.sendall(struct.pack(CTRL_FMT, steering, accel, signal))

    def recv(self) -> bytes:
        buf = b""
        while len(buf) < DATA_SIZE:
            chunk = self.sock.recv(DATA_SIZE - len(buf))
            if not chunk:
                raise ConnectionError("시뮬레이터가 연결을 닫음")
            buf += chunk
        if not self._got_first:
            # 첫 패킷이 왔으면 시뮬레이션이 돌고 있는 것. 이후엔 짧은 타임아웃으로
            # 전환해 연결 끊김을 빨리 감지한다.
            self._got_first = True
            self.sock.settimeout(self.timeout)
        return buf

    def step(self, steering: float, accel: float, signal: int = TURN_OFF) -> bytes:
        """제어 1개 송신 -> 데이터 1개 수신. 시뮬레이터 주기에 동기화된다."""
        self.send(steering, accel, signal)
        return self.recv()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()


def ego_pose(pkt: bytes) -> tuple[float, float, float, float]:
    """(x, y, z, heading[rad]). heading 은 실측 검증됨."""
    x, y, z, h = struct.unpack_from("<ffff", pkt, 0)
    return x, y, z, h


# ---------------------------------------------------------------- 신호등 (실측 확인)
# 패킷 마지막 5 B. 접근 도로 구간에서만 채워지고 교차로 내부/무관 구간에서는 0 이다
# (Redmine #4993). ID 는 위치·순서를 뜻하지 않으므로 state 만 보고 판단할 것 (#4984).
TL_ID_OFFSET = 1104      # int32
TL_STATE_OFFSET = 1108   # uint8


def traffic_light(pkt: bytes) -> tuple[int, int]:
    """(trafficLightId, state). id == 0 이면 해당 없음."""
    (tl_id,) = struct.unpack_from("<i", pkt, TL_ID_OFFSET)
    return tl_id, pkt[TL_STATE_OFFSET]


# 신호 상태 코드 — 실측 (Signal id=3, 접근로 정차 관측 + 전방카메라 프레임 대조)
#   주기: 5 (15 s) -> 2 (3 s) -> 1 (18 s) -> 5 ...
#   전방카메라 프레임 램프색 실측: 1=(204,51,60) 적색, 2=(200,151,77) 황색, 5=녹색 램프 점등 확인.
#   6 은 점멸 (Redmine #4990). 3, 4 는 미관측(화살표 계열 추정).
TL_RED, TL_YELLOW, TL_GREEN, TL_FLASH = 1, 2, 5, 6
# 코드 3: 2026-09-11 v7 주행에서 최초 관측. 신호등 38·90 둘 다 주기가
#   적색(8~13 s) → 3(10 s) → 황색 이었다. 황색은 녹색 뒤에만 오므로 3 은 녹색 자리다.
#   화면으로도 초록으로 확인했다. 4 는 아직 미관측이라 정지 취급을 유지한다.
TL_GREEN_ALT = 3
TL_GO = (TL_GREEN, TL_GREEN_ALT)
TL_NAME = {0: "없음", 1: "적색", 2: "황색", 3: "녹색(3)", 4: "코드4(미확인)", 5: "녹색", 6: "점멸"}


# ---------------------------------------------------------------- 오브젝트 (실측 확정)
# 1109 = 24(ego) + 30*36(objects) + 5(traffic light).
# 슬롯 36 B = id(int32), x, y, z, heading[rad], speed[m/s], length, width, height (float32).
# 2026-09-09 실측: NPC 아이오닉6 1대 배치 시 slot0 = id 2, L/W/H = 4.848/1.886/1.507 로
# 차량 제원과 정확히 일치 → 필드 순서 확정.
# id == 0 은 빈 슬롯. 80 m 이내를 가까운 순 정렬, 최대 30 개 (뉴스 [22]).
# 객체 타입은 오지 않는다 — 크기·속도로 직접 분류할 것.
OBJ_OFFSET = 24
OBJ_STRIDE = 36
OBJ_COUNT = 30


def objects(pkt: bytes) -> list[dict]:
    """유효 슬롯(id != 0)만 반환."""
    out = []
    for k in range(OBJ_COUNT):
        off = OBJ_OFFSET + k * OBJ_STRIDE
        (oid,) = struct.unpack_from("<i", pkt, off)
        if oid == 0:
            continue
        vals = struct.unpack_from("<8f", pkt, off + 4)
        out.append({"slot": k, "id": oid,
                    "x": vals[0], "y": vals[1], "z": vals[2], "heading": vals[3],
                    "speed": vals[4], "length": vals[5], "width": vals[6], "height": vals[7]})
    return out
