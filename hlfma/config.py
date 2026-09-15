"""제어기 설정 — 차량 제원(주최측 hl_vtd_config.json 값 고정) + 제어 상수 + 실행 설정."""
from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parent
ROOT_DIR = PKG_DIR.parent
DEFAULT_XODR = ROOT_DIR / "data" / "HL_FMA_VTD_LivingLab.xodr"


@dataclass(frozen=True)
class Vehicle:
    """00_HL_VTD/Config/HLVTD/hl_vtd_config.json vehicleGeometry (HyundaiIoniq6_23_Dyn)."""
    length: float = 4.848
    width: float = 1.886
    height: float = 1.507
    rear_to_front: float = 3.808      # 후륜축(기준점) → 앞 범퍼
    rear_to_rear: float = 1.040       # 후륜축 → 뒤 범퍼
    wheelbase: float = 2.95           # 아이오닉6 축거 (순수추종용)
    obb_half_width: float = 0.943


@dataclass(frozen=True)
class Control:
    steer_max: float = 0.65           # rad. 0.35 는 예제 제어기를 따라 걸었던 값이고
                                      # 프로토콜 제약이 아니다. 0.35 면 최소회전반경이
                                      # wheelbase/tan = 8.1 m 인데, 리빙랩 우회전
                                      # 연결로(797)는 R=6.4 m 라 원천적으로 못 돈다.
                                      # 0.50 → 5.4 m. 그래도 pretest/route_example 의
                                      # 일부 연결로(R 4.1/4.4 m)는 못 돈다 → 0.65 → 3.9 m.
                                      # 실차 조향륜도 30~35°(0.52~0.61 rad) 꺾인다.
                                      # v>10 m/s 는 steer_max_fast=0.15 로 여전히 묶인다.
    steer_rate: float = 1.0           # rad/s. 한계를 올린 만큼 도달 시간도 맞춘다
                                      # (0.6 이면 0→0.5 rad 에 0.83 s = 회전 구간 3 m)
    steer_max_fast: float = 0.15      # v > 10 m/s 일 때 조향 상한
    look_min: float = 4.0
    look_max: float = 14.0
    look_k: float = 0.9               # ld = clamp(2.5 + k·v, look_min, look_max)
    look_base: float = 2.5
    look_err: float = 0.35            # 곡선에서 허용할 순수추종 정상편차 [m].
                                      # 순수추종은 곡률 κ 에서 ld²·κ/2 만큼 벌어진다.
                                      # ld = sqrt(2·look_err/κ) 로 되짚어 제한한다.
                                      # steer_max 0.35 시절엔 이걸 켜면 발산했다 —
                                      # R=6.4 m 추종에 atan(L/R)=0.43 rad 이 필요한데
                                      # 낼 수가 없었다. 0.50 으로 올린 뒤에만 유효하다.
    look_min_curve: float = 2.5       # 곡선 ld 하한 (짧으면 조향이 떨린다)
    a_lat: float = 1.8                # 곡선 허용 횡가속 → v = sqrt(a_lat/κ)
    a_brake: float = 2.5              # 계획 감속도 (정지선·종료)
    a_brake_ped: float = 1.2          # 보행자 전용. 작을수록 **더 일찍** 감속한다
                                      # (v_cap = sqrt(2·a·d)). 저속은 무감점이고
                                      # 충돌은 전부 참가자 책임이라 보수적으로 잡는다.
    a_brake_hard: float = 4.0
    accel_max: float = 3.0
    accel_min: float = -6.0
    kp_v: float = 1.0
    v_junction: float = 5.0           # 교차로 내부 상한 (적색·황색·비신호)
    v_junction_green: float = 35 / 3.6  # 녹색이면 서행할 이유가 없다 → 35 km/h
    v_finish: float = 3.0             # 종료 20 m 전 상한
    # --- 회전 준비 (차로변경·교차로 접근 속도 계획) ---
    prep_horizon: float = 180.0       # 회전 교차로를 이 거리부터 준비한다 [m]
    v_prep: float = 8.0               # 차로변경 시작점부터 회전까지 유지할 속도 [m/s] ≈ 29 km/h
    a_prep: float = 1.2               # 그 속도로 내려갈 때의 감속도 [m/s²] (완만)
    prep_margin: float = 10.0         # 차로변경 시작점 이만큼 전에 v_prep 도달
    stop_gap: float = 0.8             # 정지선 앞 범퍼 여유 (규정: 2 m 미만)
    ped_gap: float = 2.0              # 보행자 앞 '범퍼' 여유 (기준점은 후륜축이므로 별도 보정)
    ped_corridor: float = 1.6         # 차폭 반(0.94) + 여유. 이 안이면 '내 진로 위'.
    ped_stall_s: float = 12.0         # 보행자 때문에 이만큼 멈춰 있으면 서행 통과로 전환.
                                      # 인도에 **서 있는** 사람 앞에서 영구 정지하면
                                      # (v_t<0.5 라 워치독도 리셋된다) 완주가 0 이 된다.
    ped_stop_zone: float = 18.0       # 범퍼 기준 이 거리 안에 보행자가 있으면 v_t=0.
                                      # 감속 프로파일만으로는 1 m/s 로 기어갈 뿐
                                      # 실제로 서지 않는다 (실측: 6.3 m 남고 v=1.7).
    lat_err_slow: float = 1.2         # 횡오차 초과 시 v ≤ 3
    lat_err_recover: float = 3.0      # 횡오차 초과 시 RECOVER
    respawn_jump: float = 5.0         # 한 스텝 이동 > 이 값 = 텔레포트
    teleport_sample: float = 2.5      # 속도추정에서 폐기할 스텝 이동
    dt: float = 0.05                  # lock-step 20 Hz 고정 주기
    signal_lead: float = 30.0         # 회전 지시등 사전 점등 거리 (도로교통법 38조)
    turn_min_deg: float = 20.0
    stuck_l1_s: float = 8.0
    stuck_l2_s: float = 20.0
    stuck_l3_s: float = 60.0
    tl_unknown_wait_s: float = 20.0   # 코드 3/4 정지 대기 상한 → 이후 점멸 취급


@dataclass
class RunConfig:
    host: str = "127.0.0.1"
    port: int = 9910
    cruise: float = 12.5              # m/s = 45 km/h. 저속은 무감점이지만(#4981) 신호 대기가
                                      # 길어져 상향. 제한 50 km/h 아래 5 km/h 여유를 남긴다.
    speed_limit: float = 50 / 3.6
    speed_zone30: float = 30 / 3.6
    xodr: str = str(DEFAULT_XODR)
    log_dir: str = str(ROOT_DIR / "logs")
    connect_timeout_s: float = 600.0  # 접속 재시도 총 시간
    first_packet_s: float = 120.0     # 첫 데이터 대기 (시뮬 시작 → InitDone 지연 대비)
    vehicle: Vehicle = field(default_factory=Vehicle)
    control: Control = field(default_factory=Control)

    @classmethod
    def load(cls, path: str | Path | None) -> "RunConfig":
        cfg = cls()
        if path and Path(path).is_file():
            d = json.loads(Path(path).read_text(encoding="utf-8-sig"))
            for k, v in d.items():
                if k in ("vehicle", "control"):
                    continue
                if hasattr(cfg, k):
                    setattr(cfg, k, v)
        return cfg

    def dump(self) -> dict:
        return asdict(self)
