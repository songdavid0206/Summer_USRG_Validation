"""checks - 물리 일관성 자동 검증
=================================

수집된 데이터셋이 **물리적으로 자기모순이 없는지** 자동으로 검사한다.
핵심 아이디어: 게임이 주는 여러 채널이 *독립적으로* 저장돼 있으므로,
서로 다른 채널로 같은 물리량을 두 번 구해 일치하는지 본다(교차검증).

⚠️ 순환논리 배제: `ac_dataset` 후처리가 한 채널에서 파생시킨 값끼리 비교하면
(예: opp_vx = speedKMH·cos(yaw) 이므로 |opp_v| == speedKMH) 아무것도 검증되지 않는다.
그래서 여기서는 **원 채널만** 쓴다:
  - 위치(world_position) ↔ 속도(speed / speedKMH)         : 위치를 미분해 속도와 대조
  - 자세(yaw)           ↔ 위치-진행방향                     : 미분한 진행방향과 대조
  - 가속(accelX/Y)      ↔ 속도·각속도 / 속도-미분           : 구심·접선 가속과 대조

단위 주의: AC 의 `accelX`(횡)·`accelY`(종) 는 **g 단위**다(m/s² 아님).
run 20260710_143525 에서 v·yaw_rate = 9.76·accelX 로 확인 → 여기서 G=9.81 로 환산해 비교한다.

프레임레이트(~50Hz)에서 한 프레임 유한차분은 위치 양자화·held 프레임 때문에 노이즈가 크다.
→ 위치·속도 미분은 **중앙차분(±HALF_WIN 프레임)** 으로 구해 노이즈를 평균한다.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pandas as pd

from .loader import RunData

G = 9.81                 # m/s² per g
HALF_WIN = 5             # 중앙차분 반폭(프레임). ±5 → ~0.2s 창, 기준 run 에서 ratio p95<1.05
MIN_SPEED = 3.0          # m/s. 정지 근처는 비율이 발산하므로 '움직이는' 프레임만 통계에 쓴다
MIN_LAT_ACC = 0.3        # g. 횡가속 검증에 쓸 최소 |accelX|(직진 구간 제외)
MIN_MOVING_FRAMES = 30   # 이보다 적으면 그 체크는 통계 불가(skip)


@dataclass
class CheckResult:
    """물리 체크 하나의 결과. per-frame residual 을 남겨 이상프레임 flag·plot 에 재사용."""
    key: str                      # 기계용 식별자
    title: str                    # 사람용 제목(한글)
    unit: str                     # residual 단위
    frame: np.ndarray             # residual 이 정의된 원본 프레임 번호
    measured: np.ndarray          # 독립 채널로 구한 값
    reference: np.ndarray         # 기대값(게임이 직접 준 채널)
    residual: np.ndarray          # measured - reference (같은 길이, 통계대상만)
    stats: dict = field(default_factory=dict)
    passed: bool | None = None
    note: str = ""
    # 이상프레임 flag 대상 여부. residual 이 '주행 방식과 무관하게' 항상 0 근처여야 하는
    # 체크(speed↔위치)만 True. yaw·가속은 코너링 slip·미분노이즈로 residual 이 정상적으로
    # 커지므로 per-frame flag 하면 오검출 → systematic pass/fail 로만 본다.
    flag_frames: bool = False
    anomaly_floor: float = 0.0    # |residual| 이 이보다 작으면 절대 flag 안 함(m/s 등)

    @property
    def n(self) -> int:
        return int(len(self.residual))


def _central_derivative(t: np.ndarray, arr: np.ndarray, k: int = HALF_WIN) -> np.ndarray:
    """arr 의 시간미분을 중앙차분으로. 끝 k 개는 NaN. arr 이 2D(N,2)면 행별 벡터미분."""
    n = len(t)
    if arr.ndim == 1:
        out = np.full(n, np.nan)
    else:
        out = np.full((n, arr.shape[1]), np.nan)
    for i in range(k, n - k):
        dt = t[i + k] - t[i - k]
        if dt > 0:
            out[i] = (arr[i + k] - arr[i - k]) / dt
    return out


def _robust(residual: np.ndarray) -> dict:
    """median / MAD / p95(|·|) 등 이상치에 강한 요약."""
    r = residual[np.isfinite(residual)]
    if len(r) == 0:
        return dict(n=0)
    med = float(np.median(r))
    mad = float(np.median(np.abs(r - med)))
    return dict(
        n=int(len(r)),
        median=med,
        mad=mad,
        mean=float(np.mean(r)),
        std=float(np.std(r)),
        abs_p50=float(np.percentile(np.abs(r), 50)),
        abs_p95=float(np.percentile(np.abs(r), 95)),
        abs_max=float(np.max(np.abs(r))),
    )


# --------------------------------------------------------------------------- #
# ego 체크
# --------------------------------------------------------------------------- #
def check_ego_speed_vs_position(run: RunData) -> CheckResult:
    """|위치 중앙차분| == speed 인가. (위치 채널 ↔ 속도 채널)"""
    e = run.ego[run.ego["valid"]].reset_index(drop=True)
    t = e["t_img"].to_numpy()
    pos = e[["world_position_x", "world_position_y", "world_position_z"]].to_numpy()
    speed = e["speed"].to_numpy()
    v = _central_derivative(t, pos)
    v_mag = np.sqrt(np.nansum(v ** 2, axis=1))
    v_mag[~np.isfinite(v[:, 0])] = np.nan

    m = np.isfinite(v_mag) & (speed > MIN_SPEED)
    resid = v_mag[m] - speed[m]
    stats = _robust(resid)
    if m.sum() >= MIN_MOVING_FRAMES:
        ratio = float(np.median(v_mag[m] / speed[m]))
        stats["ratio_median"] = ratio
        stats["ratio_p05"] = float(np.percentile(v_mag[m] / speed[m], 5))
        stats["ratio_p95"] = float(np.percentile(v_mag[m] / speed[m], 95))
        passed = abs(ratio - 1.0) < 0.03
        note = f"위치미분/스피드 비율 중앙값 {ratio:.4f} (1.0 이면 일치)"
    else:
        passed, note = None, "움직이는 프레임 부족"
    return CheckResult(
        key="ego_speed_vs_position", title="자차 속도 ↔ 위치미분",
        unit="m/s", frame=e["frame"].to_numpy()[m],
        measured=v_mag[m], reference=speed[m], residual=resid,
        stats=stats, passed=passed, note=note,
        flag_frames=True, anomaly_floor=8.0)


def check_ego_heading_vs_yaw(run: RunData) -> CheckResult:
    """위치 진행방향 == yaw 인가. (위치 채널 ↔ 자세 채널)"""
    e = run.ego[run.ego["valid"]].reset_index(drop=True)
    t = e["t_img"].to_numpy()
    pos = e[["world_position_x", "world_position_y"]].to_numpy()
    yaw = e["yaw"].to_numpy()
    speed = e["speed"].to_numpy()
    v = _central_derivative(t, pos)
    heading = np.arctan2(v[:, 1], v[:, 0])
    d = np.arctan2(np.sin(heading - yaw), np.cos(heading - yaw))  # wrap 한 각도차

    m = np.isfinite(d) & (speed > 5.0)   # 저속에선 진행방향이 불안정
    resid = np.degrees(d[m])
    stats = _robust(resid)
    if m.sum() >= MIN_MOVING_FRAMES:
        passed = stats["abs_p95"] < 10.0
        note = f"|yaw−진행방향| 중앙 {stats['abs_p50']:.2f}°, p95 {stats['abs_p95']:.2f}°"
    else:
        passed, note = None, "움직이는 프레임 부족"
    return CheckResult(
        key="ego_heading_vs_yaw", title="자차 자세 ↔ 진행방향",
        unit="deg", frame=e["frame"].to_numpy()[m],
        measured=np.degrees(heading[m]), reference=np.degrees(yaw[m]), residual=resid,
        stats=stats, passed=passed, note=note)


def check_ego_lateral_accel(run: RunData) -> CheckResult:
    """횡가속 accelX·g == speed·yaw_rate(구심가속) 인가. (가속 채널 ↔ 속도×각속도)"""
    e = run.ego[run.ego["valid"]].reset_index(drop=True)
    speed = e["speed"].to_numpy()
    yaw_rate = e["yaw_rate"].to_numpy()
    accX = e["accelX"].to_numpy()
    centripetal = speed * yaw_rate        # v·ω = 구심가속 (m/s²), 부호는 회전방향
    meas = accX * G

    m = np.abs(accX) > MIN_LAT_ACC
    resid = meas[m] - centripetal[m]
    stats = _robust(resid)
    if m.sum() >= MIN_MOVING_FRAMES:
        # accelX 가 g 단위임을 회귀 기울기로 재확인(부호/스케일)
        A = np.vstack([accX[m], np.ones(m.sum())]).T
        slope = float(np.linalg.lstsq(A, centripetal[m], rcond=None)[0][0])
        stats["g_slope"] = slope
        # 기울기는 +g(≈+9.81)여야 한다. 부호가 뒤집히면(좌표 버그) 음수 → FAIL.
        passed = 8.5 < slope < 11.0
        note = f"v·yaw_rate = {slope:.2f}·accelX (≈+9.81 이면 accelX 는 g 단위·부호일치)"
    else:
        passed, note = None, "선회 프레임 부족"
    return CheckResult(
        key="ego_lateral_accel", title="자차 횡가속 ↔ v·yaw_rate",
        unit="m/s²", frame=e["frame"].to_numpy()[m],
        measured=meas[m], reference=centripetal[m], residual=resid,
        stats=stats, passed=passed, note=note)


def check_ego_long_accel(run: RunData) -> CheckResult:
    """종가속 accelY·g == d(speed)/dt(접선가속) 인가. (가속 채널 ↔ 속도-미분)"""
    e = run.ego[run.ego["valid"]].reset_index(drop=True)
    t = e["t_img"].to_numpy()
    speed = e["speed"].to_numpy()
    accY = e["accelY"].to_numpy()
    dspeed = _central_derivative(t, speed)   # 접선가속 (m/s²)
    meas = accY * G

    m = np.isfinite(dspeed) & (np.abs(accY) > MIN_LAT_ACC)
    resid = meas[m] - dspeed[m]
    stats = _robust(resid)
    if m.sum() >= MIN_MOVING_FRAMES:
        corr = float(np.corrcoef(meas[m], dspeed[m])[0, 1])
        stats["corr"] = corr
        passed = corr > 0.85
        note = f"accelY·g 와 d(speed)/dt 상관 {corr:.3f} (미분 노이즈로 resid 넓음)"
    else:
        passed, note = None, "가감속 프레임 부족"
    return CheckResult(
        key="ego_long_accel", title="자차 종가속 ↔ d(speed)/dt",
        unit="m/s²", frame=e["frame"].to_numpy()[m],
        measured=meas[m], reference=dspeed[m], residual=resid,
        stats=stats, passed=passed, note=note)


# --------------------------------------------------------------------------- #
# 상대차 체크
# --------------------------------------------------------------------------- #
def check_opp_speed_vs_position(run: RunData, opp_id: int) -> CheckResult:
    """|상대차 위치 중앙차분| == opp_speedKMH 인가. (위치 채널 ↔ 속도 채널)"""
    o = run.opps[opp_id]
    ov = o[o["valid"]].reset_index(drop=True)
    t = ov["t_img"].to_numpy()
    pos = ov[["opp_x", "opp_y"]].to_numpy()
    speed = ov["opp_speedKMH"].to_numpy() / 3.6
    v = _central_derivative(t, pos)
    v_mag = np.sqrt(np.nansum(v ** 2, axis=1))
    v_mag[~np.isfinite(v[:, 0])] = np.nan

    m = np.isfinite(v_mag) & (speed > MIN_SPEED)
    resid = v_mag[m] - speed[m]
    stats = _robust(resid)
    if m.sum() >= MIN_MOVING_FRAMES:
        ratio = float(np.median(v_mag[m] / speed[m]))
        stats["ratio_median"] = ratio
        stats["ratio_p05"] = float(np.percentile(v_mag[m] / speed[m], 5))
        stats["ratio_p95"] = float(np.percentile(v_mag[m] / speed[m], 95))
        passed = abs(ratio - 1.0) < 0.05
        note = f"위치미분/speedKMH 비율 중앙값 {ratio:.4f}"
    else:
        passed, note = None, "움직이는 프레임 부족(정지/리타이어 위주)"
    return CheckResult(
        key=f"opp{opp_id}_speed_vs_position", title=f"상대차 {opp_id} 속도 ↔ 위치미분",
        unit="m/s", frame=ov["frame"].to_numpy()[m],
        measured=v_mag[m], reference=speed[m], residual=resid,
        stats=stats, passed=passed, note=note,
        flag_frames=True, anomaly_floor=8.0)


def run_all_checks(run: RunData) -> list[CheckResult]:
    """run 전체에 대해 물리 일관성 체크를 모두 수행."""
    results = [
        check_ego_speed_vs_position(run),
        check_ego_heading_vs_yaw(run),
        check_ego_lateral_accel(run),
        check_ego_long_accel(run),
    ]
    for oid in sorted(run.opps):
        results.append(check_opp_speed_vs_position(run, oid))
    return results
