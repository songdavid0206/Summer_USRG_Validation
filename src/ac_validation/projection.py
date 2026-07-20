"""projection - 상대차 3D 박스를 카메라 이미지에 투영
======================================================

opponent 의 (절대위치, 차체방향 yaw_body, 치수 L×W×H)로 3D cuboid 8꼭짓점을 만들고,
ego 기준 카메라 좌표계로 변환해 이미지 픽셀로 투영한다. nuScenes/Argoverse devkit 의
"3D box를 카메라 이미지에 그리기"와 같은 검증 방식.

좌표계 규약 (BEV 와 동일하게 world 로 통일)
-------------------------------------------
- world  : (x, y) 수평면, z = 높이(미터). ego yaw / opp yaw_body = 진행 heading(rad).
- ego-camera : 두 가지 경로.
    (1) **실측(권장)** — AC `cameras.ini` 온보드 카메라의 실제 위치(전후·좌우·상 오프셋)와
        forward/up 방향을 그대로 쓴다(project_points_extrinsic). 근거리도 정확.
    (2) 근사(fallback) — 카메라를 ego 원점+높이에 두고 고정 pitch 로 본다(project_points).
        전후 오프셋을 무시하므로 **근거리에서 박스가 어긋난다**(cameras.ini 없을 때만).
- 핀홀 투영 : u = cx + f · (right / forward),  v = cy - f · (up / forward)
    f = (이미지폭/2) / tan(hfov/2). forward>0(앞) 인 점만 이미지에 찍힌다.

⚠️ 근거리 박스가 안 맞던 문제(2026-07-14)는 **동기화가 아니라 카메라 캘리브레이션**이었다.
근사 모델이 카메라 전후 오프셋(후드캠 기준 약 0.3m)·차종별 높이/pitch 를 무시해 근거리에서 크게
벌어졌다. → cameras.ini 실측 extrinsic 으로 해결. 온보드 카메라는 차종마다 다르므로 상수 불가.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class CameraPose:
    """온보드 카메라의 차체 로컬 자세(AC cameras.ini). 축: x=우, y=상, z=전방(m)."""
    position: np.ndarray   # (3,) 차 원점 대비 위치
    forward: np.ndarray    # (3,) 바라보는 방향(단위)
    up: np.ndarray         # (3,) 위 방향(단위)

    @classmethod
    def from_ini(cls, cam: dict) -> "CameraPose":
        return cls(np.asarray(cam["position"], float),
                   np.asarray(cam["forward"], float),
                   np.asarray(cam["up"], float))


@dataclass
class RenderCameraPose:
    """CSP가 해당 렌더 프레임에서 보고한 월드 좌표 카메라 자세."""
    position: np.ndarray
    forward: np.ndarray
    up: np.ndarray


@dataclass
class RenderCarPose:
    """CSP가 같은 렌더 tick에서 제공한 차량 full pose와 차 로컬 visual AABB."""
    position: np.ndarray
    forward: np.ndarray
    up: np.ndarray
    aabb_center: np.ndarray  # (forward, right, up)
    aabb_size: np.ndarray    # (length, width, height)


def box_corners_render_car(car: RenderCarPose) -> np.ndarray:
    """차량 visual AABB 8점을 full 6DoF body axes로 월드에 변환."""
    forward = np.asarray(car.forward, float)
    up = np.asarray(car.up, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    cf, cr, cu = np.asarray(car.aabb_center, float)
    length, width, height = np.asarray(car.aabb_size, float)
    center = np.asarray(car.position, float) + cf * forward + cr * right + cu * up
    lf, wr, hu = length / 2, width / 2, height / 2
    local = np.array([[lf, wr, -hu], [lf, -wr, -hu],
                      [-lf, -wr, -hu], [-lf, wr, -hu],
                      [lf, wr, hu], [lf, -wr, hu],
                      [-lf, -wr, hu], [-lf, wr, hu]])
    return (center + local[:, 0, None] * forward
            + local[:, 1, None] * right + local[:, 2, None] * up)


def camera_intrinsics(img_w: int, img_h: int, hfov_deg: float,
                      crop_top: int = 80, full_h: int = 1080):
    """이미지 크기·수평화각에서 (f, cx, cy). region crop 된 이미지의 주점 보정 포함.

    수집이 full_h(1080)에서 위 crop_top(80)px 를 잘라 img_h(945)로 저장하므로,
    광학 중심(원래 이미지 중앙)은 crop 된 이미지에서 cy = full_h/2 - crop_top 이다.
    """
    f = (img_w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    cx = img_w / 2.0
    cy = full_h / 2.0 - crop_top
    return f, cx, cy


def _box_corners_3d(cx: float, cy: float, cz: float, yaw: float,
                    length: float, width: float, height: float,
                    center_offset: tuple[float, float, float] | None = None) -> np.ndarray:
    """차량 3D cuboid 8꼭짓점(월드). cz = 차량 바닥(접지) 높이 기준.

    반환 순서: 아래4(FL,FR,RL,RR) + 위4(FL,FR,RL,RR). 전방=+길이축.
    """
    if center_offset is None:
        center_offset = (0.0, 0.0, height / 2.0)
    forward_offset, left_offset, up_offset = center_offset
    c, s = math.cos(yaw), math.sin(yaw)
    cx += forward_offset * c - left_offset * s
    cy += forward_offset * s + left_offset * c
    cz += up_offset - height / 2.0
    l, w = length / 2.0, width / 2.0
    # 바닥 사각형(local), 전방=+x_local, 왼쪽=+y_local
    ground = np.array([[l, w], [l, -w], [-l, -w], [-l, w]])
    R = np.array([[c, -s], [s, c]])
    xy = (R @ ground.T).T + np.array([cx, cy])          # (4,2) 월드 수평
    bottom = np.column_stack([xy, np.full(4, cz)])       # z = 바닥
    top = np.column_stack([xy, np.full(4, cz + height)])
    return np.vstack([bottom, top])                      # (8,3)


def project_points(pts_world: np.ndarray, ego_xyz: np.ndarray, ego_yaw: float,
                   f: float, cx: float, cy: float,
                   cam_height: float = 0.9, pitch_deg: float = -2.0):
    """월드 3D 점들을 이미지 픽셀로. 반환 (uv (N,2), forward (N,)).

    cam_height : 카메라(드라이버 시점)가 ego 기준점(world_position_z)보다 높은 높이(m).
    pitch_deg  : 카메라가 아래를 보는 각도(+ = 아래). AC 온보드 카메라는 살짝 아래를 본다.
    forward<=0(뒤/옆) 인 점은 uv 가 NaN. 호출측에서 걸러 쓴다.
    """
    cam = ego_xyz + np.array([0.0, 0.0, cam_height])      # 카메라 월드 위치
    rel = pts_world - cam                                 # (N,3)
    c, s = math.cos(ego_yaw), math.sin(ego_yaw)
    F = rel[:, 0] * c + rel[:, 1] * s                     # ego 앞(+) 성분(수평)
    R = rel[:, 0] * s - rel[:, 1] * c                     # ego 오른쪽(+) 성분
    U = rel[:, 2]                                         # 위(+) 성분
    # pitch: 카메라를 아래로 p 만큼 기울인 카메라 프레임으로 회전
    p = math.radians(pitch_deg)
    cp, sp = math.cos(p), math.sin(p)
    Fc = F * cp - U * sp
    Uc = F * sp + U * cp
    with np.errstate(divide="ignore", invalid="ignore"):
        u = cx + f * (R / Fc)
        v = cy - f * (Uc / Fc)
    bad = Fc <= 0.1
    u = np.where(bad, np.nan, u)
    v = np.where(bad, np.nan, v)
    return np.column_stack([u, v]), Fc


def project_points_extrinsic(pts_world: np.ndarray, ego_xyz: np.ndarray, ego_yaw: float,
                             cam: CameraPose, f: float, cx: float, cy: float,
                             ego_pitch: float = 0.0, ego_roll: float = 0.0):
    """월드 3D 점들을 **실측 카메라 자세**(cameras.ini)로 이미지 픽셀에 투영. 반환 (uv, forward).

    project_points 의 근사(카메라=ego중심+높이, 고정 pitch, 전후오프셋 무시)와 달리,
    카메라의 전후/좌우/상 오프셋과 forward/up 방향을 그대로 쓴다 → 근거리도 정확.
    차체 로컬축(x=우,y=상,z=전방)을 ego 자세(yaw+pitch+roll)로 월드에 실어 카메라 좌표계를 만든다.

    ⚠️ ego_pitch/ego_roll 필수(실측 검증, run 20260714_171231): 온보드 카메라는 차체에 붙어
    **트랙 경사·차체 기울기를 그대로 따라간다**("lock to horizon" 설정과 무관하게 pitch 는 차체
    추종이 실측과 일치). 수평 가정(pitch=0)이면 경사 1° 당 박스가 ~18px 뜬다(imola 오르막 2.7°
    에서 ~47px 부양 → pitch 추종으로 해결).
    """
    c, s = math.cos(ego_yaw), math.sin(ego_yaw)
    fh = np.array([c, s, 0.0])        # ego 전방(수평 투영)
    # ⚠️ AC 차체 로컬 +x 는 **왼쪽**이다(오른쪽 아님, 실측 검증: cameras.ini 좌우오프셋을
    # 오른쪽으로 두면 근거리 박스가 반대로 밀림). 그래서 로컬 x → ego 왼쪽으로 매핑.
    lh = np.array([-s, c, 0.0])       # ego 왼쪽(수평)
    uh = np.array([0.0, 0.0, 1.0])    # 위(수직)
    # 차체 자세: pitch(왼쪽축 회전, +=기수 위) → roll(전방축 회전) 순서로 기울인다.
    cp, sp = math.cos(ego_pitch), math.sin(ego_pitch)
    bfw = fh * cp + uh * sp           # 차체 전방(경사 반영)
    bup1 = uh * cp - fh * sp
    cr, sr = math.cos(ego_roll), math.sin(ego_roll)
    blf = lh * cr + bup1 * sr         # 차체 왼쪽(롤 반영)
    bup = bup1 * cr - lh * sr         # 차체 위
    px, py, pz = cam.position
    fx, fy, fz = cam.forward
    ux, uy, uz = cam.up
    cam_w = ego_xyz + pz * bfw + px * blf + py * bup     # 카메라 월드 위치
    cfw = fz * bfw + fx * blf + fy * bup                 # 카메라 전방(월드)
    cup = uz * bfw + ux * blf + uy * bup                 # 카메라 위(월드)
    cri = np.cross(cfw, cup)                             # 카메라 오른쪽(이미지 u+ 방향)
    rel = pts_world - cam_w
    F = rel @ cfw
    R = rel @ cri
    U = rel @ cup
    with np.errstate(divide="ignore", invalid="ignore"):
        u = cx + f * (R / F)
        v = cy - f * (U / F)
    bad = F <= 0.1
    u = np.where(bad, np.nan, u)
    v = np.where(bad, np.nan, v)
    return np.column_stack([u, v]), F


def project_points_render_camera(pts_world: np.ndarray, cam: RenderCameraPose,
                                 f: float, cx: float, cy: float):
    """CSP 실측 월드 카메라 pose로 투영한다. 자차 자세나 이미지 보정값은 사용하지 않는다."""
    cfw = np.asarray(cam.forward, float)
    cup = np.asarray(cam.up, float)
    cfw /= np.linalg.norm(cfw)
    cri = np.cross(cfw, cup)
    cri /= np.linalg.norm(cri)
    cup = np.cross(cri, cfw)
    rel = pts_world - np.asarray(cam.position, float)
    F, R, U = rel @ cfw, rel @ cri, rel @ cup
    with np.errstate(divide="ignore", invalid="ignore"):
        u = cx + f * (R / F)
        v = cy - f * (U / F)
    bad = F <= 0.1
    return np.column_stack([np.where(bad, np.nan, u),
                            np.where(bad, np.nan, v)]), F


def opp_box_yaw(opp_row) -> float:
    """상대차 박스의 yaw 선택(이미지 투영·BEV 공용).

    차체 3D 박스이므로 수집된 차체 방향(yaw_body)을 항상 우선한다. 속도방향 opp_yaw는
    yaw_body가 없는 구형 데이터의 fallback일 뿐이며 코너 슬립/정지에서 차체 자세가 아니다.
    """
    yv = opp_row.get("opp_yaw", np.nan)
    yb = opp_row.get("yaw_body", np.nan)
    if np.isfinite(yb):
        return float(yb)
    return float(yv)


# cuboid 12개 모서리(꼭짓점 인덱스 쌍): 아래4, 위4, 기둥4
_EDGES = [
    (0, 1, "front"), (1, 2, "side"), (2, 3, "rear"), (3, 0, "side"),
    (4, 5, "front"), (5, 6, "side"), (6, 7, "rear"), (7, 4, "side"),
    (0, 4, "front"), (1, 5, "front"), (2, 6, "rear"), (3, 7, "rear"),
]


def opponent_box_edges(opp_row, ego_xyz, ego_yaw, dims, f, cx, cy,
                       cam_height: float = 0.9, pitch_deg: float = -2.0,
                       cam_pose: CameraPose | None = None,
                       bbox_center: tuple[float, float, float] | None = None,
                       ego_pitch: float = 0.0, ego_roll: float = 0.0,
                       render_camera: RenderCameraPose | None = None,
                       render_car: RenderCarPose | None = None):
    """상대차 한 대의 3D 박스를 투영해 그릴 선분 리스트로 반환.

    cam_pose 가 주어지면 **실측 카메라 자세(cameras.ini)** 로 투영(정확). 없으면 기존 근사
    (카메라=ego중심+높이, 고정 pitch) 로 fallback.
    ego_pitch/ego_roll(rad): 차체 자세. 온보드 카메라가 차체를 따라 기울므로 반드시 넘겨야
    경사·기울기에서 박스가 안 뜬다(cam_pose 경로에서만 사용).

    반환: (edges [(uv0,uv1),...], front_face_uv (4,2) 또는 None, all_behind bool, in_view bool)
      - front_face_uv : 앞면 4점(0,1,5,4). 앞면을 채워 방향 표시.
      - in_view       : 차 중심이 화면 안. False 면 near-field/화각밖이라 박스 안 그리는 게 낫다.
    """
    L, W, H = dims
    if render_car is not None:
        corners = box_corners_render_car(render_car)
        center_world = corners.mean(axis=0)
    else:
        ox, oy = float(opp_row["opp_x"]), float(opp_row["opp_y"])
        oz = (float(opp_row["opp_z"])
              if "opp_z" in opp_row and np.isfinite(opp_row["opp_z"]) else ego_xyz[2])
        oyaw = opp_box_yaw(opp_row)
        corners = _box_corners_3d(ox, oy, oz, oyaw, L, W, H, bbox_center)
        center_world = np.array([ox, oy, oz + H / 2])

    def _proj(pts):
        if render_camera is not None:
            return project_points_render_camera(pts, render_camera, f, cx, cy)
        if cam_pose is not None:
            return project_points_extrinsic(pts, ego_xyz, ego_yaw, cam_pose, f, cx, cy,
                                            ego_pitch=ego_pitch, ego_roll=ego_roll)
        return project_points(pts, ego_xyz, ego_yaw, f, cx, cy,
                              cam_height=cam_height, pitch_deg=pitch_deg)

    uv, fwd = _proj(corners)

    # 차 중심을 투영해 '화면 안'인지 판단. 너무 가깝거나(near-field) 화각 밖이면 박스가
    # 거대하게 사방으로 뻗어 무의미하므로, 중심이 화면 밖이면 그리지 않는다.
    ctr_uv, ctr_fwd = _proj(np.array([center_world]))
    img_w = 2 * cx
    in_view = bool(np.isfinite(ctr_uv).all() and -0.1 * img_w <= ctr_uv[0, 0] <= 1.1 * img_w)

    edges = []
    for a, b, kind in _EDGES:
        if np.isfinite(uv[a]).all() and np.isfinite(uv[b]).all():
            edges.append((uv[a], uv[b], kind))
    all_behind = not np.isfinite(uv).any()
    front = uv[[0, 1, 5, 4]] if np.isfinite(uv[[0, 1, 5, 4]]).all() else None
    return edges, front, all_behind, in_view
