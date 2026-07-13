"""projection - 상대차 3D 박스를 카메라 이미지에 투영
======================================================

opponent 의 (절대위치, 차체방향 yaw_body, 치수 L×W×H)로 3D cuboid 8꼭짓점을 만들고,
ego 기준 카메라 좌표계로 변환해 이미지 픽셀로 투영한다. nuScenes/Argoverse devkit 의
"3D box를 카메라 이미지에 그리기"와 같은 검증 방식.

좌표계 규약 (BEV 와 동일하게 world 로 통일)
-------------------------------------------
- world  : (x, y) 수평면, z = 높이(미터). ego yaw / opp yaw_body = 진행 heading(rad).
- ego-camera : 카메라가 ego 원점, ego 진행방향(yaw)을 바라본다고 근사.
    forward = ego 앞(+), right = ego 오른쪽(+), up = +z.
    (드라이버 시점 오프셋·pitch 는 근사로 무시. 먼 거리에선 영향 작음.)
- 핀홀 투영 : u = cx + f · (right / forward),  v = cy - f · (up / forward)
    f = (이미지폭/2) / tan(hfov/2). forward>0(앞) 인 점만 이미지에 찍힌다.

주의: BEV(정확)와 달리 이미지 투영은 카메라 높이·pitch 근사가 들어가므로, 세로(v) 위치가
약간 어긋날 수 있다. 가로(u)·좌우 크기는 신뢰도가 높다(BEV 투영으로 이미 검증).
"""

from __future__ import annotations

import math

import numpy as np


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
                    length: float, width: float, height: float) -> np.ndarray:
    """차량 3D cuboid 8꼭짓점(월드). cz = 차량 바닥(접지) 높이 기준.

    반환 순서: 아래4(FL,FR,RL,RR) + 위4(FL,FR,RL,RR). 전방=+길이축.
    """
    l, w = length / 2.0, width / 2.0
    # 바닥 사각형(local), 전방=+x_local, 왼쪽=+y_local
    ground = np.array([[l, w], [l, -w], [-l, -w], [-l, w]])
    c, s = math.cos(yaw), math.sin(yaw)
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


# cuboid 12개 모서리(꼭짓점 인덱스 쌍): 아래4, 위4, 기둥4
_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0),
          (4, 5), (5, 6), (6, 7), (7, 4),
          (0, 4), (1, 5), (2, 6), (3, 7)]


def opponent_box_edges(opp_row, ego_xyz, ego_yaw, dims, f, cx, cy,
                       cam_height: float = 0.9, pitch_deg: float = -2.0):
    """상대차 한 대의 3D 박스를 투영해 그릴 선분 리스트로 반환.

    반환: (edges [(uv0,uv1),...], front_face_uv (4,2) 또는 None, all_behind bool, in_view bool)
      - front_face_uv : 앞면 4점(0,1,5,4). 앞면을 채워 방향 표시.
      - in_view       : 차 중심이 화면 안. False 면 near-field/화각밖이라 박스 안 그리는 게 낫다.
    """
    L, W, H = dims
    ox, oy = float(opp_row["opp_x"]), float(opp_row["opp_y"])
    oz = float(opp_row["opp_z"]) if "opp_z" in opp_row and np.isfinite(opp_row["opp_z"]) else ego_xyz[2]
    oyaw = float(opp_row["yaw_body"]) if "yaw_body" in opp_row and np.isfinite(opp_row["yaw_body"]) \
        else float(opp_row["opp_yaw"])
    corners = _box_corners_3d(ox, oy, oz, oyaw, L, W, H)   # (8,3)
    uv, fwd = project_points(corners, ego_xyz, ego_yaw, f, cx, cy,
                             cam_height=cam_height, pitch_deg=pitch_deg)

    # 차 중심을 투영해 '화면 안'인지 판단. 너무 가깝거나(near-field) 화각 밖이면 박스가
    # 거대하게 사방으로 뻗어 무의미하므로, 중심이 화면 밖이면 그리지 않는다.
    ctr_uv, ctr_fwd = project_points(np.array([[ox, oy, oz + H / 2]]), ego_xyz, ego_yaw,
                                     f, cx, cy, cam_height=cam_height, pitch_deg=pitch_deg)
    img_w = 2 * cx
    in_view = bool(np.isfinite(ctr_uv).all() and -0.1 * img_w <= ctr_uv[0, 0] <= 1.1 * img_w)

    edges = []
    for a, b in _EDGES:
        if np.isfinite(uv[a]).all() and np.isfinite(uv[b]).all():
            edges.append((uv[a], uv[b]))
    all_behind = not np.isfinite(uv).any()
    front = uv[[0, 1, 5, 4]] if np.isfinite(uv[[0, 1, 5, 4]]).all() else None
    return edges, front, all_behind, in_view
