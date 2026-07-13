"""bev - Bird's-Eye-View 렌더러 (검증용)
=========================================

한 프레임의 장면을 top-down 으로 그린다. 좌표계 오류(축·부호·yaw)를 가장 빨리 잡는 도구다.
두 가지 뷰:
  - **global** : 트랙 전체 좌표에서 racing line + ego + opp. 주행 경로·트랙이탈 확인.
  - **ego-centric** : ego 를 원점, 진행방향을 위(↑)로. 근접 상황·FOV·상대차 확인.

좌표 규약 (문서화 — 이걸 고정해야 opp/FOV/궤적이 안 꼬인다)
------------------------------------------------------------
- world  : (x, y) 수평면, 미터. ego yaw / opp yaw_body = 진행 heading(rad),
           forward = (cos yaw, sin yaw), left = (-sin yaw, cos yaw).
- ego-centric plot : ego 를 원점에 두고 진행방향이 화면 위(+Y_plot)로 오게 회전.
           X_plot = ego 기준 오른쪽(+), Y_plot = ego 기준 전방(+).
           즉 forward 성분 → 위, right 성분 → 오른쪽. (자율주행 BEV 관례)

CLI:  av-bev dataset/<run_id> [--frame N] [--view ego|global|both] [--out DIR]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")   # 파일 저장용(창 안 띄움)
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

from .loader import RunData, load_run


# --------------------------------------------------------------------------- 기하
def _rot(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _box_corners(cx: float, cy: float, yaw: float, length: float, width: float) -> np.ndarray:
    """차량 박스 4모서리(월드 좌표). yaw 방향이 길이축(전방)."""
    l, w = length / 2.0, width / 2.0
    local = np.array([[l, w], [l, -w], [-l, -w], [-l, w]])   # FL, FR, RL, RR (전방=+x_local)
    return (_rot(yaw) @ local.T).T + np.array([cx, cy])


def _to_ego_frame(pts: np.ndarray, ego_xy: np.ndarray, ego_yaw: float) -> np.ndarray:
    """월드 점들을 ego-centric plot 좌표로. 반환 (X_plot=right, Y_plot=forward)."""
    rel = pts - ego_xy
    fwd = rel @ np.array([np.cos(ego_yaw), np.sin(ego_yaw)])       # 전방 성분
    left = rel @ np.array([-np.sin(ego_yaw), np.cos(ego_yaw)])     # 왼쪽 성분
    return np.column_stack([-left, fwd])                          # right = -left


# --------------------------------------------------------------------------- 색
_EGO_C = "#1f77b4"      # 파랑 (ego)
_OPP_C = "#2ca02c"      # 초록 (상대차)
_OPP_RET = "#d62728"    # 빨강 (리타이어 — 화각 안에 남으면 강조)
_LINE_C = "#ff7f0e"     # 주황 (racing line)
_EDGE_C = "#999999"     # 회색 (트랙 경계)
_FOV_C = "#1f77b4"


def _draw_box(ax, corners, color, label=None, alpha=0.55):
    ax.add_patch(Polygon(corners, closed=True, facecolor=color, edgecolor="black",
                         linewidth=1.0, alpha=alpha, zorder=5))
    # 전방 표시(앞모서리 중점에 점)
    front = (corners[0] + corners[1]) / 2.0
    ax.plot(front[0], front[1], "o", color="black", ms=3, zorder=6)
    if label is not None:
        c = corners.mean(0)
        ax.text(c[0], c[1], label, fontsize=7, ha="center", va="center",
                color="black", zorder=7)


def _track_edges(rl):
    """racing line + side_left/right 로 좌우 경계 polyline 을 만든다(월드)."""
    x, y = rl["x"].to_numpy(), rl["y"].to_numpy()
    # 라인 접선 → 왼쪽 법선
    th = np.arctan2(np.gradient(y), np.gradient(x))
    nx, ny = -np.sin(th), np.cos(th)
    sL = rl["side_left"].to_numpy() if "side_left" in rl else np.zeros_like(x)
    sR = rl["side_right"].to_numpy() if "side_right" in rl else np.zeros_like(x)
    left = np.column_stack([x + nx * sL, y + ny * sL])
    right = np.column_stack([x - nx * sR, y - ny * sR])
    return np.column_stack([x, y]), left, right


# --------------------------------------------------------------------------- 렌더
def _camera_image_path(run: RunData, i: int) -> Path | None:
    """프레임 i 의 원본 카메라 이미지 절대경로(camera/000350.jpg). 없으면 None."""
    if run.camera is None or i >= len(run.camera):
        return None
    rel = run.camera["image_path"].iloc[i]
    p = run.run_dir / rel
    return p if p.exists() else None


def _export_image_path(run: RunData, i: int) -> Path | None:
    """프레임 i 에 대응하는 **export 폴더**의 이미지(균일격자 ms 이름). 없으면 None.

    export 는 실측 t_img 를 균일 격자(step ms)에 리샘플하므로 파일 이름이 원본과 다르다
    (원본 000350.jpg ↔ export 8750.jpg 처럼). 프레임 i 의 t_img 에 가장 가까운 격자 ms 파일을 찾는다.
    """
    if run.camera is None or i >= len(run.camera):
        return None
    exp_dirs = [d for d in sorted(run.run_dir.parent.glob(run.run_dir.name + "_*"))
                if (d / "CAMERA").is_dir()]
    if not exp_dirs:
        return None
    E = exp_dirs[-1]
    step = None
    info = E / "export_info.json"
    if info.exists():
        step = json.loads(info.read_text(encoding="utf-8")).get("grid_step_ms")
    if not step:
        return None
    t_ms = float(run.camera["t_img"].iloc[i]) * 1000.0
    grid_ms = int(round(t_ms / step) * step)
    for ext in (".jpg", ".png", ".jpeg"):
        p = E / "CAMERA" / ("%d%s" % (grid_ms, ext))
        if p.exists():
            return p
    return None


def render_frame(run: RunData, frame: int, view: str = "ego",
                 fov_deg: float | None = None,
                 past_s: float = 2.0, future_s: float = 3.0,
                 ego_range: tuple[float, float] = (40.0, 60.0),
                 show_image: bool = True):
    """한 프레임을 [카메라 이미지 | BEV] 2패널로 그려 Figure 반환. view = 'ego' | 'global'.

    show_image=True 면 왼쪽에 그 프레임의 실제 카메라 이미지를 함께 보여준다
    (파일 이름도 제목에 표시 → "이 BEV = 이 사진"을 바로 확인).
    """
    ego = run.ego
    i = int(np.clip(frame, 0, len(ego) - 1))
    ex, ey = float(ego["world_position_x"].iloc[i]), float(ego["world_position_y"].iloc[i])
    eyaw = float(ego["yaw"].iloc[i])
    ego_xy = np.array([ex, ey])

    # 프레임 레이트 → past/future 프레임 수
    hz = run.meta.get("image_hz_actual") or run.meta.get("image_hz") or 40.0
    n_past, n_future = int(past_s * hz), int(future_s * hz)

    img_path = _camera_image_path(run, i) if show_image else None
    if img_path is not None:
        fig, (ax_img, ax) = plt.subplots(1, 2, figsize=(16, 8),
                                         gridspec_kw={"width_ratios": [1.2, 1]})
        import matplotlib.image as mpimg
        ax_img.imshow(mpimg.imread(str(img_path)))
        # 원본 이름 + export(최종 산출물) 이름 둘 다 표시 → 어느 폴더를 보든 파일 찾기 쉬움.
        exp_path = _export_image_path(run, i)
        cap = "camera(raw): %s" % img_path.name
        if exp_path is not None:
            cap += "   |   export: %s" % exp_path.name
        ax_img.set_title(cap, fontsize=9)
        ax_img.axis("off")
    else:
        fig, ax = plt.subplots(figsize=(8, 8))

    to = (lambda P: _to_ego_frame(P, ego_xy, eyaw)) if view == "ego" else (lambda P: P)

    # --- 트랙 경계 + racing line ---
    if run.racing_line is not None:
        line, left, right = _track_edges(run.racing_line)
        L_ = to(left); ax.plot(L_[:, 0], L_[:, 1], color=_EDGE_C, lw=1.0, zorder=1,
                               alpha=0.8, label="track edge")
        R_ = to(right); ax.plot(R_[:, 0], R_[:, 1], color=_EDGE_C, lw=1.0, zorder=1, alpha=0.8)
        C_ = to(line); ax.plot(C_[:, 0], C_[:, 1], color=_LINE_C, lw=1.2, zorder=1,
                               alpha=0.9, label="racing line (AI)")

    # --- ego 과거/미래 궤적 ---
    a, b = max(0, i - n_past), min(len(ego), i + n_future + 1)
    traj = ego[["world_position_x", "world_position_y"]].iloc[a:b].to_numpy()
    tj = to(traj)
    cut = i - a
    ax.plot(tj[:cut + 1, 0], tj[:cut + 1, 1], "--", color="black", lw=1.0, alpha=0.5, zorder=3,
            label="ego past %.0fs" % past_s)
    ax.plot(tj[cut:, 0], tj[cut:, 1], "-", color=_EGO_C, lw=1.8, alpha=0.9, zorder=4,
            label="ego future %.0fs" % future_s)

    # --- FOV 콘 (ego 뷰에서만 의미가 큼) ---
    if fov_deg:
        half = np.radians(fov_deg) / 2.0
        R = 80.0
        for sgn in (+1, -1):
            ang = eyaw + sgn * half
            tip = ego_xy + R * np.array([np.cos(ang), np.sin(ang)])
            seg = to(np.array([ego_xy, tip]))
            ax.plot(seg[:, 0], seg[:, 1], ":", color=_FOV_C, lw=1.0, alpha=0.6, zorder=2)

    # --- ego 박스 ---
    L, W, _ = run.car_dim(None)
    _draw_box(ax, to(_box_corners(ex, ey, eyaw, L, W)), _EGO_C, alpha=0.7)

    # --- 상대차 박스 ---
    n_shown = 0
    for oid, o in run.opps.items():
        r = o.iloc[i]
        if not bool(r.get("present", True)):
            continue
        retired = bool(r.get("retired", False))
        ox, oy = float(r["opp_x"]), float(r["opp_y"])
        oyaw = float(r["yaw_body"]) if "yaw_body" in o.columns and np.isfinite(r["yaw_body"]) \
            else float(r["opp_yaw"])
        Lo, Wo, _ = run.car_dim(oid)
        color = _OPP_RET if retired else _OPP_C
        _draw_box(ax, to(_box_corners(ox, oy, oyaw, Lo, Wo)), color, label="%d" % oid)
        n_shown += 1

    # --- 뷰 설정 ---
    if view == "ego":
        ax.set_xlim(-ego_range[0], ego_range[0])
        ax.set_ylim(-ego_range[1] * 0.4, ego_range[1])
        ax.set_xlabel("right (m)"); ax.set_ylabel("forward (m)")
        ax.plot(0, 0, "+", color="black", ms=8, zorder=8)
    else:
        pad = 30.0
        if run.racing_line is not None:
            ax.set_xlim(run.racing_line["x"].min() - pad, run.racing_line["x"].max() + pad)
            ax.set_ylim(run.racing_line["y"].min() - pad, run.racing_line["y"].max() + pad)
        ax.set_xlabel("world x (m)"); ax.set_ylabel("world y (m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)
    # 범례용 프록시(박스 색)
    from matplotlib.patches import Patch
    handles, labels = ax.get_legend_handles_labels()
    handles += [Patch(facecolor=_EGO_C, edgecolor="black", label="ego"),
                Patch(facecolor=_OPP_C, edgecolor="black", label="opponent"),
                Patch(facecolor=_OPP_RET, edgecolor="black", label="opp retired")]
    ax.legend(handles=handles, loc="upper right", fontsize=7, framealpha=0.85)

    spd = float(ego["speed"].iloc[i]) if "speed" in ego else float("nan")
    ax.set_title("%s  frame %d  |  %s view  |  %.1f m/s (%.0f km/h)  |  %d opp"
                 % (run.track_name, i, view, spd, spd * 3.6, n_shown), fontsize=10)
    fig.tight_layout()
    return fig


def render_bev(run_dir, frame: int = -1, view: str = "ego", out_dir=None,
               fov_deg: float = 86.8):
    """run 의 한 프레임을 BEV png 로 저장. view='both' 면 두 장. frame=-1 이면 중앙 프레임."""
    run = load_run(run_dir)
    if frame < 0:
        frame = run.n_frames // 2
    out_dir = Path(out_dir) if out_dir else Path(run.run_dir).parent / "validation" / run.run_id / "bev"
    out_dir.mkdir(parents=True, exist_ok=True)

    # 이 프레임의 이미지 이름을 알려준다(원본 + export 최종 산출물 둘 다).
    fi = int(np.clip(frame, 0, run.n_frames - 1))
    img = _camera_image_path(run, fi)
    if img is not None:
        print("[bev] frame %d  원본: %s" % (frame, img))
    exp = _export_image_path(run, fi)
    if exp is not None:
        print("[bev] frame %d  export(최종): %s" % (frame, exp))

    views = ["ego", "global"] if view == "both" else [view]
    saved = []
    for v in views:
        fig = render_frame(run, frame, view=v, fov_deg=fov_deg if v == "ego" else None)
        p = out_dir / ("bev_%s_frame%06d.png" % (v, frame))
        fig.savefig(p, dpi=110)
        plt.close(fig)
        saved.append(p)
        print("[bev] %s" % p)
    return saved


def main():
    ap = argparse.ArgumentParser(description="BEV 렌더러 (데이터셋 검증용)")
    ap.add_argument("run_dir", help="dataset/<run_id>")
    ap.add_argument("--frame", type=int, default=-1, help="프레임 번호(기본 -1=중앙)")
    ap.add_argument("--view", default="both", choices=["ego", "global", "both"])
    ap.add_argument("--out", default=None, help="출력 폴더")
    ap.add_argument("--fov", type=float, default=86.8, help="수평 화각(도)")
    args = ap.parse_args()
    render_bev(args.run_dir, frame=args.frame, view=args.view, out_dir=args.out, fov_deg=args.fov)


if __name__ == "__main__":
    main()
