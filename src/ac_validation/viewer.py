"""viewer - 인터랙티브 데이터셋 검증 뷰어 (av-view)
====================================================

한 run 을 프레임 단위로 넘겨보며 검증한다. 조교 요청대로 **미리 저장하지 않고**, 실행하면
그 프레임만 즉석에서 그린다. 한 프레임 = 3패널:

  [ 카메라 이미지 + 3D bbox 투영 ]  |  [ BEV(ego-centric) ]  |  [ ego 검수 plot ]

조작
----
  ← / →      : 이전 / 다음 프레임 (Shift 누르면 10프레임씩)
  Home / End : 처음 / 마지막
  s          : 현재 프레임 png 저장
  q          : 종료

CLI:  av-view dataset/<run_id> [--frame N] [--step 1]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from matplotlib.patches import Polygon

from .loader import RunData, load_run
from .projection import camera_intrinsics, opponent_box_edges
from . import bev as bevmod


_OPP_C = "#2ca02c"
_OPP_RET = "#d62728"


class Viewer:
    def __init__(self, run: RunData, frame: int = 0, step: int = 1,
                 fov_deg: float = 86.78):
        self.run = run
        self.i = int(np.clip(frame, 0, run.n_frames - 1))
        self.step = step
        self.fov = fov_deg
        self.f, self.cx, self.cy = camera_intrinsics(1920, 945, fov_deg)

        # matplotlib 기본 단축키가 화살표(뒤로/앞으로 뷰) 등을 가로채지 않도록 비운다.
        for k in ("keymap.back", "keymap.forward", "keymap.pan", "keymap.zoom",
                  "keymap.home", "keymap.save", "keymap.quit"):
            try:
                plt.rcParams[k] = []
            except Exception:
                pass

        self.fig = plt.figure(figsize=(19, 7))
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        # 3패널: 이미지(넓게) | BEV | ego plots. plot 4개는 세로로, 제목이 아래 눈금과
        # 안 겹치게 간격(0.235)을 높이(0.15)보다 넉넉히 둔다.
        self.ax_img = self.fig.add_axes([0.01, 0.05, 0.40, 0.86])
        self.ax_bev = self.fig.add_axes([0.44, 0.06, 0.26, 0.85])
        self.ax_p = [self.fig.add_axes([0.76, 0.75 - k * 0.22, 0.22, 0.15]) for k in range(4)]
        self._draw()

    # ------------------------------------------------------------------ 그리기
    def _draw(self):
        i = self.i
        run = self.run
        ego = run.ego.iloc[i]
        ego_xyz = np.array([ego.world_position_x, ego.world_position_y, ego.world_position_z])
        eyaw = float(ego.yaw)

        # --- 패널 1: 카메라 이미지 + 3D bbox ---
        self.ax_img.clear()
        img_path = bevmod._camera_image_path(run, i)
        exp_path = bevmod._export_image_path(run, i)
        if img_path is not None:
            self.ax_img.imshow(mpimg.imread(str(img_path)))
        n_box = 0
        for oid, o in run.opps.items():
            r = o.iloc[i]
            if not bool(r.get("present", True)) or bool(r.get("retired", False)):
                continue
            edges, front, behind, in_view = opponent_box_edges(
                r, ego_xyz, eyaw, run.car_dim(oid), self.f, self.cx, self.cy)
            if behind or not in_view:
                continue
            for a, b in edges:
                self.ax_img.plot([a[0], b[0]], [a[1], b[1]], color=_OPP_C, lw=1.5, zorder=5)
            if front is not None:
                self.ax_img.add_patch(Polygon(front, closed=True, fc=_OPP_C,
                                              alpha=0.15, ec="none", zorder=4))
            c = np.nanmean([p for e in edges for p in e], axis=0)
            self.ax_img.text(c[0], c[1], str(oid), color="yellow", fontsize=12,
                             weight="bold", zorder=6)
            n_box += 1
        self.ax_img.set_xlim(0, 1920); self.ax_img.set_ylim(945, 0)
        self.ax_img.axis("off")
        cap = "raw: %s" % (img_path.name if img_path else "?")
        if exp_path is not None:
            cap += "   |   export: %s" % exp_path.name
        self.ax_img.set_title("image + 3D bbox (%d cars)   %s" % (n_box, cap), fontsize=9)

        # --- 패널 2: BEV (ego-centric) ---
        self.ax_bev.clear()
        self._draw_bev(self.ax_bev, i)

        # --- 패널 3: ego 검수 plot ---
        self._draw_plots(i)

        self.fig.suptitle("%s  |  frame %d / %d   (arrows or a/d: move,  Shift: x10,  s: save,  q: quit)"
                          % (run.track_name, i, run.n_frames - 1), fontsize=12)
        self.fig.canvas.draw_idle()

    def _draw_bev(self, ax, i):
        """bev 모듈의 렌더 로직을 축 하나에 재사용(ego-centric)."""
        run = self.run
        ego = run.ego.iloc[i]
        ex, ey, eyaw = float(ego.world_position_x), float(ego.world_position_y), float(ego.yaw)
        ego_xy = np.array([ex, ey])
        to = lambda P: bevmod._to_ego_frame(P, ego_xy, eyaw)
        if run.racing_line is not None:
            line, left, right = bevmod._track_edges(run.racing_line)
            for seg, c in ((left, "#999"), (right, "#999"), (line, "#ff7f0e")):
                s = to(seg); ax.plot(s[:, 0], s[:, 1], color=c, lw=1.0, alpha=0.8)
        hz = run.meta.get("image_hz_actual") or 40.0
        a, b = max(0, i - int(2 * hz)), min(run.n_frames, i + int(3 * hz) + 1)
        tj = to(run.ego[["world_position_x", "world_position_y"]].iloc[a:b].to_numpy())
        cut = i - a
        ax.plot(tj[:cut + 1, 0], tj[:cut + 1, 1], "--", color="black", lw=1.0, alpha=0.5)
        ax.plot(tj[cut:, 0], tj[cut:, 1], "-", color="#1f77b4", lw=1.8)
        for sgn in (+1, -1):
            ang = eyaw + sgn * np.radians(self.fov) / 2
            tip = ego_xy + 80 * np.array([np.cos(ang), np.sin(ang)])
            s = to(np.array([ego_xy, tip])); ax.plot(s[:, 0], s[:, 1], ":", color="#1f77b4", alpha=0.6)
        L, W, _ = run.car_dim(None)
        bevmod._draw_box(ax, to(bevmod._box_corners(ex, ey, eyaw, L, W)), "#1f77b4",
                         label="ego", alpha=0.7)
        for oid, o in run.opps.items():
            r = o.iloc[i]
            if not bool(r.get("present", True)):
                continue
            oyaw = float(r["yaw_body"]) if np.isfinite(r.get("yaw_body", np.nan)) else float(r["opp_yaw"])
            Lo, Wo, _ = run.car_dim(oid)
            col = _OPP_RET if bool(r.get("retired", False)) else _OPP_C
            bevmod._draw_box(ax, to(bevmod._box_corners(float(r["opp_x"]), float(r["opp_y"]),
                             oyaw, Lo, Wo)), col, label=str(oid))
        ax.set_xlim(-40, 40); ax.set_ylim(-20, 60); ax.set_aspect("equal")
        ax.grid(True, alpha=0.2); ax.plot(0, 0, "+", color="black", ms=8)
        ax.set_xlabel("right (m)"); ax.set_ylabel("forward (m)")
        ax.set_title("BEV (ego-centric)", fontsize=9)

    def _draw_plots(self, i):
        """EGO 검수 plot: 속도 / 조작값 / yaw_rate / 가속도(모두 자차).
        x축 = 시간(초), 빨간 세로선 = 현재 프레임."""
        run = self.run
        e = run.ego
        t = e["t_img"].to_numpy() if "t_img" in e else np.arange(len(e))
        specs = [
            ("EGO speed (m/s)", [("speed", "#1f77b4")]),
            ("EGO controls (label)", [("throttle", "#2ca02c"), ("brake", "#d62728"), ("steer", "#1f77b4")]),
            ("EGO yaw_rate (rad/s)", [("yaw_rate", "#9467bd")]),
            ("EGO accel (G)", [("accelY", "#ff7f0e"), ("accelX", "#8c564b")]),
        ]
        for k, (ax, (title, cols)) in enumerate(zip(self.ax_p, specs)):
            ax.clear()
            for col, c in cols:
                if col in e.columns:
                    ax.plot(t, e[col].to_numpy(), color=c, lw=0.7, label=col)
            ax.axvline(t[i], color="red", lw=1.2)
            ax.set_title(title, fontsize=8)
            ax.tick_params(labelsize=6)
            if k == len(self.ax_p) - 1:            # 맨 아래에만 x축 라벨
                ax.set_xlabel("time (s)", fontsize=7)
            if len(cols) > 1:                      # 채널 여럿일 때만 범례
                ax.legend(fontsize=5, loc="upper right")
        # "current frame" 은 범례 대신 첫 plot 바로 위에 빨간 글씨로.
        self.ax_p[0].text(0.0, 1.28, "red line = current frame", transform=self.ax_p[0].transAxes,
                          color="red", fontsize=7, ha="left", va="bottom")

    # ------------------------------------------------------------------ 조작
    def _on_key(self, event):
        n = self.run.n_frames
        big = self.step * 10
        k = event.key
        if k in ("right", "d", "."):
            self.i = min(n - 1, self.i + self.step)
        elif k in ("left", "a", ","):
            self.i = max(0, self.i - self.step)
        elif k in ("shift+right", "D", "shift+."):
            self.i = min(n - 1, self.i + big)
        elif k in ("shift+left", "A", "shift+,"):
            self.i = max(0, self.i - big)
        elif k == "home":
            self.i = 0
        elif k == "end":
            self.i = n - 1
        elif k == "s":
            self._save()
            return
        elif k == "q":
            plt.close(self.fig)
            return
        else:
            return
        self._draw()

    def _save(self):
        out = Path(self.run.run_dir).parent / "validation" / self.run.run_id
        out.mkdir(parents=True, exist_ok=True)
        p = out / ("frame%06d.png" % self.i)
        self.fig.savefig(p, dpi=110)
        print("[view] 저장: %s" % p)


def main():
    ap = argparse.ArgumentParser(description="인터랙티브 데이터셋 검증 뷰어")
    ap.add_argument("run_dir", help="dataset/<run_id>")
    ap.add_argument("--frame", type=int, default=0, help="시작 프레임")
    ap.add_argument("--step", type=int, default=1, help="←/→ 이동 간격")
    ap.add_argument("--fov", type=float, default=86.78, help="수평 화각(도)")
    args = ap.parse_args()
    # 인터랙티브 창이 필요. Agg(저장전용)로 잡혀 있으면 창 띄우는 백엔드로 전환 시도.
    if matplotlib.get_backend().lower() == "agg":
        for b in ("TkAgg", "Qt5Agg"):
            try:
                matplotlib.use(b, force=True)
                break
            except Exception:
                continue
    run = load_run(args.run_dir)
    # 인스턴스를 반드시 변수로 붙잡아 둔다. matplotlib 은 키 콜백(bound method)을
    # 약한 참조로 저장하므로, Viewer 를 어디에도 안 담으면 GC 로 사라져 키가 안 먹는다.
    viewer = Viewer(run, frame=args.frame, step=args.step, fov_deg=args.fov)
    plt.show()
    return viewer


if __name__ == "__main__":
    main()
