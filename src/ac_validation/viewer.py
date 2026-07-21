"""viewer - 인터랙티브 데이터셋 검증 뷰어 (av-view)
====================================================

한 run 을 프레임 단위로 넘겨보며 검증한다. 조교 요청대로 **미리 저장하지 않고**, 실행하면
그 프레임만 즉석에서 그린다. 창은 전체화면으로 띄운다. 레이아웃:

  [ 카메라 이미지 + 3D bbox ]   [ 물리검증 요약 ]   [ ego 검수 plot 4개 ]
  [ 전체맵 | 로컬 BEV       ]

- 전체 트랙 맵(좌하-좌): 트랙 전체에서 ego·상대차·지나온 경로. 트랙이탈/위치 확인.
- 로컬 BEV(좌하-우): ego-centric, 근접 상황·FOV·상대차.
- 물리검증 요약(가운데): av-check 물리 일관성 PASS/FAIL + 핵심 수치(프레임 무관 고정).
- ego 검수 plot(우) 의 controls 패널은 throttle/brake([0,1], 왼쪽축)와 steer(도, 오른쪽 twin 축)를
  스케일이 달라 축을 나눠 그린다(한 축에 겹치면 throttle/brake 가 바닥에 깔림).

조작
----
  ← / →      : 이전 / 다음 프레임 (Shift 누르면 50프레임씩)
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
import pandas as pd
from matplotlib.patches import Polygon

from .loader import RunData, load_run
from .projection import (CameraPose, RenderCameraPose, RenderCarPose, camera_intrinsics,
                         opponent_box_edges)
from . import bev as bevmod


_OPP_C = "#2ca02c"
_OPP_RET = "#d62728"


def _bbox_row_visible(row) -> bool:
    """Return whether an opponent is eligible for a camera training label."""
    return (bool(row.get("present", True))
            and not bool(row.get("retired", False))
            and bool(row.get("valid", True)))


def _layout_image_labels(labels, width=1920, height=945,
                         label_width=24, label_height=20):
    """Place image labels without overlap, staying inside the captured image."""
    placed = []
    out = []
    margin_x, margin_y = label_width / 2 + 2, label_height / 2 + 2

    def clamp(x, y):
        return (float(np.clip(x, margin_x, width - margin_x)),
                float(np.clip(y, margin_y, height - margin_y)))

    def free(x, y):
        return all(abs(x - px) >= label_width or abs(y - py) >= label_height
                   for px, py in placed)

    for oid, ax, ay in labels:
        candidates = [(0, 0)]
        for ring in range(1, 13):
            dx, dy = ring * label_width, ring * label_height
            candidates.extend([(0, -dy), (dx, 0), (0, dy), (-dx, 0),
                               (dx, -dy), (dx, dy), (-dx, dy), (-dx, -dy)])
        pos = None
        for dx, dy in candidates:
            x, y = clamp(ax + dx, ay + dy)
            if free(x, y):
                pos = (x, y)
                break
        if pos is None:
            for y in np.arange(margin_y, height - margin_y + 1, label_height):
                for x in np.arange(margin_x, width - margin_x + 1, label_width):
                    if free(float(x), float(y)):
                        pos = (float(x), float(y))
                        break
                if pos is not None:
                    break
        if pos is None:
            pos = clamp(ax, ay)
        placed.append(pos)
        out.append((oid, (float(ax), float(ay)), pos))
    return out


class Viewer:
    def __init__(self, run: RunData, frame: int = 0, step: int = 1,
                 fov_deg: float | None = None, camera_index: int | None = None,
                 tau_ms: float = 0.0):
        self.run = run
        self.i = int(np.clip(frame, 0, run.n_frames - 1))
        self.step = step
        # 캡처 지연 τ: 게임시간 페어링(build_dataset)이 이미 이미지 게임시간에 telemetry 를
        # 맞춰두므로 기본 τ=0. (구버전 벽시계-페어링 run 을 볼 때만 --tau-ms 로 보정. 예전엔
        # 30ms 를 썼으나 그건 페어링 잔차 보정용이었고, 게임시간 페어링에선 이중보정이 된다.)
        self.tau_s = float(tau_ms) / 1000.0
        self._t = run.ego["t_img"].to_numpy() if "t_img" in run.ego.columns else None
        pose_cols = {"cam_pos_x", "cam_pos_y", "cam_pos_z", "cam_forward_x",
                     "cam_forward_y", "cam_forward_z", "cam_up_x", "cam_up_y",
                     "cam_up_z", "cam_fov_deg"}
        self.has_render_camera = (run.camera is not None
                                  and pose_cols <= set(run.camera.columns)
                                  and run.camera[list(pose_cols)].notna().all(axis=1).any())
        self.has_render_cars = bool(run.render_cars)
        # roll 적용 여부는 카메라 확정 후 아래에서 세팅(camera_onboard.ini 의 lock 설정 기준).
        self._world_aligned = bool(run.meta.get("camera_world_aligned", 1))
        # 3D bbox 투영 카메라: AC cameras.ini/dash_cam.ini 의 실측 온보드 카메라.
        # camera_index 를 명시 안 하면 카메라를 자동 선택한다. 우선순위:
        #   (1) meta['onboard_camera_index'] — 수집 때 사용자가 --onboard-camera 로 선언한 값.
        #       게임은 활성 F6 온보드캠을 노출하지 않으므로(camera_drivable_mode 는 F1 뷰만,
        #       telemetry 엔 없음) 이 선언값이 **유일하게 신뢰할 수 있는** 카메라 정보다.
        #   (2) (구버전 run 폴백) camera_drivable_mode 휴리스틱 — 신뢰 낮음(F6 미반영/stale).
        cams = run.onboard_cams or []
        auto_note = ""
        if camera_index is None:
            oc = run.meta.get("onboard_camera_index")
            if oc is not None and 0 <= int(oc) < len(cams):
                camera_index = int(oc)
                auto_note = "(meta:onboard_camera_index)"
            else:
                camera_index = 1
                dm = run.meta.get("camera_drivable_mode")
                if run.meta.get("camera_mode") == 2 and dm == 4:
                    camera_index = next((c["idx"] for c in cams if c.get("name") == "DASH"), 1)
                    auto_note = "(폴백:drivable_mode, 부정확 — --camera 권장)"
                elif run.meta.get("camera_mode") == 2 and dm == 2:
                    camera_index = 1
                    auto_note = "(폴백:drivable_mode, 부정확 — --camera 권장)"
                elif "camera_mode" in run.meta:
                    auto_note = "(폴백:기본값 CAMERA_1 — 다르면 --camera)"
                else:
                    auto_note = "(기본값, meta 에 카메라 기록 없음 — 다르면 --camera)"
        self.cam_pose = None
        cam_fov = None
        if 0 <= camera_index < len(cams):
            self.cam_pose = CameraPose.from_ini(cams[camera_index])
            self.cam_name = cams[camera_index].get("name", f"CAMERA_{camera_index}")
            cam_fov = cams[camera_index].get("fov")
        else:
            self.cam_name = "approx(no cameras.ini)"
        # roll: "Lock onboard camera to horizon"(IS_WORLD_ALIGNED) 은 **F6 온보드캠에도 먹는다**
        # → lock ON 이면 카메라는 수평 유지, 차만 기움 → ego_roll 을 적용하면 안 된다.
        # 근거(run 20260716_160824, lock ON, roll +6.13° vs 0.00°):
        #   · 화면 지평선은 수평 유지(6° 기울면 1900px 폭에서 ~200px 기울어야 하는데 ~25px)
        #   · 자차 바디워크는 roll 만큼 기움(두 프레임 바디워크 픽셀차 45/255, Shell 노즈가 눈에
        #     띄게 회전) → 카메라가 안 기울고 차만 기운다는 뜻
        self.apply_roll = not self._world_aligned
        self.roll_src = f"meta world_aligned={int(self._world_aligned)}"

        # FOV(수직) 우선순위 — **카메라마다 다르다**:
        #   (1) --fov 로 명시
        #   (2) cameras.ini 의 그 카메라 FOV — F6 온보드캠(CAMERA_N)은 **자체 FOV**(예: 60)로
        #       렌더된다. 게임 설정(Video>View)의 온보드 FOV 슬라이더는 F1 콕핏뷰에만 먹고
        #       F6 캠엔 안 먹는다(실측: 56 을 쓰면 f 가 8% 커져 화면 가장자리·원거리 박스가
        #       주점 바깥으로 밀려 '뜬다'. 60 이면 원거리도 차에 앉음).
        #   (3) meta 의 수집시 온보드 FOV 설정(DASH 등 자체 FOV 없는 카메라용). 없으면 56.
        fov_src = "--fov 지정"
        if fov_deg is None:
            if self.has_render_camera:
                vfov = float(run.camera.loc[run.camera["cam_fov_deg"].notna(),
                                             "cam_fov_deg"].iloc[0])
                fov_src = f"CSP render FOV={vfov:.1f}"
            elif cam_fov:
                vfov = float(cam_fov)
                fov_src = f"cameras.ini {self.cam_name} FOV={vfov:.0f}"
            else:
                vfov = float(run.meta.get("onboard_fov_deg") or 56.0)
                fov_src = f"meta onboard_fov_deg={vfov:.0f}"
            fov_deg = float(np.degrees(2 * np.arctan(np.tan(np.radians(vfov) / 2) * 1920 / 1080)))
        self.fov = fov_deg
        self.fov_src = fov_src
        self.f, self.cx, self.cy = camera_intrinsics(1920, 945, fov_deg)
        # 항상 출력 — 이 줄이 안 뜨거나 버전이 다르면 옛 코드가 도는 것(최신 코드 지문 역할).
        projection_source = ("CSP 실제 렌더 pose" if self.has_render_camera
                             else f"{self.cam_name} {auto_note or '(--camera 지정)'}")
        box_source = ("visual AABB/full 6DoF" if self.has_render_cars
                      else "collider/yaw fallback")
        print(f"[view/v14] 투영 카메라 = {projection_source}, "
              f"hfov={self.fov:.2f}°(f={self.f:.0f}, {self.fov_src}), "
              f"pitch+roll={'ON' if self.apply_roll else 'OFF'}({self.roll_src}), "
              f"τ={self.tau_s*1000:.0f}ms, box={box_source}")
        if not self.has_render_camera:
            print("[view] WARNING: 이 run에는 CSP 실제 카메라 pose가 없어 cameras.ini 재구성 fallback을 사용합니다.")
        print(f"[view] 상대차 3D pose = "
              f"{'CSP same-render full 6DoF + visual AABB' if self.has_render_cars else 'telemetry yaw-only + collider fallback'}")
        # 수집 당시 카메라 흔들림이 켜져 있었으면 박스가 프레임별로 어긋난다 → 경고.
        mix = str(run.meta.get("camera_gforce_mix") or "")
        shake = str(run.meta.get("camera_shake_scale") or "")
        for name, val in (("G-Force", mix), ("Shake", shake)):
            try:
                if val and any(abs(float(x)) > 1e-6 for x in val.split(",")):
                    print(f"[view] ⚠️ 수집 당시 카메라 {name} 흔들림이 켜져 있었음({val}) — "
                          "고g 프레임에서 박스가 어긋날 수 있음(게임 설정에서 0 권장)")
            except ValueError:
                pass

        # matplotlib 기본 단축키가 화살표(뒤로/앞으로 뷰) 등을 가로채지 않도록 비운다.
        for k in ("keymap.back", "keymap.forward", "keymap.pan", "keymap.zoom",
                  "keymap.home", "keymap.save", "keymap.quit"):
            try:
                plt.rcParams[k] = []
            except Exception:
                pass

        # 한글 라벨(물리 요약·패널 제목)용 폰트. 없으면 첫 후보로 폴백.
        for _f in ("Malgun Gothic", "AppleGothic", "NanumGothic", "DejaVu Sans"):
            if any(_f == f.name for f in matplotlib.font_manager.fontManager.ttflist):
                plt.rcParams["font.family"] = _f
                break
        plt.rcParams["axes.unicode_minus"] = False

        self.fig = plt.figure(figsize=(20, 11))
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        # 레이아웃(전체화면 기준):
        #   좌측 열  : 카메라 이미지(위, 크게) + [전체맵 | 로컬 BEV](아래 나란히)
        #   가운데 열: 물리검증 요약(av-check, 고정)
        #   우측 열  : ego 검수 plot 4개(세로로)
        # 상단 전체 제목과 패널 제목이 작은 화면/Windows 배율에서도 겹치지
        # 않도록 위쪽에 독립된 헤더 여백을 둔다.
        self.ax_img = self.fig.add_axes([0.02, 0.53, 0.42, 0.385])   # 카메라 + 3D bbox
        # 좌하: 전체맵(좌) + 로컬 BEV(우). 전체맵은 트랙 종횡비에 맞춰 크기.
        # 두 지도가 같은 시각 기호를 쓰므로 하단의 공통 범례 하나를 공유한다.
        # 지도와 범례의 figure 영역은 물리적으로 분리되어 서로 가릴 수 없다.
        self.ax_glob = self.fig.add_axes(
            self._fit_rect((0.015, 0.105, 0.215, 0.355), self._track_aspect()))
        # x축 라벨까지 범례와 분리되도록 지도와 범례 사이에 넉넉한 간격을 둔다.
        self.ax_bev = self.fig.add_axes([0.25, 0.14, 0.19, 0.32])    # BEV ego-centric(로컬)
        self.ax_scene_legend = self.fig.add_axes([0.015, 0.012, 0.425, 0.075])
        self.ax_scene_legend.axis("off")
        # 가운데: 물리 일관성(av-check). 프레임 무관 고정 → __init__ 에서 1회 렌더.
        #   체크마다 measured vs reference 산점도(y=x 에 붙으면 일치). 세로로 5개 쌓음.
        self.ax_phys_hdr = self.fig.add_axes([0.475, 0.905, 0.245, 0.03]); self.ax_phys_hdr.axis("off")
        self.ax_phys_list = [self.fig.add_axes([0.52, 0.72 - k * 0.172, 0.185, 0.12])
                             for k in range(5)]
        # 우측 plot 4개. 제목이 아래 눈금과 안 겹치게 간격(0.22)을 높이(0.15)보다 넉넉히.
        self.ax_p = [self.fig.add_axes([0.76, 0.75 - k * 0.22, 0.21, 0.15]) for k in range(4)]
        # controls 패널(ax_p[1])은 throttle/brake([0,1])와 steer(도, ±약160)의 스케일이
        # 크게 달라 한 축에 겹치면 throttle/brake 가 바닥에 깔린다. → steer 를 오른쪽
        # twin 축에 따로 그린다. twin 은 매 프레임 새로 만들면 축이 쌓이므로 여기서 한 번만.
        self.ax_ctrl_tw = self.ax_p[1].twinx()
        self._draw_phys()   # 물리검증 요약(고정, 1회)
        self._maximize()
        self._draw()

    def _draw_phys(self):
        """가운데 패널: av-check 물리 일관성. 체크마다 measured vs reference 산점도(y=x 일치).
        상대차 5대는 값이 거의 같아 한 산점도에 겹쳐 그린다. 범례(y=x)는 맨 위에만."""
        try:
            from .checks import run_all_checks
            results = run_all_checks(self.run)
        except Exception as e:
            self.ax_phys_hdr.text(0.5, 0.5, "물리검증 실패: %s" % e, ha="center", fontsize=8)
            return
        ego = [c for c in results if not c.key.startswith("opp")]
        opp = [c for c in results if c.key.startswith("opp")]
        npass = sum(1 for c in results if c.passed)
        nfail = sum(1 for c in results if c.passed is False)
        self.ax_phys_hdr.text(0.5, 0.5,
                              "물리 일관성 (av-check)   %d PASS / %d FAIL" % (npass, nfail),
                              ha="center", va="center", fontsize=12, weight="bold",
                              color=("#1a7f37" if nfail == 0 else "#cf222e"),
                              transform=self.ax_phys_hdr.transAxes)

        col = {True: "#1a7f37", False: "#cf222e", None: "#888"}
        # (제목, x축이름, reference, measured, passed) 5개 — 상대차는 5대 합침.
        panels = [(c.title.replace("자차 ", ""), c.unit, c.reference, c.measured, c.passed)
                  for c in ego]
        if opp:
            allpass = all(c.passed for c in opp)
            ref = np.concatenate([c.reference for c in opp]) if opp else np.array([])
            mea = np.concatenate([c.measured for c in opp]) if opp else np.array([])
            panels.append(("상대차 속도 (5대)", opp[0].unit, ref, mea, allpass))

        for k, ax in enumerate(self.ax_phys_list):
            if k >= len(panels):
                ax.axis("off"); continue
            title, unit, ref, mea, passed = panels[k]
            if len(ref):
                ax.scatter(ref, mea, s=2.5, alpha=0.18, color="#4c78a8", edgecolors="none")
                lo = float(np.nanmin([np.nanmin(ref), np.nanmin(mea)]))
                hi = float(np.nanmax([np.nanmax(ref), np.nanmax(mea)]))
                ax.plot([lo, hi], [lo, hi], color="#cf222e", lw=1.1, ls="--",
                        label=("y = x (일치)" if k == 0 else None))
            if k == 0:
                ax.legend(loc="upper left", fontsize=7.5, frameon=False)
            ax.set_title("● %s  [%s]" % (title, unit), fontsize=9, loc="left",
                         color=col[passed])
            ax.tick_params(labelsize=6.5)

    def _interp_row(self, df, tt: float) -> dict:
        """df(프레임 정렬) 를 시각 tt 로 선형보간한 한 행(dict). 각도(yaw/pitch/roll)는 wrap 보간.
        present/retired 같은 비수치 플래그는 앞 프레임 값을 그대로 쓴다."""
        t = self._t
        j = int(np.clip(np.searchsorted(t, tt) - 1, 0, len(t) - 2))
        a = float(np.clip((tt - t[j]) / (t[j + 1] - t[j]), 0.0, 1.0))
        r0, r1 = df.iloc[j], df.iloc[j + 1]
        out = {}
        for c in df.columns:
            v0, v1 = r0[c], r1[c]
            if isinstance(v0, (int, float, np.floating)) and np.isfinite(v0) and np.isfinite(v1):
                if "yaw" in c or c in ("pitch", "roll"):
                    d = np.arctan2(np.sin(v1 - v0), np.cos(v1 - v0))
                    out[c] = float(v0 + a * d)
                else:
                    out[c] = float(v0 + a * (v1 - v0))
            else:
                out[c] = v0
        return out

    def _track_aspect(self) -> float | None:
        """트랙 bbox 종횡비(가로/세로, pad 포함). racing_line 없으면 None."""
        rl = self.run.racing_line
        if rl is None:
            return None
        pad = 30.0
        dx = float(rl["x"].max() - rl["x"].min()) + 2 * pad
        dy = float(rl["y"].max() - rl["y"].min()) + 2 * pad
        return dx / dy if dy > 0 else None

    def _fit_rect(self, box, aspect):
        """box(=[x,y,w,h] fig fraction) 안에, 주어진 데이터 종횡비를 갖는 최대 축 rect를
        가운데 정렬로 맞춘다. aspect None 이면 box 그대로(정사각 패널)."""
        x0, y0, w, h = box
        if not aspect:
            return list(box)
        fw, fh = self.fig.get_size_inches()
        W, H = w * fw, h * fh                     # box 를 인치로
        if aspect >= W / H:                       # 가로가 한계 → 폭 꽉, 높이 줄임
            w_in, h_in = W, W / aspect
        else:                                     # 세로가 한계 → 높이 꽉, 폭 줄임
            h_in, w_in = H, H * aspect
        nw, nh = w_in / fw, h_in / fh
        return [x0 + (w - nw) / 2, y0 + (h - nh) / 2, nw, nh]   # box 안 가운데

    def _maximize(self):
        """백엔드별로 창을 최대화(전체화면). 실패해도 무시(figsize 로 이미 크게)."""
        try:
            mng = self.fig.canvas.manager
            for fn in (lambda: mng.window.state("zoomed"),        # TkAgg (Windows)
                       lambda: mng.window.showMaximized(),        # Qt
                       lambda: mng.full_screen_toggle()):         # 그 외
                try:
                    fn(); return
                except Exception:
                    continue
        except Exception:
            pass

    # ------------------------------------------------------------------ 그리기
    def _recorded_camera(self, i: int):
        """프레임별 CSP 실측 pose와 intrinsics. 불완전 행이면 (None, 기존 intrinsics)."""
        if not self.has_render_camera or self.run.camera is None or i >= len(self.run.camera):
            return None, self.f, self.cx, self.cy
        r = self.run.camera.iloc[i]
        names = ["cam_pos_x", "cam_pos_y", "cam_pos_z", "cam_forward_x",
                 "cam_forward_y", "cam_forward_z", "cam_up_x", "cam_up_y",
                 "cam_up_z", "cam_fov_deg"]
        if not all(pd.notna(r.get(c)) and np.isfinite(float(r[c])) for c in names):
            return None, self.f, self.cx, self.cy
        cam = RenderCameraPose(
            np.array([r.cam_pos_x, r.cam_pos_y, r.cam_pos_z], float),
            np.array([r.cam_forward_x, r.cam_forward_y, r.cam_forward_z], float),
            np.array([r.cam_up_x, r.cam_up_y, r.cam_up_z], float),
        )
        # CSP/AC camera FOV는 수직 FOV. 핀홀 함수에는 1920x1080 기준 수평 FOV를 넘긴다.
        vfov = float(r.cam_fov_deg)
        hfov = float(np.degrees(2 * np.arctan(np.tan(np.radians(vfov) / 2)
                                              * 1920 / 1080)))
        f, cx, cy = camera_intrinsics(1920, 945, hfov)
        return cam, f, cx, cy

    def _recorded_car(self, car_id: int, i: int) -> RenderCarPose | None:
        df = self.run.render_cars.get(car_id)
        if df is None or i >= len(df):
            return None
        r = df.iloc[i]
        names = ["car_pos_x", "car_pos_y", "car_pos_z", "car_forward_x",
                 "car_forward_y", "car_forward_z", "car_up_x", "car_up_y",
                 "car_up_z", "aabb_center_forward", "aabb_center_right",
                 "aabb_center_up", "aabb_length", "aabb_width", "aabb_height"]
        if not all(pd.notna(r.get(c)) and np.isfinite(float(r[c])) for c in names):
            return None
        return RenderCarPose(
            np.array([r.car_pos_x, r.car_pos_y, r.car_pos_z], float),
            np.array([r.car_forward_x, r.car_forward_y, r.car_forward_z], float),
            np.array([r.car_up_x, r.car_up_y, r.car_up_z], float),
            np.array([r.aabb_center_forward, r.aabb_center_right,
                      r.aabb_center_up], float),
            np.array([r.aabb_length, r.aabb_width, r.aabb_height], float),
        )

    def _draw(self):
        i = self.i
        run = self.run
        ego = run.ego.iloc[i]

        # --- 패널 1: 카메라 이미지 + 3D bbox ---
        # 이미지 내용은 t_img 보다 τ(캡처 지연)만큼 과거 장면 → ego/opp 상태를 t_img-τ 로
        # 보간해서 투영해야 코너·근접에서 박스가 진행방향으로 안 밀린다.
        # 온보드 카메라는 차체에 붙어 트랙 경사·차체 기울기를 따라간다 → pitch/roll 도 반영
        # (경사 1° 무시하면 박스가 ~18px 뜬다).
        if self._t is not None and self.tau_s:
            tt = float(self._t[i]) - self.tau_s
            e = self._interp_row(run.ego, tt)
        else:
            tt = None
            e = ego
        ego_xyz = np.array([e["world_position_x"], e["world_position_y"], e["world_position_z"]])
        eyaw = float(e["yaw"])
        epitch = float(e["pitch"]) if "pitch" in run.ego.columns and np.isfinite(e["pitch"]) else 0.0
        # roll 은 horizon-lock 캡처면 카메라가 안 따라가므로 0 (apply_roll=False).
        eroll = (float(e["roll"]) if self.apply_roll and "roll" in run.ego.columns
                 and np.isfinite(e["roll"]) else 0.0)
        self.ax_img.clear()
        render_camera, frame_f, frame_cx, frame_cy = self._recorded_camera(i)
        img_path = bevmod._camera_image_path(run, i)
        exp_path = bevmod._export_image_path(run, i)
        if img_path is not None:
            self.ax_img.imshow(mpimg.imread(str(img_path)))
        n_box = 0
        image_labels = []
        for oid, o in run.opps.items():
            r = o.iloc[i]
            if not _bbox_row_visible(r):
                continue
            if tt is not None:
                r = pd.Series(self._interp_row(o, tt))
            render_car = self._recorded_car(oid, i)
            edges, front, behind, in_view = opponent_box_edges(
                r, ego_xyz, eyaw, run.car_dim(oid), frame_f, frame_cx, frame_cy,
                cam_pose=self.cam_pose, bbox_center=run.car_bbox_center(oid),
                ego_pitch=epitch, ego_roll=eroll, render_camera=render_camera,
                render_car=render_car)
            if behind or not in_view:
                continue
            for a, b, kind in edges:
                if kind == "front":
                    color, style, alpha, width = "#00d7ff", "--", 0.95, 1.4
                elif kind == "rear":
                    color, style, alpha, width = _OPP_C, "-", 1.0, 1.8
                else:
                    color, style, alpha, width = _OPP_C, "-", 0.55, 1.2
                self.ax_img.plot([a[0], b[0]], [a[1], b[1]], color=color,
                                 ls=style, alpha=alpha, lw=width, zorder=5)
            if front is not None:
                self.ax_img.add_patch(Polygon(front, closed=True, fc="#00d7ff",
                                              alpha=0.08, ec="none", zorder=4))
            c = np.nanmean([p for e in edges for p in e[:2]], axis=0)
            image_labels.append((oid, float(c[0]), float(c[1])))
            n_box += 1
        for oid, anchor, pos in _layout_image_labels(image_labels):
            if np.hypot(pos[0] - anchor[0], pos[1] - anchor[1]) > 2:
                self.ax_img.plot([anchor[0], pos[0]], [anchor[1], pos[1]],
                                 color="yellow", lw=0.7, alpha=0.75, zorder=6)
            self.ax_img.text(pos[0], pos[1], str(oid), color="yellow", fontsize=11,
                             ha="center", va="center", weight="bold", zorder=7,
                             bbox=dict(facecolor="black", edgecolor="none",
                                       alpha=0.42, pad=0.6))
        self.ax_img.set_xlim(0, 1920); self.ax_img.set_ylim(945, 0)
        self.ax_img.axis("off")
        cap = "raw: %s" % (img_path.name if img_path else "?")
        if exp_path is not None:
            cap += "   |   export: %s" % exp_path.name
        cam_label = "CSP render pose" if render_camera is not None else self.cam_name
        self.ax_img.set_title("image + 3D bbox (%d cars, %s; front=cyan dashed, rear=green)   %s"
                              % (n_box, cam_label, cap),
                              fontsize=9)

        # --- 패널 2: BEV (ego-centric) ---
        self.ax_bev.clear()
        self.ax_scene_legend.clear()
        self.ax_scene_legend.axis("off")
        self._draw_bev(self.ax_bev, i)

        # --- 패널 3: 전체 트랙 맵 ---
        self.ax_glob.clear()
        self._draw_global(self.ax_glob, i)

        # --- 패널 4: ego 검수 plot ---
        self._draw_plots(i)

        self.fig.suptitle("%s  |  frame %d / %d   (arrows or a/d: move,  Shift: x50,  s: save,  q: quit)"
                          % (run.track_name, i, run.n_frames - 1), fontsize=12,
                          y=0.988, va="top")
        self.fig.canvas.draw_idle()

    def _draw_bev(self, ax, i):
        """BEV(ego-centric). 범례는 global view와 하단 공통 범례를 사용한다."""
        bevmod.draw_scene(ax, self.run, i, view="ego", fov_deg=self.fov,
                          legend=False, title=False)
        ax.set_title("BEV (ego-centric)", fontsize=9)

    def _draw_global(self, ax, i):
        """전체 트랙 맵. av-bev 의 global 뷰와 **동일한 렌더링**을 공용 draw_scene 으로 재사용."""
        n_shown = bevmod.draw_scene(ax, self.run, i, view="global", title=False,
                                    legend=True, legend_ax=self.ax_scene_legend,
                                    legend_ncol=4)
        ego = self.run.ego.iloc[i]
        spd = float(ego.get("speed", float("nan")))
        ax.set_title("%s · frame %d\n%.1f m/s (%.0f km/h) · %d opp"
                     % (self.run.track_name, i, spd, spd * 3.6, n_shown),
                     fontsize=8.5, pad=3)
        ax.tick_params(labelsize=7)

    def _draw_plots(self, i):
        """EGO 검수 plot: 속도 / 조작값 / yaw_rate / 가속도(모두 자차).
        x축 = 시간(초), 빨간 세로선 = 현재 프레임."""
        run = self.run
        e = run.ego
        t = e["t_img"].to_numpy() if "t_img" in e else np.arange(len(e))
        # controls(k=1)는 아래에서 twin 축으로 따로 처리하므로 여기선 speed/yaw_rate/accel 만.
        specs = {
            0: ("EGO speed (m/s)", [("speed", "#1f77b4")]),
            2: ("EGO yaw_rate (rad/s)", [("yaw_rate", "#9467bd")]),
            3: ("EGO accel (G)", [("accelY", "#ff7f0e"), ("accelX", "#8c564b")]),
        }
        for k, (title, cols) in specs.items():
            ax = self.ax_p[k]
            ax.clear()
            for col, c in cols:
                if col in e.columns:
                    ax.plot(t, e[col].to_numpy(), color=c, lw=0.7, label=col)
            ax.axvline(t[i], color="red", lw=1.2)
            ax.set_title(title, fontsize=8)
            ax.tick_params(labelsize=6)
            if k == 3:                             # 맨 아래에만 x축 라벨
                ax.set_xlabel("time (s)", fontsize=7)
            if len(cols) > 1:
                ax.legend(fontsize=5, loc="upper right")

        # --- controls 패널: throttle/brake 는 왼쪽축[0,1], steer(도)는 오른쪽 twin 축 ---
        axc, axs = self.ax_p[1], self.ax_ctrl_tw
        axc.clear(); axs.clear()
        h = []
        for col, c in (("throttle", "#2ca02c"), ("brake", "#d62728")):
            if col in e.columns:
                h += axc.plot(t, e[col].to_numpy(), color=c, lw=0.7, label=col)
        axc.set_ylim(-0.05, 1.05)
        axc.set_ylabel("thr/brk", fontsize=6, color="#555")
        if "steer" in e.columns:
            sv = e["steer"].to_numpy()
            h += axs.plot(t, sv, color="#1f77b4", lw=0.7, label="steer(deg)")
            m = float(np.nanmax(np.abs(sv))) or 1.0
            axs.set_ylim(-1.1 * m, 1.1 * m)
            axs.set_ylabel("steer(deg)", fontsize=6, color="#1f77b4")
            # clear() 가 라벨 위치를 기본(왼쪽)으로 되돌리므로 매번 오른쪽으로 고정(눈금은 오른쪽).
            axs.yaxis.set_label_position("right")
        axs.tick_params(labelsize=6, colors="#1f77b4")
        axc.axvline(t[i], color="red", lw=1.2)
        axc.set_title("EGO controls (thr/brk=L, steer=R)", fontsize=8)
        axc.tick_params(labelsize=6)
        axc.legend(handles=h, fontsize=5, loc="upper right")
        # "current frame" 은 범례 대신 첫 plot 바로 위에 빨간 글씨로.
        self.ax_p[0].text(0.0, 1.28, "red line = current frame", transform=self.ax_p[0].transAxes,
                          color="red", fontsize=7, ha="left", va="bottom")

    # ------------------------------------------------------------------ 조작
    def _on_key(self, event):
        n = self.run.n_frames
        big = self.step * 50
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
    ap.add_argument("--fov", type=float, default=None,
                    help="수평 화각(도). 기본: meta 의 수집시 FOV 기록으로 자동")
    ap.add_argument("--camera", type=int, default=None,
                    help="3D bbox 투영 온보드 카메라 인덱스(cameras.ini, DASH 는 마지막 인덱스). "
                         "기본: meta 의 수집시 카메라 모드로 자동(기록 없으면 1=후드/보닛)")
    ap.add_argument("--tau-ms", type=float, default=0.0,
                    help="캡처 지연 보정(ms). 이미지 내용이 t_img 보다 과거라 telemetry 를 "
                         "t_img-τ 로 보간해 투영. 코너서 박스가 진행방향 반대로 밀리면 ↑, "
                         "앞서가면 ↓ (기본 30)")
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
    viewer = Viewer(run, frame=args.frame, step=args.step, fov_deg=args.fov,
                    camera_index=args.camera, tau_ms=args.tau_ms)
    plt.show()
    return viewer


if __name__ == "__main__":
    main()
