"""loader - run 하나를 RunData 로 로드
=====================================

`ac_dataset` 가 만든 `dataset/<run_id>/` 를 읽어, 검증 모듈들이 공통으로 쓰는 형태로 모은다.
읽기 전용. 좌표 규약은 ac_dataset 와 동일:
  - world_position 수평면 = (x, y), z = 높이. 트랙별 로컬 직교좌표(미터).
  - ego yaw / opp yaw_body = 진행 heading (rad). (cos, sin) 이 forward 방향.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


@dataclass
class RunData:
    run_dir: Path
    run_id: str
    track_name: str
    ego: pd.DataFrame                  # processed/ego_features (frame,t_img,world_position_*,yaw,speed,valid,...)
    opps: dict[int, pd.DataFrame]      # opp_id -> processed/opponent/opp_<id>_features
    racing_line: pd.DataFrame | None   # track/racing_line (x,y,side_left,side_right,...)
    camera: pd.DataFrame | None        # camera/camera_index (frame,t_img,image_path)
    render_cars: dict[int, pd.DataFrame]  # CSP same-render full pose + visual AABB
    meta: dict
    ego_dim: tuple[float, float, float]              # (length, width, height) m
    opp_dim: dict[int, tuple[float, float, float]]   # opp_id -> dims
    ego_bbox_center: tuple[float, float, float]      # (forward, left, up) m
    opp_bbox_center: dict[int, tuple[float, float, float]]
    onboard_cams: list[dict]                          # ego 차종 cameras.ini (없으면 [])

    @property
    def n_frames(self) -> int:
        return len(self.ego)

    def car_dim(self, opp_id: int | None) -> tuple[float, float, float]:
        """opp_id=None 이면 ego 치수, 아니면 그 상대차 치수."""
        return self.ego_dim if opp_id is None else self.opp_dim.get(opp_id, self.ego_dim)

    def car_bbox_center(self, opp_id: int | None) -> tuple[float, float, float]:
        """차량 원점 기준 collider bbox 중심 (forward, left, up)."""
        return (self.ego_bbox_center if opp_id is None
                else self.opp_bbox_center.get(opp_id, self.ego_bbox_center))


# 치수를 못 읽을 때(예: data.acd 접근 불가) 쓰는 기본값. ac_dataset 기본값과 맞춤.
_DEFAULT_DIM = (4.5, 1.8, 1.0)
_DEFAULT_BBOX_CENTER = (0.0, 0.0, _DEFAULT_DIM[2] / 2)


def _read_car_models(run_dir: Path) -> dict[int, str]:
    """export_info.json 에 기록된 car_id→모델. 없으면 빈 dict."""
    for p in sorted(run_dir.parent.glob(run_dir.name + "_*")):
        f = p / "export_info.json"
        if f.exists():
            info = json.loads(f.read_text(encoding="utf-8"))
            return {int(k): v for k, v in info.get("car_models", {}).items()}
    return {}


def _collision_boxes_for(models: dict[int, str]) -> dict[int, tuple[tuple[float, float, float],
                                                                       tuple[float, float, float]]]:
    """car_id→치수. ac_dataset.acd 로 data.acd 에서 직접 읽는다(중복 구현 안 함)."""
    try:
        from ac_dataset.acd import read_car_specs_by_model
    except Exception:
        return {}
    cache: dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]] = {}
    out: dict[int, tuple[tuple[float, float, float], tuple[float, float, float]]] = {}
    for cid, model in models.items():
        if model not in cache:
            specs = read_car_specs_by_model(model)
            dims = tuple(specs["dimension"]) if (specs and specs.get("dimension")) else _DEFAULT_DIM
            center = (tuple(specs["bbox_center"]) if (specs and specs.get("bbox_center"))
                      else _DEFAULT_BBOX_CENTER)
            cache[model] = dims, center
        out[cid] = cache[model]
    return out


def _onboard_cams_for(ego_model: str | None) -> list[dict]:
    """ego 차종의 온보드 카메라(cameras.ini) 목록. ac_dataset.acd 로 data.acd 에서 직접 읽는다."""
    if not ego_model:
        return []
    try:
        from ac_dataset.acd import read_onboard_cameras_by_model
    except Exception:
        return []
    return read_onboard_cameras_by_model(ego_model) or []


def load_run(run_dir: str | Path) -> RunData:
    """dataset/<run_id> 를 RunData 로 로드. processed/ 가 있어야 한다."""
    run_dir = Path(run_dir)
    proc = run_dir / "processed"
    if not (proc / "ego_features.parquet").exists():
        raise FileNotFoundError(
            f"{run_dir} 에 processed/ego_features.parquet 이 없습니다. "
            "ac-process 를 먼저 돌리거나 올바른 run 폴더인지 확인하세요.")

    ego = pd.read_parquet(proc / "ego_features.parquet")
    opps: dict[int, pd.DataFrame] = {}
    for pf in sorted((proc / "opponent").glob("opp_*_features.parquet")):
        oid = int(pf.stem.split("_")[1])
        opps[oid] = pd.read_parquet(pf)

    racing_line = None
    rl_path = run_dir / "track" / "racing_line.parquet"
    if rl_path.exists():
        racing_line = pd.read_parquet(rl_path)

    camera = None
    cam_path = run_dir / "camera" / "camera_index.parquet"
    if cam_path.exists():
        camera = pd.read_parquet(cam_path)

    render_cars: dict[int, pd.DataFrame] = {}
    render_path = run_dir / "camera" / "render_car_state.parquet"
    if render_path.exists():
        render_all = pd.read_parquet(render_path)
        for cid, group in render_all.groupby("car_id"):
            # frame lookup을 iloc로 안전하게 쓰도록 전체 frame 축에 재색인한다.
            render_cars[int(cid)] = group.sort_values("frame").set_index("frame").reindex(
                range(len(ego))).reset_index()

    track_name = "unknown"
    tj = run_dir / "track" / "track.json"
    if tj.exists():
        track_name = json.loads(tj.read_text(encoding="utf-8")).get("track_name", "unknown")

    meta = {}
    mj = run_dir / "meta.json"
    if mj.exists():
        meta = json.loads(mj.read_text(encoding="utf-8"))

    models = _read_car_models(run_dir)
    boxes = _collision_boxes_for(models)
    ego_dim, ego_bbox_center = boxes.get(0, (_DEFAULT_DIM, _DEFAULT_BBOX_CENTER))
    opp_dim = {oid: boxes.get(oid, (ego_dim, ego_bbox_center))[0] for oid in opps}
    opp_bbox_center = {oid: boxes.get(oid, (ego_dim, ego_bbox_center))[1] for oid in opps}
    onboard_cams = _onboard_cams_for(models.get(0))

    return RunData(
        run_dir=run_dir, run_id=run_dir.name, track_name=track_name,
        ego=ego, opps=opps, racing_line=racing_line, camera=camera,
        render_cars=render_cars, meta=meta,
        ego_dim=ego_dim, opp_dim=opp_dim, ego_bbox_center=ego_bbox_center,
        opp_bbox_center=opp_bbox_center, onboard_cams=onboard_cams,
    )
