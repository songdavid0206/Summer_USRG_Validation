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
    meta: dict
    ego_dim: tuple[float, float, float]              # (length, width, height) m
    opp_dim: dict[int, tuple[float, float, float]]   # opp_id -> dims

    @property
    def n_frames(self) -> int:
        return len(self.ego)

    def car_dim(self, opp_id: int | None) -> tuple[float, float, float]:
        """opp_id=None 이면 ego 치수, 아니면 그 상대차 치수."""
        return self.ego_dim if opp_id is None else self.opp_dim.get(opp_id, self.ego_dim)


# 치수를 못 읽을 때(예: data.acd 접근 불가) 쓰는 기본값. ac_dataset 기본값과 맞춤.
_DEFAULT_DIM = (4.5, 1.8, 1.0)


def _read_car_models(run_dir: Path) -> dict[int, str]:
    """export_info.json 에 기록된 car_id→모델. 없으면 빈 dict."""
    for p in sorted(run_dir.parent.glob(run_dir.name + "_*")):
        f = p / "export_info.json"
        if f.exists():
            info = json.loads(f.read_text(encoding="utf-8"))
            return {int(k): v for k, v in info.get("car_models", {}).items()}
    return {}


def _dims_for(models: dict[int, str]) -> dict[int, tuple[float, float, float]]:
    """car_id→치수. ac_dataset.acd 로 data.acd 에서 직접 읽는다(중복 구현 안 함)."""
    try:
        from ac_dataset.acd import read_car_specs_by_model
    except Exception:
        return {}
    cache: dict[str, tuple[float, float, float]] = {}
    out: dict[int, tuple[float, float, float]] = {}
    for cid, model in models.items():
        if model not in cache:
            specs = read_car_specs_by_model(model)
            cache[model] = (tuple(specs["dimension"]) if (specs and specs.get("dimension"))
                            else _DEFAULT_DIM)
        out[cid] = cache[model]
    return out


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

    track_name = "unknown"
    tj = run_dir / "track" / "track.json"
    if tj.exists():
        track_name = json.loads(tj.read_text(encoding="utf-8")).get("track_name", "unknown")

    meta = {}
    mj = run_dir / "meta.json"
    if mj.exists():
        meta = json.loads(mj.read_text(encoding="utf-8"))

    models = _read_car_models(run_dir)
    dims = _dims_for(models)
    ego_dim = dims.get(0, _DEFAULT_DIM)
    opp_dim = {oid: dims.get(oid, ego_dim) for oid in opps}

    return RunData(
        run_dir=run_dir, run_id=run_dir.name, track_name=track_name,
        ego=ego, opps=opps, racing_line=racing_line, camera=camera, meta=meta,
        ego_dim=ego_dim, opp_dim=opp_dim,
    )
