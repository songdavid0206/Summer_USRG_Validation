"""Read an extracted structured ``dataset_v2`` multi-session for validation.

The v2 export stores the continuous session as a disjoint partition of
interaction clips and non-interaction segments.  This loader de-duplicates the
hard-linked per-frame files by their normalized ``time_ms`` name and adapts
them to the same :class:`RunData` interface used by the raw-run viewer.
"""

from __future__ import annotations

import csv
import io
import math
import os
from collections import defaultdict
from pathlib import Path

import numpy as np
import pandas as pd

from .loader import RunData


G = 9.81
_FRAME_DIRS = {"CAMERA", "EGO_INFO", "OBJECT_INFO"}


def _session_root(path: str | Path) -> Path:
    root = Path(path)
    if root.suffix.lower() == ".zip":
        raise ValueError(
            "av-view-v2는 압축을 푼 dataset_v2 세션 폴더를 입력으로 받습니다. "
            "ZIP 파일을 먼저 '모두 압축 풀기' 하세요."
        )
    if (root / "SESSION_INFO.txt").exists():
        return root
    children = [p for p in root.iterdir() if p.is_dir()] if root.is_dir() else []
    nested = [p for p in children if (p / "SESSION_INFO.txt").exists()]
    if len(nested) == 1:
        return nested[0]
    raise FileNotFoundError(f"dataset_v2 세션 폴더가 아닙니다: {root}")


def _read_kv(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


def _frame_files(root: Path) -> dict[str, dict[int, Path]]:
    """Find one copy of each hard-linked v2 frame asset."""
    found: dict[str, dict[int, Path]] = {name: {} for name in _FRAME_DIRS}
    for current, dirs, files in os.walk(root):
        leaf = Path(current).name
        if leaf not in _FRAME_DIRS:
            continue
        target = found[leaf]
        for name in files:
            try:
                time_ms = int(Path(name).stem)
            except ValueError:
                continue
            target.setdefault(time_ms, Path(current) / name)
        dirs.clear()
    return found


def _parse_ego(path: Path) -> dict[str, list[float]]:
    values: dict[str, list[float]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in line:
            continue
        key, raw = line.split(":", 1)
        values[key.strip()] = [float(item) for item in raw.split()]
    required = {
        "position", "orientation", "velocity", "angular_velocity",
        "acceleration", "throttle", "brake", "steer_angle",
    }
    missing = required - values.keys()
    if missing:
        raise ValueError(f"{path}: missing EGO_INFO fields {sorted(missing)}")
    return values


def _quat_to_rpy(q: list[float]) -> tuple[float, float, float]:
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    sin_pitch = float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _ego_table(index: pd.DataFrame, paths: dict[int, Path]) -> pd.DataFrame:
    rows = []
    total = len(index)
    for number, item in enumerate(index.itertuples(index=False), 1):
        time_ms = int(item.time_ms)
        path = paths.get(time_ms)
        if path is None:
            raise FileNotFoundError(f"EGO_INFO/{time_ms}.txt is missing")
        values = _parse_ego(path)
        position = values["position"]
        velocity = values["velocity"]
        angular = values["angular_velocity"]
        acceleration = values["acceleration"]
        roll, pitch, yaw = _quat_to_rpy(values["orientation"])
        rows.append({
            "frame": number - 1,
            "t_img": time_ms / 1000.0,
            "speed": float(np.linalg.norm(velocity)),
            "local_velocity_x": velocity[0],
            "local_velocity_y": velocity[1],
            "local_velocity_z": velocity[2],
            "angular_velocity_x": math.radians(angular[0]),
            "angular_velocity_y": math.radians(angular[1]),
            "angular_velocity_z": math.radians(angular[2]),
            "yaw_rate": math.radians(angular[2]),
            "accelY": acceleration[0] / G,
            "accelX": acceleration[1] / G,
            "yaw": yaw,
            "pitch": pitch,
            "roll": roll,
            "world_position_x": position[0],
            "world_position_y": position[1],
            "world_position_z": position[2],
            "NormalizedSplinePosition": float(item.spline_position),
            "LapCount": int(item.lap_count),
            "steer": values["steer_angle"][0],
            "throttle": values["throttle"][0],
            "brake": values["brake"][0],
            "in_pit": False,
            "valid": True,
        })
        if number % 4000 == 0 or number == total:
            print(f"[v2] EGO_INFO {number}/{total}", flush=True)
    return pd.DataFrame(rows)


def _object_rows(path: Path) -> list[dict[str, str]]:
    text = path.read_text(encoding="utf-8")
    reader = csv.DictReader(io.StringIO(text), skipinitialspace=True)
    return [{str(key).strip(): str(value).strip() for key, value in row.items()}
            for row in reader]


def _opponent_tables(index: pd.DataFrame, paths: dict[int, Path]) -> dict[int, pd.DataFrame]:
    records: dict[int, list[dict]] = defaultdict(list)
    total = len(index)
    for number, item in enumerate(index.itertuples(index=False), 1):
        time_ms = int(item.time_ms)
        path = paths.get(time_ms)
        if path is None:
            raise FileNotFoundError(f"OBJECT_INFO/{time_ms}.txt is missing")
        for row in _object_rows(path):
            oid = int(row["track_id"])
            q = [float(row[f"orientation.{axis}"]) for axis in "xyzw"]
            _, _, yaw = _quat_to_rpy(q)
            velocity = np.array([float(row[f"velocity.{axis}"]) for axis in "xyz"])
            records[oid].append({
                "frame": number - 1,
                "t_img": time_ms / 1000.0,
                "opp_id": oid,
                "present": True,
                "retired": False,
                "valid": True,
                "opp_x": float(row["position.x"]),
                "opp_y": float(row["position.y"]),
                "opp_z": float(row["position.z"]),
                "opp_forward": float(row["relative.forward"]),
                "opp_lateral": float(row["relative.lateral"]),
                "distance": float(row["distance"]),
                "opp_vx": velocity[0],
                "opp_vy": velocity[1],
                "opp_vz": velocity[2],
                "opp_speed": float(np.linalg.norm(velocity)),
                "opp_speedKMH": float(np.linalg.norm(velocity) * 3.6),
                "closing_speed": float(row["closing_speed"]),
                "opp_yaw": yaw,
                "yaw_body": yaw,
                "visible": bool(int(row["visible"])),
            })
        if number % 4000 == 0 or number == total:
            print(f"[v2] OBJECT_INFO {number}/{total}", flush=True)

    tables: dict[int, pd.DataFrame] = {}
    base = pd.DataFrame({
        "frame": np.arange(total, dtype=np.int64),
        "t_img": index["time_ms"].to_numpy(dtype=float) / 1000.0,
    }).set_index("frame")
    for oid, rows in sorted(records.items()):
        observed = pd.DataFrame(rows).set_index("frame")
        table = base.join(observed.drop(columns="t_img"), how="left").reset_index()
        for column in ("present", "retired", "valid", "visible"):
            table[column] = table[column].fillna(False).astype(bool)
        tables[oid] = table
    return tables


def _box_geometry(render: pd.DataFrame, car_id: int,
                  fallback=(4.5, 1.8, 1.0)) -> tuple[tuple[float, float, float],
                                                        tuple[float, float, float]]:
    car = render[render["car_id"] == car_id]
    if car.empty:
        dims = tuple(float(value) for value in fallback)
        return dims, (0.0, 0.0, dims[2] / 2.0)
    dims = tuple(float(car[name].median())
                 for name in ("aabb_length", "aabb_width", "aabb_height"))
    forward = float(car["aabb_center_forward"].median())
    right = float(car["aabb_center_right"].median())
    up = float(car["aabb_center_up"].median())
    return dims, (forward, -right, up)  # RunData/BEV uses (forward, left, up).


def load_v2_session(session_dir: str | Path) -> RunData:
    """Load one extracted, render-metadata-enriched v2 multi session."""
    root = _session_root(session_dir)
    required = [
        root / "FRAME_INDEX.csv",
        root / "image_index.parquet",
        root / "render_car_state.parquet",
        root / "TRACK_INFO" / "AI_RACING_LINE.parquet",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError(
            "이 v2 세션에는 viewer용 render metadata가 없습니다: " + ", ".join(missing)
        )

    info = _read_kv(root / "SESSION_INFO.txt")
    if info.get("mode") != "multi":
        raise ValueError("현재 av-view-v2는 사진에 나온 multi 세션만 지원합니다")

    index = pd.read_csv(root / "FRAME_INDEX.csv").sort_values("time_ms").reset_index(drop=True)
    if index["time_ms"].duplicated().any():
        raise ValueError("FRAME_INDEX.csv contains duplicate time_ms")
    assets = _frame_files(root)
    print(
        f"[v2] frames={len(index)} images={len(assets['CAMERA'])} "
        f"ego={len(assets['EGO_INFO'])} objects={len(assets['OBJECT_INFO'])}",
        flush=True,
    )

    ego = _ego_table(index, assets["EGO_INFO"])
    opps = _opponent_tables(index, assets["OBJECT_INFO"])

    camera = pd.read_parquet(root / "image_index.parquet").sort_values("frame").reset_index(drop=True)
    if len(camera) != len(index) or not np.array_equal(
            camera["time_ms"].to_numpy(), index["time_ms"].to_numpy()):
        raise ValueError("image_index.parquet does not match FRAME_INDEX.csv")
    camera["image_path"] = [
        str(assets["CAMERA"][int(time_ms)].relative_to(root))
        if int(time_ms) in assets["CAMERA"] else ""
        for time_ms in index["time_ms"]
    ]
    if (camera["image_path"] == "").any():
        missing_images = index.loc[camera["image_path"] == "", "time_ms"].tolist()[:10]
        raise FileNotFoundError(f"v2 CAMERA images are missing, e.g. {missing_images}")

    render = pd.read_parquet(root / "render_car_state.parquet")
    render_cars: dict[int, pd.DataFrame] = {}
    for cid, group in render.groupby("car_id"):
        if group["image_frame"].duplicated().any():
            raise ValueError(f"duplicate render state for car {cid}")
        render_cars[int(cid)] = (
            group.sort_values("image_frame").set_index("image_frame")
            .reindex(range(len(index))).reset_index()
        )

    racing_line = pd.read_parquet(root / "TRACK_INFO" / "AI_RACING_LINE.parquet")
    ego_dim, ego_center = _box_geometry(render, 0)
    opp_dim = {}
    opp_center = {}
    for oid in opps:
        opp_dim[oid], opp_center[oid] = _box_geometry(render, oid, ego_dim)

    meta: dict[str, object] = dict(info)
    meta.update({
        "image_hz_actual": float(info.get("grid_hz", 40.0)),
        "camera_world_aligned": 1,
        "v2_session": True,
        "steer_unit": "normalized",
    })
    return RunData(
        run_dir=root,
        run_id=info.get("run_id", root.name),
        track_name=info.get("track", "unknown"),
        ego=ego,
        opps=opps,
        racing_line=racing_line,
        camera=camera,
        render_cars=render_cars,
        meta=meta,
        ego_dim=ego_dim,
        opp_dim=opp_dim,
        ego_bbox_center=ego_center,
        opp_bbox_center=opp_center,
        onboard_cams=[],
    )
