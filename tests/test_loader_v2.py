import tempfile
import unittest
from pathlib import Path

import pandas as pd

from ac_validation.loader_v2 import load_v2_session


EGO = """position: {x} 2.0 3.0
orientation: 0.0 0.0 0.0 1.0
velocity: 10.0 0.0 0.0
angular_velocity: 0.0 0.0 5.0
acceleration: 1.0 2.0 0.0
throttle: 0.8
brake: 0.0
steer_angle: 0.25
"""

OBJECT_HEADER = (
    "position.x,position.y,position.z,orientation.x,orientation.y,orientation.z,"
    "orientation.w,dimension.length,dimension.width,dimension.height,velocity.x,"
    "velocity.y,velocity.z,track_id,visible,relative.forward,relative.lateral,"
    "distance,closing_speed\n"
)
OBJECT_ROW = "5,6,3,0,0,0,1,4.7,1.7,1.0,9,0,0,1,1,3,4,5,1\n"


class V2LoaderTests(unittest.TestCase):
    def make_session(self, root: Path) -> Path:
        session = root / "20260729_000000_lagunaseca_test_multi_40hz"
        leaf = session / "NON_INTERACTION" / "0001"
        for name in ("CAMERA", "EGO_INFO", "OBJECT_INFO"):
            (leaf / name).mkdir(parents=True, exist_ok=True)
        (session / "TRACK_INFO").mkdir(parents=True)
        (session / "SESSION_INFO.txt").write_text(
            "managed_by=ac_dataset.structured_export\n"
            "schema_version=3\nrun_id=20260729_000000\nmode=multi\n"
            "track=ks_laguna_seca\ngrid_hz=40\n",
            encoding="utf-8",
        )
        index = pd.DataFrame({
            "time_ms": [0, 25], "source_time_ms": [100, 125],
            "source_index": [4, 5], "source_frame": [4, 5],
            "t_img": [0.1, 0.125], "lap_count": [0, 0],
            "spline_position": [0.1, 0.2], "sync_ok": [True, True],
        })
        index.to_csv(session / "FRAME_INDEX.csv", index=False)
        for number, time_ms in enumerate((0, 25)):
            (leaf / "CAMERA" / f"{time_ms}.jpg").write_bytes(b"jpeg")
            (leaf / "EGO_INFO" / f"{time_ms}.txt").write_text(
                EGO.format(x=1.0 + number), encoding="utf-8")
            (leaf / "OBJECT_INFO" / f"{time_ms}.txt").write_text(
                OBJECT_HEADER + OBJECT_ROW, encoding="utf-8")

        camera = index.copy()
        camera.insert(0, "frame", [0, 1])
        for name, values in {
            "cam_pos_x": [0.0, 0.0], "cam_pos_y": [0.0, 0.0],
            "cam_pos_z": [1.0, 1.0], "cam_forward_x": [1.0, 1.0],
            "cam_forward_y": [0.0, 0.0], "cam_forward_z": [0.0, 0.0],
            "cam_up_x": [0.0, 0.0], "cam_up_y": [0.0, 0.0],
            "cam_up_z": [1.0, 1.0], "cam_fov_deg": [60.0, 60.0],
        }.items():
            camera[name] = values
        camera.to_parquet(session / "image_index.parquet", index=False)

        render_rows = []
        for frame in (0, 1):
            for car_id in (0, 1):
                render_rows.append({
                    "image_frame": frame, "car_id": car_id,
                    "car_pos_x": float(frame + car_id), "car_pos_y": 0.0,
                    "car_pos_z": 0.0, "car_forward_x": 1.0,
                    "car_forward_y": 0.0, "car_forward_z": 0.0,
                    "car_up_x": 0.0, "car_up_y": 0.0, "car_up_z": 1.0,
                    "aabb_center_forward": 0.0, "aabb_center_right": 0.0,
                    "aabb_center_up": 0.5, "aabb_length": 4.7,
                    "aabb_width": 1.7, "aabb_height": 1.0,
                })
        pd.DataFrame(render_rows).to_parquet(
            session / "render_car_state.parquet", index=False)
        pd.DataFrame({
            "x": [0.0, 10.0], "y": [0.0, 0.0],
            "side_left": [5.0, 5.0], "side_right": [5.0, 5.0],
        }).to_parquet(session / "TRACK_INFO" / "AI_RACING_LINE.parquet", index=False)
        return session

    def test_loads_v2_partition_as_run_data(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = self.make_session(Path(tmp))
            run = load_v2_session(session)
            self.assertEqual(run.n_frames, 2)
            self.assertEqual(run.track_name, "ks_laguna_seca")
            self.assertEqual(run.meta["steer_unit"], "normalized")
            self.assertAlmostEqual(run.ego.iloc[0]["speed"], 10.0)
            self.assertAlmostEqual(run.ego.iloc[0]["yaw_rate"], 5.0 * 3.141592653589793 / 180)
            self.assertEqual(sorted(run.opps), [1])
            self.assertTrue(run.opps[1].iloc[0]["visible"])
            self.assertTrue((session / run.camera.iloc[0]["image_path"]).exists())
            self.assertEqual(sorted(run.render_cars), [0, 1])

    def test_zip_requires_extraction(self):
        with self.assertRaisesRegex(ValueError, "압축을 푼"):
            load_v2_session("session.zip")


if __name__ == "__main__":
    unittest.main()
