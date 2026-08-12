"""Interactive viewer for extracted, render-metadata-enriched dataset_v2 sessions."""

from __future__ import annotations

import argparse

import matplotlib
import matplotlib.pyplot as plt

from .loader_v2 import load_v2_session
from .viewer import Viewer


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="av-view-v2",
        description="dataset_v2 multi 세션 인터랙티브 검증 뷰어",
    )
    parser.add_argument("session_dir", help="압축을 푼 dataset_v2/multi/<session>")
    parser.add_argument("--frame", type=int, default=0, help="시작 v2 frame index")
    parser.add_argument("--step", type=int, default=1, help="←/→ 이동 간격")
    parser.add_argument("--fov", type=float, default=None, help="FOV 수동 override")
    args = parser.parse_args()

    if matplotlib.get_backend().lower() == "agg":
        for backend in ("QtAgg", "TkAgg"):
            try:
                matplotlib.use(backend, force=True)
                break
            except Exception:
                continue

    run = load_v2_session(args.session_dir)
    viewer = Viewer(run, frame=args.frame, step=args.step, fov_deg=args.fov,
                    camera_index=None, tau_ms=0.0)
    plt.show()
    _ = viewer


if __name__ == "__main__":
    main()
