"""report - 물리 일관성 검증 리포트(PNG) + summary.json + 이상프레임 flag
==========================================================================

`checks.run_all_checks` 결과를 사람이 한 장으로 심사할 수 있는 **PNG 리포트**와
기계가 읽는 **summary.json** 으로 낸다(Streamlit 아님, 저장형 산출물).

이상프레임 flag: 각 체크의 residual 에서 median±K·MAD 밖을 이상치로 본다(로버스트).
프레임 번호는 원본 camera/{frame:06d}.jpg 와 1:1 이므로, 그 프레임을 바로 열어 눈으로 확인 가능.

산출물(기본): <run>/validation/
  - physics_report.png   : 체크별 (measured vs reference 산점도 + residual 히스토그램)
  - summary.json         : run 메타 + 체크별 stats/pass + 이상프레임 목록
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

import numpy as np

from .checks import CheckResult, run_all_checks
from .loader import RunData, load_run

ANOMALY_K = 6.0          # median ± K·MAD 밖이면 이상프레임. 6 → 정규분포면 ~4σ, 로버스트라 보수적
MAX_ANOMALY_LIST = 40    # summary.json 에 남길 체크당 최대 이상프레임 수


def flag_anomalies(c: CheckResult, k: float = ANOMALY_K) -> np.ndarray:
    """residual 이 median±k·MAD 밖 '이면서' |resid|>anomaly_floor 인 프레임 번호(심한 순).

    flag_frames=False 인 체크(yaw·가속)는 residual 이 주행에 따라 정상적으로 커지므로
    per-frame flag 를 하지 않는다(빈 배열). floor 는 미분노이즈성 잔변동을 걸러낸다.
    """
    if not c.flag_frames:
        return np.array([], dtype=int)
    r = c.residual
    if len(r) == 0 or not np.isfinite(r).any():
        return np.array([], dtype=int)
    med = np.median(r)
    mad = np.median(np.abs(r - med))
    if mad <= 0:
        return np.array([], dtype=int)
    dev = np.abs(r - med) / mad
    idx = np.where((dev > k) & (np.abs(r) > c.anomaly_floor))[0]
    idx = idx[np.argsort(-np.abs(r[idx]))]     # 심한 것부터
    return c.frame[idx]


def build_summary(run: RunData, results: list[CheckResult]) -> dict:
    checks = []
    for c in results:
        anom = flag_anomalies(c)
        checks.append({
            "key": c.key,
            "title": c.title,
            "unit": c.unit,
            "n": c.n,
            "passed": c.passed,
            "note": c.note,
            "stats": {k: (round(v, 5) if isinstance(v, float) else v)
                      for k, v in c.stats.items()},
            "n_anomaly": int(len(anom)),
            "anomaly_frames": anom[:MAX_ANOMALY_LIST].astype(int).tolist(),
        })
    passed = [c["passed"] for c in checks if c["passed"] is not None]
    return {
        "run_id": run.run_id,
        "track": run.track_name,
        "n_frames": run.n_frames,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "n_checks": len(checks),
        "n_pass": int(sum(bool(p) for p in passed)),
        "n_fail": int(sum(not p for p in passed)),
        "all_passed": all(passed) if passed else None,
        "checks": checks,
    }


def render_png(run: RunData, results: list[CheckResult], out_path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")                    # 저장 전용(창 안 띄움)
    import matplotlib.pyplot as plt
    # 한글 라벨용. Windows 는 Malgun Gothic 이 기본 설치. 없으면 첫 후보로 폴백.
    for _f in ("Malgun Gothic", "AppleGothic", "NanumGothic", "DejaVu Sans"):
        if any(_f == f.name for f in matplotlib.font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = _f
            break
    plt.rcParams["axes.unicode_minus"] = False   # 유니코드 − 대신 ASCII -

    n = len(results)
    fig, axes = plt.subplots(n, 2, figsize=(12, 2.6 * n), squeeze=False)
    status = {True: ("PASS", "#1a7f37"), False: ("FAIL", "#cf222e"), None: ("SKIP", "#888")}

    for i, c in enumerate(results):
        ax_s, ax_h = axes[i]
        label, color = status[c.passed]

        # 왼쪽: measured vs reference 산점도 + y=x
        if c.n:
            ax_s.scatter(c.reference, c.measured, s=3, alpha=0.25,
                         color="#4c78a8", edgecolors="none")
            lo = float(np.nanmin([c.reference.min(), c.measured.min()]))
            hi = float(np.nanmax([c.reference.max(), c.measured.max()]))
            ax_s.plot([lo, hi], [lo, hi], color="#cf222e", lw=1.2, ls="--", label="y = x")
            ax_s.legend(loc="upper left", fontsize=8, frameon=False)
        ax_s.set_title(f"[{label}] {c.title}", color=color, fontsize=11, loc="left")
        ax_s.set_xlabel(f"reference (game)  [{c.unit}]", fontsize=8)
        ax_s.set_ylabel(f"measured (indep)  [{c.unit}]", fontsize=8)
        ax_s.tick_params(labelsize=8)

        # 오른쪽: residual 히스토그램 + p95 / anomaly 경계
        if c.n:
            r = c.residual[np.isfinite(c.residual)]
            ax_h.hist(r, bins=60, color="#54a24b", alpha=0.8)
            med = float(np.median(r)); mad = float(np.median(np.abs(r - med)))
            p95 = float(np.percentile(np.abs(r), 95))
            ax_h.axvline(med, color="#333", lw=1, label=f"median {med:.2f}")
            ax_h.axvline(med + ANOMALY_K * mad, color="#cf222e", lw=0.9, ls=":",
                         label=f"±{ANOMALY_K:g}·MAD (anomaly)")
            ax_h.axvline(med - ANOMALY_K * mad, color="#cf222e", lw=0.9, ls=":")
            ax_h.legend(loc="upper right", fontsize=7, frameon=False)
            ax_h.set_title(f"residual  |p95|={p95:.2f}{c.unit}  n={c.n}", fontsize=9, loc="left")
        ax_h.set_xlabel(f"measured - reference  [{c.unit}]", fontsize=8)
        ax_h.tick_params(labelsize=8)

    n_pass = sum(1 for c in results if c.passed)
    n_fail = sum(1 for c in results if c.passed is False)
    fig.suptitle(f"물리 일관성 검증 — {run.run_id} ({run.track_name}, {run.n_frames} frames)   "
                 f"PASS {n_pass} / FAIL {n_fail}",
                 fontsize=13, y=0.997)
    fig.tight_layout(rect=(0, 0, 1, 0.995))
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def make_report(run_dir: str | Path, out_dir: str | Path | None = None) -> dict:
    """run 하나에 대해 물리검증 PNG + summary.json 생성. summary dict 반환."""
    run = load_run(run_dir)
    results = run_all_checks(run)
    out = Path(out_dir) if out_dir else (run.run_dir / "validation")
    out.mkdir(parents=True, exist_ok=True)

    summary = build_summary(run, results)
    (out / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    render_png(run, results, out / "physics_report.png")
    summary["_out_dir"] = str(out)
    return summary


def main(argv: list[str] | None = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(
        prog="av-check", description="AC 데이터셋 물리 일관성 검증 리포트 생성")
    ap.add_argument("run_dir", help="dataset/<run_id> (processed/ 있는 원본 run 폴더)")
    ap.add_argument("-o", "--out", default=None, help="출력 폴더(기본: <run>/validation)")
    args = ap.parse_args(argv)

    s = make_report(args.run_dir, args.out)
    print(f"run {s['run_id']} ({s['track']}, {s['n_frames']} frames)")
    for c in s["checks"]:
        mark = {True: "PASS", False: "FAIL", None: "skip"}[c["passed"]]
        print(f"  [{mark}] {c['title']}  n={c['n']}  이상프레임 {c['n_anomaly']}개  — {c['note']}")
    verdict = {True: "전부 통과", False: "실패 있음", None: "판정불가"}[s["all_passed"]]
    print(f"→ {s['n_pass']} PASS / {s['n_fail']} FAIL  ({verdict})")
    print(f"→ 저장: {s['_out_dir']}\\physics_report.png, summary.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
