# ac_validation — AC IL 데이터셋 검증/시각화 툴킷

`ac_dataset` 이 만든 run(`dataset/<run_id>/`)을 **읽기 전용**으로 심사한다.
수집 파이프라인과 분리돼 있어(의존성 한 방향: validation → dataset), 이 패키지가 깨져도
수집·학습은 영향이 없다. 좌표/투영/제원 로직은 `ac_dataset` 를 import 해서 재사용한다.

실제 자율주행 데이터셋 devkit(nuScenes/Argoverse/Waymo)이 하는 검증 방식을 AC 데이터에 맞춰
재구현하는 것을 목표로 한다: 카메라 재투영(3D bbox), BEV, 물리 일관성, 시간 동기화, 미래 궤적.

## 설치

두 패키지를 `-e`(editable)로 설치한다. `ac_validation` 이 `ac_dataset` 을 import 하므로 순서 중요.

```bash
cd ac_dataset      && pip install -e .     # 먼저 수집 패키지
cd ../ac_validation && pip install -e .
```

> 이미 설치돼 있어도, 새 CLI 명령(예: `av-check`)이 추가되면 `cd ac_validation && pip install -e .`
> 를 **한 번 다시** 돌려야 그 명령이 터미널에서 인식된다(진입점 재등록). 코드만 고친 경우는 불필요(editable).

## 실행 위치와 경로 (중요)

CLI 는 어느 폴더에서든 실행되지만, run 폴더 **경로는 현재 위치 기준**이다. 헷갈리면 아래 표대로.

| 현재 터미널 위치 | 명령 예시 |
|---|---|
| `정근/` (상위 폴더) | `av-view ac_dataset/dataset/<run_id>` |
| `정근/ac_dataset/` | `av-view dataset/<run_id>` |
| `정근/ac_validation/` | `av-view ../ac_dataset/dataset/<run_id>` |

- 넣는 것은 **원본 run 폴더**(`dataset/<run_id>`)다. 뒤에 `_imola_40hz` 가 붙은 **export 폴더가 아니다.**
  (도구가 원본 폴더 안의 `processed/`·`camera/`·`track/` 을 알아서 찾는다.)
- `<run_id>` = 수집 시각(예: `20260713_161926`). `collect` 종료 시 `[export]` 줄에 나오는 이름의 앞부분.

## 도구

| 명령 | 하는 일 |
|---|---|
| **`av-view <run>`** | **인터랙티브 검증 뷰어** — 프레임을 넘겨가며 이미지+3D bbox / BEV / ego plot 을 본다 |
| `av-bev <run>` | 정지 BEV 한 프레임(ego/global) + 카메라 이미지 나란히 (파일 저장) |
| **`av-check <run>`** | **물리 일관성 자동 검증** — 채널 교차검증으로 PNG 리포트 + `summary.json` 생성 |

### `av-view` — 인터랙티브 뷰어 (메인)

**전체화면** 창에 **4패널**을 띄우고 프레임을 넘겨가며 검증한다. **미리 저장하지 않고** 넘길
때마다 그 프레임만 즉석에서 그린다(저장 용량 0). 모든 프레임/모든 run 에 그대로 쓸 수 있다.

```bash
av-view ac_dataset/dataset/<run_id> --frame 350
#   --frame N   시작 프레임(processed 행 index). 기본 0
#   --step N    ←/→ 이동 간격. 기본 1
#   --fov 86.78 수평 화각(도)
```

- **카메라 이미지 + 상대차 3D bbox** (좌측 위) — 상대차의 3D 박스를 이미지에 투영.
  박스가 실제 차에 얹히면 위치·방향·치수·투영이 맞는 것. 화각 밖/너무 가까운 차는 자동 제외.
  제목에 원본 이름(`000350.jpg`)과 export 최종 이름(`12325.jpg`)을 함께 표시.
- **전체 트랙 맵 (world)** (좌측 아래) — 트랙 전체 좌표에서 ego(파란 점+화살표=진행방향)·
  상대차(초록 점, 리타이어=빨강)·지나온 경로. 트랙 이탈·전체 위치·상대차 배치를 한눈에.
- **BEV (ego-centric)** (가운데) — ego(파랑, "ego")·상대차(초록, 번호)·racing line(주황)·
  track edge(회색)·FOV cone(점선)·ego 미래3s(파란 실선)/과거2s(검은 점선).
- **EGO 검수 plot** (우측) — 속도 / 조작값 / yaw_rate / 가속도(G). **x축 = 시간(초)**,
  **빨간 세로선 = 현재 프레임**. 조작값 패널은 throttle·brake(왼쪽축 [0,1])와 **steer(도, 오른쪽
  twin 축)** 를 스케일이 달라 축을 나눠 그린다.

조작 (창을 클릭해 포커스를 준 뒤):

| 키 | 동작 |
|---|---|
| `←` `→` (또는 `a` `d`) | 이전 / 다음 프레임 |
| `Shift+←/→` (또는 `A` `D`) | 10프레임씩 |
| `Home` / `End` | 처음 / 마지막 |
| `s` | 현재 프레임 png 저장 (`dataset/validation/<run_id>/`) |
| `q` | 종료 |

### `av-bev` — 정지 BEV 저장

```bash
av-bev ac_dataset/dataset/<run_id> --frame 350 --view both
#   --frame N   기본 -1 = 중앙 프레임
#   --view      ego | global | both
#   --grid-hz   (해당 없음)  --out DIR  출력 폴더
```
`av-view` 와 달리 창을 안 띄우고 png 로 저장한다(리포트·공유용).

### `av-check` — 물리 일관성 자동 검증 (정량)

게임이 주는 **여러 채널로 같은 물리량을 두 번 구해** 일치하는지 자동으로 판정한다.
사람이 눈으로 보는 `av-view` 와 달리 숫자로 PASS/FAIL 을 낸다.

```bash
av-check ac_dataset/dataset/<run_id>
#   -o DIR   출력 폴더(기본: <run>/validation/)
```

- 산출물: `<run>/validation/physics_report.png` + `summary.json`.
- 리포트는 체크마다 **(measured vs reference 산점도 + residual 히스토그램)** 을 그린다.
  산점도가 `y=x` 에 붙고 residual 이 0 근처면 일치.

검증 항목(전부 **독립 채널쌍** — 후처리가 파생시킨 값끼리 비교하는 순환논리는 배제):

| 체크 | 교차검증 | 기대 |
|---|---|---|
| 자차 속도 ↔ 위치미분 | `speed` vs `world_position` 중앙차분 | 비율 ≈ 1.0 |
| 자차 자세 ↔ 진행방향 | `yaw` vs 위치 진행방향 | |Δ| ≈ 0° |
| 자차 횡가속 ↔ v·yaw_rate | `accelX·g` vs `speed·yaw_rate` | 기울기 ≈ +9.81 |
| 자차 종가속 ↔ d(speed)/dt | `accelY·g` vs 속도 미분 | 상관 > 0.85 |
| 상대차 속도 ↔ 위치미분 | `opp_speedKMH` vs `opp_x/y` 중앙차분 | 비율 ≈ 1.0 |

이상프레임 flag: residual 이 항상 0 근처여야 하는 속도↔위치 체크에서만, `median±6·MAD` 밖
이면서 절대 residual 이 큰 프레임을 `summary.json` 에 남긴다(순간이동/스톨 같은 진짜 글리치).
프레임 번호는 `camera/{frame:06d}.jpg` 와 1:1 이라 바로 열어 확인할 수 있다.

> **단위 주의:** AC 의 `accelX`(횡)·`accelY`(종)는 **m/s² 가 아니라 g 단위**다.
> (기준 run 에서 `v·yaw_rate = 9.76·accelX` 로 확인 → 리포트는 `×9.81` 로 환산해 비교.)

## 카메라 투영 파라미터 (3D bbox)

새 run의 이미지 위 3D 박스는 CSP 브리지가 **각 렌더 프레임에서 직접 기록한 월드 카메라
position/forward/up/FOV**로 투영한다(`projection.py`). 따라서 g-force, shake, horizon lock,
트랙 경사나 카메라 전환으로 카메라가 움직여도 그 움직인 pose 자체가 라벨에 반영된다.

- pose 선택은 이미지와 함께 저장한 AC 게임 랩타임·랩 번호로 한다. 이미지 특징이나 픽셀 좌표는
  선택·보정·박스 계산 어느 단계에서도 사용하지 않는다.
- 새 run의 3D cuboid는 카메라와 같은 CSP 렌더 tick의 차량 full body axes
  (`position/look/up`)와 visual AABB로 만든다. yaw뿐 아니라 pitch/roll과 렌더 모델 원점도
  그대로 반영된다. 이미지에 맞춘 차종별 오프셋은 사용하지 않는다.
- `camera_index.parquet`에 `cam_pos_*`, `cam_forward_*`, `cam_up_*`, `cam_fov_deg`가 있어야
  실제 pose 경로가 켜진다. `av-view` 시작 로그의 `CSP 실제 렌더 pose`로 확인한다.
- 브리지 v2 전의 옛 run은 차량 render state가 없으므로 `yaw_body + colliders.ini`로 fallback한다.
  카메라 pose까지 없는 더 오래된 run은 `cameras.ini + ego pose`도 함께 fallback한다.

> 이미지·telemetry·카메라 pose는 같은 AC 게임시간으로 페어링하므로 기본 `τ=0`이다.
> `--tau-ms`는 게임시간을 기록하지 못한 옛 run을 확인할 때만 쓰는 수동 fallback이다.

## 모듈

```
src/ac_validation/
    loader.py       # run 하나를 RunData(ego/opp/track/치수)로 로드
    bev.py          # BEV 렌더러 (av-bev) — ego/global, 카메라 이미지 나란히
    projection.py   # 상대차 3D cuboid → 카메라 이미지 픽셀 투영
    viewer.py       # 인터랙티브 검증 뷰어 (av-view) — 3패널
    checks.py       # 물리 일관성 체크(채널 교차검증) — CheckResult, run_all_checks
    report.py       # 검증 PNG 리포트 + summary.json + 이상프레임 flag (av-check)
```

## 검증 로드맵

- [x] **시각화 검증** (`av-view`) — 사람이 눈으로: 이미지 재투영, BEV, ego plot, 미래 궤적
- [x] **물리 일관성 자동 체크** (`av-check`) — 위치미분 속도 vs 기록 속도, yaw vs 진행방향,
      가속도 일관성. 채널 교차검증(순환논리 배제), PNG 리포트 + summary.json + 이상프레임 flag
- [ ] **정량 검증** (남음):
  - 재투영 오차(픽셀) 측정 + 히스토그램 (독립 GT 필요)
  - 시간 동기화 오프셋(τ) 측정
