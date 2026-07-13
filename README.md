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

### `av-view` — 인터랙티브 뷰어 (메인)

한 창에 **3패널**을 띄우고 프레임을 넘겨가며 검증한다. **미리 저장하지 않고** 넘길 때마다
그 프레임만 즉석에서 그린다(저장 용량 0). 모든 프레임/모든 run 에 그대로 쓸 수 있다.

```bash
av-view ac_dataset/dataset/<run_id> --frame 350
#   --frame N   시작 프레임(processed 행 index). 기본 0
#   --step N    ←/→ 이동 간격. 기본 1
#   --fov 86.78 수평 화각(도)
```

- **패널 1 (왼쪽): 카메라 이미지 + 상대차 3D bbox** — 상대차의 3D 박스를 이미지에 투영.
  박스가 실제 차에 얹히면 위치·방향·치수·투영이 맞는 것. 화각 밖/너무 가까운 차는 자동 제외.
  제목에 원본 이름(`000350.jpg`)과 export 최종 이름(`12325.jpg`)을 함께 표시.
- **패널 2 (가운데): BEV (ego-centric)** — ego(파랑, "ego" 라벨)·상대차(초록, 번호)·racing
  line(주황)·track edge(회색)·FOV cone(점선)·ego 미래3s(파란 실선)/과거2s(검은 점선).
- **패널 3 (오른쪽): EGO 검수 plot** — 속도 / 조작값(throttle·brake·steer) / yaw_rate / 가속도.
  **x축 = 시간(초)**, **빨간 세로선 = 현재 프레임**.

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

## 카메라 투영 파라미터 (3D bbox)

이미지 투영은 카메라 위치를 근사한다: `projection.py` 의 `cam_height=0.9m`(드라이버 시점 높이),
`pitch=-2°`(살짝 아래). imola/monza 실주행 프레임으로 박스가 차에 얹히도록 보정한 값이다.
**가로(u)·좌우 크기는 BEV 로 이미 검증돼 신뢰도 높고, 세로(v)는 이 근사에 의존**한다.

## 모듈

```
src/ac_validation/
    loader.py       # run 하나를 RunData(ego/opp/track/치수)로 로드
    bev.py          # BEV 렌더러 (av-bev) — ego/global, 카메라 이미지 나란히
    projection.py   # 상대차 3D cuboid → 카메라 이미지 픽셀 투영
    viewer.py       # 인터랙티브 검증 뷰어 (av-view) — 3패널
```

## 검증 로드맵

- [x] **시각화 검증** (`av-view`) — 사람이 눈으로: 이미지 재투영, BEV, ego plot, 미래 궤적
- [ ] **정량 검증** (예정) — 숫자로 자동 판정:
  - 재투영 오차(픽셀) 측정 + 히스토그램
  - 물리 일관성 자동 체크(위치미분 속도 vs 기록 속도, yaw vs 진행방향, 가속도 일관성)
  - 시간 동기화 오프셋(τ) 측정
  - 이상 프레임 자동 flag + bad-case 저장 + `summary.json` 리포트
