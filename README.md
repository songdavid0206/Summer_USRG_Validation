# Summer_USRG Validation

`Summer_USRG`가 수집한 Assetto Corsa 원본 run을 **읽기 전용으로 검증**하는 독립
툴킷이다. 이미지 위 3D bbox, world/ego BEV, 시계열과 물리 채널 교차검증을 통해
수집·동기화·좌표 변환이 실제 렌더와 일치하는지 확인한다.

이 저장소는 데이터 수집이나 라벨 수정 작업을 하지 않는다.

## 저장소 관계

| 저장소 | 역할 |
|---|---|
| [Summer_USRG](https://github.com/songdavid0206/Summer_USRG) | 화면·telemetry·CSP 상태 수집, 정렬, structured export, 분포 분석 |
| **Summer_USRG_Validation** (현재 저장소) | 원본 run의 시각적·기하학적·물리적 독립 검증 |

의존성은 한 방향이다.

```text
Summer_USRG dataset/<run_id>
            ↓ read only
Summer_USRG_Validation
  ├─ av-view   대화형 프레임 검사
  ├─ av-check  정량 물리 검사
  └─ av-bev    정지 이미지 생성
```

## 현재 검증 범위

- CSP 실제 렌더 카메라 position/forward/up/FOV 기반 3D bbox 재투영
- same-render 상대차 full 6DoF pose와 visual AABB 사용
- AC 게임시간과 graphics packet/CSP render-frame 동기화 확인
- world map, ego-centric BEV, racing line, 트랙 경계와 상대차 표시
- ego 미래 3초/과거 2초 궤적
- 속도·자세·가속도의 독립 채널 교차검증
- 상대차 기록 속도와 위치 미분 속도 비교
- retirement/invalid 상대차 제외와 이상 프레임 flag

이미지 검출 결과에 GT 박스를 맞추지 않는다. 수집된 3D 렌더 상태만으로 먼저 bbox를
계산하고, RGB 이미지는 그 결과가 실제 화면과 일치하는지 확인하는 독립 자료로 사용한다.

## 설치

두 저장소가 같은 상위 폴더에 있다고 가정한다.

```powershell
cd Summer_USRG
python -m pip install -e ".[capture]"

cd ..\Summer_USRG_Validation
python -m pip install -e .
```

`ac_validation`은 `ac_dataset`의 좌표·트랙·차량 제원 로직을 재사용하므로
`Summer_USRG`를 먼저 설치해야 한다. 새 CLI가 추가된 경우 validation 저장소에서
`pip install -e .`를 다시 실행한다.

## 빠른 시작

상위 폴더에서:

```powershell
av-check Summer_USRG\dataset\<run_id>
av-view  Summer_USRG\dataset\<run_id> --frame 0
```

두 로컬 폴더 이름이 `ac_dataset`, `ac_validation`이라면:

```powershell
av-check ac_dataset\dataset\<run_id>
av-view  ac_dataset\dataset\<run_id> --frame 0
```

### dataset_v2 multi 세션 뷰어

원본 run 없이 배포된 v2 multi ZIP은 먼저 Windows 탐색기에서 **모두 압축 풀기**를 한다.
세션 최상위에 `FRAME_INDEX.csv`, `image_index.parquet`,
`render_car_state.parquet`, `TRACK_INFO`가 있는 render-metadata-enriched 세션만 지원한다.
기존 원본용 `av-view`와 별도 명령이므로 같은 환경에 함께 설치해도 충돌하지 않는다.

```powershell
av-view-v2 ac_dataset\dataset_v2\multi\<session> --frame 3000
```

모듈로 직접 실행할 수도 있다.

```powershell
python -m ac_validation.viewer_v2 ac_dataset\dataset_v2\multi\<session> --frame 3000
```

v2의 `steer_angle`은 degree가 아닌 `[-1, 1]` 정규화 조작값이므로 controls 패널에
`steer(norm)`으로 표시한다. v2 파티션의 `INTERACTIONS` 중첩 프레임과
`NON_INTERACTION` 프레임은 `time_ms`로 중복 제거하여 원래의 연속 세션 순서로 읽는다.

기존 `av-check`와 `av-view`의 입력은 다음 구조를 가진 **원본 run**이다.
`dataset_v2`에는 위의 별도 `av-view-v2` 명령을 사용한다.

```text
dataset/<run_id>/
├─ camera/
├─ ego_state/
├─ opponent/
├─ processed/
├─ track/
└─ meta.json
```

## 명령 요약

| 명령 | 용도 | 출력 |
|---|---|---|
| `av-view <run>` | 이미지·bbox·BEV·시계열을 프레임별로 대화형 검사 | 필요할 때 현재 화면 PNG |
| `av-check <run>` | 물리 채널 교차검증과 이상 프레임 탐지 | `physics_report.png`, `summary.json` |
| `av-bev <run>` | 지정 프레임의 ego/global BEV 저장 | PNG |

### `av-view`

```powershell
av-view ac_dataset\dataset\<run_id> --frame 350
```

옵션:

| 옵션 | 의미 |
|---|---|
| `--frame N` | 시작 processed frame index, 기본 `0` |
| `--step N` | 좌우 이동 간격, 기본 `1` |
| `--fov DEG` | FOV 수동 override, 실제 CSP FOV가 있으면 보통 생략 |
| `--camera N` | fallback onboard camera 번호 |
| `--tau-ms N` | 게임시간이 없는 구 run의 수동 시간 보정 |

화면 구성:

- **Camera + 3D bbox:** 실제 이미지에 상대차 3D cuboid 재투영
- **World map:** 전체 트랙에서 ego·상대차·지나온 경로 확인
- **Ego BEV:** racing line, track edge, FOV, 상대차와 미래/과거 궤적
- **Physics mini-plots:** 현재 run의 채널별 일관성 결과
- **Ego timeline:** 속도, throttle/brake/steer, yaw rate와 가속도

조작:

| 키 | 동작 |
|---|---|
| `←`, `→` 또는 `a`, `d` | 이전/다음 프레임 |
| `Shift+←`, `Shift+→` 또는 `A`, `D` | 50프레임 이동 |
| `Home`, `End` | 첫/마지막 프레임 |
| `s` | 현재 화면 저장 |
| `q` | 종료 |

### `av-check`

```powershell
av-check ac_dataset\dataset\<run_id>
```

출력 위치를 지정하려면:

```powershell
av-check ac_dataset\dataset\<run_id> -o validation_output
```

검증 항목:

| 검사 | 측정 채널 | 독립 기준 채널 | 정상 기대 |
|---|---|---|---|
| ego 속도 | telemetry speed | world position 중앙차분 | 비율 약 1 |
| ego 방향 | body yaw | 위치 진행 방향 | 각도 residual 약 0° |
| ego 횡가속 | `accelX × g` | `speed × yaw_rate` | 선형 일치 |
| ego 종가속 | `accelY × g` | `d(speed)/dt` | 높은 상관 |
| 상대차 속도 | opponent speed | opponent position 중앙차분 | 비율 약 1 |

`summary.json`에는 PASS/FAIL, 상관·기울기·오차 통계와 이상 프레임이 들어간다.
속도↔위치 residual이 `median ± 6·MAD`를 넘으면서 절대 오차도 큰 프레임을
순간이동·스톨 후보로 기록한다.

### `av-bev`

```powershell
av-bev ac_dataset\dataset\<run_id> --frame 350 --view both
```

`--view`는 `ego`, `global`, `both` 중 하나다. GUI 없이 정적 PNG만 필요할 때 사용한다.

## 3D bbox와 동기화 원리

### 카메라

각 이미지 프레임에 저장된 다음 값을 사용한다.

```text
cam_pos_*
cam_forward_*
cam_up_*
cam_fov_deg
```

forward/up을 정규직교화하여 world→camera extrinsic을 만들고, 실제 이미지 크기와
수직 FOV로 intrinsic을 계산한다. 고정 yaw/pitch/roll 보정값을 덧붙이지 않는다.

### 상대차

최신 run에서는 카메라와 같은 CSP render tick의 상대차:

- position
- forward/up
- visual AABB

를 이용해 full 6DoF cuboid를 만든다. 따라서 경사, 뱅크와 연석에서 실제 차체가
pitch/roll하면 bbox도 함께 기울어진다.

### 시간 정합

이미지, telemetry, 카메라와 차량 렌더 상태는 AC 게임시간으로 페어링한다.
graphics packet과 CSP render frame 관계는 same-render 후보를 고르는 추가 증거다.

우선 확인할 잔차:

```text
cam_pose_game_delta_ms
cam_packet_render_error
car_state_age_ms
```

최신 run에서는 이 값들이 0에 가깝고 연속적이어야 한다. `--tau-ms`는 게임시간이 없는
구 run을 위한 fallback이며 최신 run에서는 기본 `0`을 유지한다.

## 결과 해석 주의사항

- 기울어진 bbox가 항상 오류는 아니다. full 6DoF 차체가 경사를 따라 움직일 수 있다.
- amodal wireframe은 가려진 모서리도 그리므로 먼 차량 두 대가 겹쳐 보일 수 있다.
  이때 world/BEV에서 차량 수와 위치를 함께 확인한다.
- `retired=True` 또는 `valid=False`인 상대차는 학습 후보 bbox에서 제외한다.
- AC의 `accelX`, `accelY`는 m/s²가 아니라 g 단위이므로 검사에서 `×9.81`로 환산한다.
- 좋은 재투영은 이미지 fitting의 결과가 아니라 독립 GT가 RGB와 일치한다는 증거여야 한다.

## 모듈

```text
src/ac_validation/
├─ loader.py       # 원본 run 로더
├─ projection.py   # 3D cuboid → 이미지 픽셀
├─ bev.py          # ego/global BEV 렌더러
├─ viewer.py       # 대화형 통합 검증 UI
├─ checks.py       # 물리 일관성 검사
└─ report.py       # PNG/JSON 정량 리포트
```

## 테스트

게임과 데이터셋 없이:

```powershell
python -m unittest discover -s tests -v
```

또는:

```powershell
python -m pytest -q
```

## 범위와 다음 단계

완료:

- 이미지 재투영, world/BEV와 시계열 시각 검사
- ego/opponent 물리 일관성 검사
- 실제 CSP 카메라 pose와 same-render 차량 상태 사용
- PNG 리포트와 이상 프레임 JSON

후속 작업:

- 여러 run을 한 번에 비교하는 batch 품질 리포트
- RGB black frame, blur, desktop/UI 노출과 perceptual duplicate 검사
- occlusion/truncation 등 학습용 visibility 품질 검사

분포 분석과 run/lap 비교는 수집 저장소의
[`analysis/lagunaseca_distribution`](https://github.com/songdavid0206/Summer_USRG/tree/main/analysis/lagunaseca_distribution)
에서 관리한다.
