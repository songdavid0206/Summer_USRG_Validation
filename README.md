# ac_validation — AC IL 데이터셋 검증/시각화 툴킷

`ac_dataset` 이 만든 run(`dataset/<run_id>/`)을 **읽기 전용**으로 심사한다.
수집 파이프라인과 분리돼 있어(의존성 한 방향: validation → dataset), 이 패키지가 깨져도
수집·학습은 영향이 없다. 좌표/투영/제원 로직은 `ac_dataset` 를 import 해서 재사용한다.

실제 자율주행 데이터셋 devkit(nuScenes/Argoverse/Waymo)이 하는 검증 방식을 AC 데이터에 맞춰
재구현하는 것을 목표로 한다: BEV 렌더링, 카메라 재투영, 물리 일관성, 시간 동기화, 미래 궤적.

## 설치

```bash
cd ac_dataset && pip install -e .      # 먼저 수집 패키지 (이걸 import 함)
cd ../ac_validation && pip install -e .
```

## 도구

| 명령 | 하는 일 | 상태 |
|---|---|---|
| `av-bev <run>` | Bird's-Eye-View (ego/global) + 카메라 이미지 나란히 | ✅ |
| 재투영 오버레이 | 이미지에 상대차 3D 박스 투영 | 예정 |
| 물리/timing 대시보드 | 속도·yaw·가속도 일관성 + 시간 오프셋 | 예정 |
| 미래 trajectory 뷰어 | ego-centric 미래 waypoint | 예정 |

### `av-bev`

```bash
av-bev ac_dataset/dataset/<run_id> --frame 350 --view both
#   --frame N     프레임 번호(processed 행 index). 기본 -1 = 중앙
#   --view        ego | global | both
#   --fov 86.8    수평 화각(도)
#   --out DIR     출력 폴더 (기본: dataset/validation/<run_id>/bev)
```

- 왼쪽에 그 프레임의 **실제 카메라 이미지**를 나란히 보여주고, 제목에 원본 이름(`000350.jpg`)과
  export 최종 이름(`8750.jpg`)을 함께 표시한다.
- 색: ego(파랑 박스), opponent(초록), retired(빨강), racing line(주황), track edge(회색),
  ego future 3s(파란 실선), ego past 2s(검은 점선), FOV cone(파란 점선).

> **프레임 번호 주의:** `--frame` 은 processed 의 행 index(원본 `camera/000350.jpg`)다.
> export 폴더의 파일 이름은 균일격자 ms(`8750.jpg`)라 숫자가 다르다 — BEV 제목에 둘 다 찍어 헷갈림을 없앴다.
