"""ac_validation - Assetto Corsa IL 데이터셋 검증/시각화 툴킷.

`ac_dataset` 가 만든 run(dataset/<run_id>/)을 **읽기 전용**으로 심사한다.
수집 파이프라인(ac_dataset)과 분리돼 있어, 이 패키지가 깨져도 수집·학습은 영향 없다.
좌표/투영/제원 로직은 ac_dataset 를 import 해서 재사용한다(중복 구현 금지).

모듈
----
- loader   : run 을 RunData(ego/opp/track/dims/meta) 하나로 로드
- bev      : Bird's-Eye-View 렌더러 (전역 / ego-centric)  → CLI: av-bev
"""

from .loader import RunData, load_run

__all__ = ["RunData", "load_run"]
__version__ = "0.1.0"
