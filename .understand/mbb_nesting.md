# MBB Python 백엔드 네스팅 코어 엔진 해부 (mbb_nesting.py)

> **엔진 버전**: v5.22 (초고속 정밀 중력 압축 엔진)  
> **최종 수정 일시**: 2026-10-28 07:22:00  
> **인코딩**: UTF-8  
> **핵심 의존성**: Python 3.14, Shapely (GEOS C-API), concurrent.futures (ProcessPoolExecutor)

---

## 1. 모듈 개요 및 역할 (Overview & Role)
`mbb_nesting.py`는 AutoCAD에서 전달받은 잔재(Scrap Sheets/Containers)와 단품(Parts) 데이터를 바탕으로, **수율 극대화**, **부재 간 0-간섭(Zero-Overlap)**, **경계선 100% 준수**를 보장하며 배치 좌표를 계산하는 핵심 솔버 엔진입니다.

최신 v5.22에서는 기존의 불필요한 다단계 중복 루프를 제거하고 **1.5~3초 이내 즉각 반환하는 초고속 정밀 파이프라인**으로 완결되었습니다.

---

## 2. 핵심 클래스: `EvolutionContainer`

`EvolutionContainer`는 단일 잔재 컨테이너(직사각형, 원형, 다각형)를 추상화하고 부재 배치를 관리하는 기본 단위입니다.

### 주요 속성 (Attributes)
- `id`: 컨테이너 고유 식별자.
- `type`: 컨테이너 형태 (`'rectangle'`, `'box'`, `'circle'`, `'polygon'`).
- `margin`: 외곽 경계 안전 마진 (기본 1.0~2.0mm).
- `gap`: 부재 간 이격 거리 (기본 3.0~5.0mm).
- `placed`: 현재 배치된 부재 리스트 (`[{id, container_id, gx, gy, w, h, rot, area}, ...]`).
- `placed_polys`: 빠른 충돌 검사를 위해 캐싱된 Shapely 다각형 및 사전 계산된 버퍼 객체.
- `placed_area`: 현재까지 배치된 부재들의 실효 면적 합계.

### 주요 메서드 (Key Methods)
1. `can_place(gx, gy, w, h, rot=0, part_poly=None)`:
   - 해당 좌표에 부재 배치 시 기존 부재들과의 충돌 여부 및 컨테이너 경계선 수용 여부를 다단계(BBOX 분리축 -> Shapely 버퍼 간섭)로 고속 판정.
2. `is_inside(gx, gy, w, h, part_poly=None)`:
   - 부재가 컨테이너 외곽 경계선 내부에 100% 온전히 수용되는지 판정. (원형: 중심거리+반경 판정 / 다각형: Shapely `contains` 판정).
3. `find_best_position(pw, ph, allowed_rotations=[0, 90], part_data=None)`:
   - **Bottom-Left Fill (BLF)** 기본 전략 + **Arc-Pocket(초승달/원호 오목부) 집중 앵커링**.
   - 부재를 가능한 한 좌측 하단(Left-Bottom)으로 조밀하게 밀착시키는 최적 (gx, gy, rot) 탐색.
4. `jostle_compaction()`:
   - 배치된 부재들을 아래쪽과 좌측으로 미세 밀착 이동시키는 지터링 압축 루틴.

---

## 3. 핵심 알고리즘 및 실행 파이프라인 (Nesting Pipeline)

```mermaid
flowchart TD
    Start([solve_nesting 진입]) --> Sanitize[JSON 포맷 위생화 (rtos 선행 0 보정)]
    Sanitize --> ModeCheck{컨테이너 수 판정}
    
    ModeCheck -- 단일 컨테이너 --> SinglePipe[solve_single_container_nesting]
    ModeCheck -- 다중 컨테이너 --> GroupCheck{1:1 그룹 좌표 존재 여부}
    
    GroupCheck -- 존재함 --> GroupPipe[solve_grouped_nesting (1:1 매칭)]
    GroupCheck -- 존재하지 않음 --> GAPipe[solve_competitive_evolution (20-Core GA)]
    
    SinglePipe --> NotchCheck{오목 거대 모재 존재 여부?}
    NotchCheck -- Yes (Part 0 등) --> NotchFirst[_solve_notch_first 파이프라인]
    NotchCheck -- No --> StandardNesting[_solve_standard_nesting]
    
    subgraph NotchFirst_Detail [v5.22 초고속 Notch-First 파이프라인]
        Step1[Step 1: 거대 베이스 모재 BLF 좌하단 우선 안착]
        Step2[Step 2: 모재 BBOX 차집합 기반 직사각형 노치 서브캐비티 검출]
        Step3[Step 3: 0.05초 초고속 Greedy 충진으로 노치 내부 소형 부재 우선 수납]
        Step4[Step 4: 잔여 부재 메인 컨테이너 자유 공간 고속 배치]
        Step1 --> Step2 --> Step3 --> Step4
    end
    
    NotchFirst --> NotchFirst_Detail
    NotchFirst_Detail --> Verify[post_verify_and_relax (거대 모재 절대 보존)]
    StandardNesting --> Verify
    GroupPipe --> Verify
    GAPipe --> Verify
    
    subgraph Compaction_Detail [v5.22 3단 정밀 압축 파이프라인]
        Comp1[_gravity_compact: 상단 부재 수직 하향 중력 밀착]
        Comp2[_shake_and_drop_annealing: 좌우 진동+회전으로 오목 호/캐비티 낙하]
        Comp3[_gravity_compact: 최종 잔여 유격 밀착 침전]
        Comp1 --> Comp2 --> Comp3
    end
    
    Verify --> Compaction_Detail
    Compaction_Detail --> Output[결과 LISP 및 JSON 저장]
    Output --> End([완료 (소요 시간 1.5~3.1초)])
```

---

## 4. 특수 전략 및 엔지니어링 최적화 상세

### 4.1 거대 베이스 모재(Part 0) 절대 보호 (`post_verify_and_relax`)
- **문제점**: 종래에는 검증 순서가 무작위여서, 소형 부재가 먼저 등록된 후 거대 모재(Part 0)가 미세 간섭으로 인해 탈락하여 외곽으로 튕겨 나가는 치명적 버그 발생.
- **해결책**:
  ```python
  sorted_plan = sorted(plan, key=lambda it: parts_map.get(it.get('id', it.get('part_id')), {}).get('area', 0), reverse=True)
  ```
  면적이 큰 거대 베이스 모재를 **무조건 1순위로 확정 배치**하고, 간섭 발생 시 소형 부재만 배제하도록 역전 방지.

### 4.2 0.05초 초고속 Greedy 노치 충진 (`_solve_notch_first` Step 3)
- **문제점**: 서브 노치 영역(예: 643×503mm)에 20개 부재 전체를 대상으로 무거운 다중 GA 및 8,000회 Simulated Annealing을 시도하여 30~40초 연산 멈춤 발생.
- **해결책**: 서브 노치는 단순 직사각형(`'type': 'box'`)으로 설정하고, 단일 패스의 고속 Greedy BLF로 들어갈 수 있는 소형 부재(10~11개)만 0.05초 만에 즉시 채워 넣고 탈출.

### 4.3 3단 진동 중력 낙하 압축 (Shake-Annealing & Gravity)
1. **`_gravity_compact`**: 상단에 붕 떠 있는 부재들을 Y축 아래 방향으로 충돌 직전까지 단계적(100mm -> 50mm -> 10mm -> 1mm)으로 수직 하강.
2. **`_shake_and_drop_annealing`**: 부재를 좌우(`±50, ±20, ±10, ±5, ±2, ±1mm`)로 진동시키고 0°/90° 회전을 시도하여, 아래쪽 초승달(102-10)의 오목 호 내부나 노치 틈새로 부재를 쏙 집어넣음.
3. **`_gravity_compact` 재호출**: 진동으로 공간이 생긴 상단 부재들을 다시 한번 최종 침전시켜 상단 절단선 높이를 극적으로 감축.
