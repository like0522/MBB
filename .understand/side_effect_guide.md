# MBB 사전 사이드이펙트 분석 가이드 (Pre-Modification Side-Effect Guide)

> **문서 버전**: v5.22  
> **최종 갱신 일시**: 2026-10-28 15:30:00  
> **엔지니어링 표준**: 25년 경력 CAD/AutoLISP 수석 엔지니어 안전 가이드라인

---

## 1. 사전 사이드이펙트 분석 원칙 (Core Principles)
MBB 프로젝트의 코드를 수정하기 전, 아래 6대 위험 요소를 반드시 사전 점검하여 **CAD 화면 멈춤, 한글 깨짐, 부재 겹침, 좌표 왜곡** 등의 결함을 원천 차단합니다.

---

## 2. 6대 핵심 체크리스트 (6-Point Safety Checklist)

### 1) ANSI (CP949) 무손실 저장 원칙
- **위험**: VSCode나 일반 텍스트 에디터가 `MBB.LSP`를 UTF-8로 임의 저장하면 한글 DCL 및 주석이 깨져 AutoCAD에서 문법 에러(`malformed string`)가 발생합니다.
- **방어책**: AutoLISP 관련 파일은 반드시 Python의 `encoding='cp949', errors='replace'` 옵션으로 읽고 저장합니다.

### 2) AutoCAD `rtos` 선행 0 누락 방어
- **위험**: AutoLISP `rtos` 함수는 0.3297을 `.3297`로 출력합니다. 이것이 JSON 문자열에 포함되면 `json.loads`에서 즉각 `JSONDecodeError` 크래시가 발생합니다.
- **방어책**: `mbb_nesting.py`의 `solve_nesting` 진입부 위생화 코드를 절대로 제거하거나 우회하지 마십시오:
  ```python
  raw_json = raw_json.replace(', .', ', 0.').replace('[.', '[0.').replace('-.', '-0.').replace(': .', ': 0.')
  ```

### 3) 거대 베이스 모재(Part 0) 절대 보존 원칙
- **위험**: `post_verify_and_relax` 순서가 뒤섞이면, 면적 80만 이상의 주 모재(700-22 등)가 작은 부재들과의 미세 간섭으로 인해 탈락하여 컨테이너 외곽(`X=477566`)으로 쫓겨나는 참사가 발생합니다.
- **방어책**: 검증 진입 시 반드시 면적 내림차순 정렬(`reverse=True`)을 유지하여 거대 모재를 1순위로 확정하십시오.

### 4) Shapely 연산 오버헤드 및 화면 멈춤(Hang) 방지
- **위험**:
  - `cnt.is_inside`나 `buffer()`를 570개 정점의 다각형에 대해 수만 번 반복 호출하면 연산 시간이 200초를 초과하여 AutoCAD가 "응답 없음"에 빠집니다.
  - `rescue_unplaced_in_container`의 Simulated Annealing을 부재 수가 5개 이상일 때 돌리면 극심한 지연이 발생합니다.
- **방어책**:
  - 사각 박스는 `box(x, y, x+w, y+h)`로 BBOX 선행 필터링.
  - 서브 노치 충진은 다중 GA 대신 0.05초 Greedy BLF 적용.
  - Simulated Annealing은 `len(cnt.placed) <= 3` 및 원형 컨테이너에서만 제한적으로 실행.

### 5) 좌표계 일치성 (WCS vs UCS vs OCS)
- **위험**: 사용자가 UCS를 변경한 상태에서 `vla-get-InsertionPoint`나 `assoc 10` 좌표를 그대로 취하면 WCS와 불일치하여 부재가 엉뚱한 위치로 날아갑니다.
- **방어책**: AutoLISP에서 모든 점 좌표는 반드시 `(trans pt ename 0)` 또는 `(trans pt 1 0)`으로 WCS로 정규화한 뒤 JSON에 기록합니다.

### 6) 단일 Undo 트랜잭션 무결성
- **위험**: 중간에 에러가 발생했을 때 트랜잭션 마크가 닫히지 않으면 사용자의 Undo 스택이 망가집니다.
- **방어책**: 모든 AutoLISP 진입점은 `*error*` 핸들러 내에 `(vla-EndUndoMark acDoc)` 및 시스템 변수 복원을 의무화합니다.

---

## 3. 코드 수정 시 변경 영향도 평가 매트릭스 (Blast Radius Matrix)

| 수정 대상 파일 / 영역 | 직접적인 영향 범위 | 잠재적 사이드이펙트 및 회귀 테스트 |
| :--- | :--- | :--- |
| **`mbb_nesting.py`**<br>`_solve_notch_first` | L자 모재 및 노치 충진 부재 | 소형 부재 수납 누락, 연산 시간 증가<br>👉 `run_notch_test.py` 필히 실행 |
| **`mbb_nesting.py`**<br>`post_verify_and_relax` | 전체 12개 그룹의 최종 배치 | 부재 간 간섭(Overlap) 발생 또는 거대 모재 탈락<br>👉 `test_all_12_groups.py` 필히 실행 |
| **`mbb_nesting.py`**<br>`_gravity_compact` | 상단 부재의 Y 좌표 | 부재 간 수직 침범, 경계선 이탈<br>👉 BBOX Bounding Check 필히 확인 |
| **`MBB.LSP`**<br>`c:MBBN` | CAD 객체 이동 및 회전 | 텍스트 분리, 도면 레이어 오염, 괄호 불일치<br>👉 AutoCAD 내 단독 로드 테스트 |
