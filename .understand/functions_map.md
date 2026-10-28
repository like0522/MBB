# MBB 시스템 함수 및 심볼 카탈로그 (Functions & Symbols Map)

> **문서 버전**: v5.22  
> **최종 갱신 일시**: 2026-10-28 15:30:00  
> **인코딩**: UTF-8

---

## 1. Python 백엔드 함수 카탈로그 (`mbb_nesting.py`)

| 함수 / 클래스명 | 주요 인자 (Parameters) | 반환값 (Returns) | 역할 및 설명 |
| :--- | :--- | :--- | :--- |
| **`class EvolutionContainer`** | `(cnt, gap=5.0, margin=1.0)` | 인스턴스 | 단일 잔재 컨테이너(박스/원/다각형) 모델링, 경계 판정, BLF 및 Arc 앵커 탐색 |
| **`solve_nesting`** | `(json_in, lsp_out, json_out, mode='fast')` | `list` (배치 플랜) | **메인 진입점**: JSON 위생화, 단일/다중 컨테이너 라우팅, 결과 파일 저장 |
| **`solve_single_container_nesting`** | `(cnt, c_parts, gap=5.0, margin=1.0, mode='fast')` | `list` (배치 플랜) | 단일 컨테이너 전용 파이프라인. L자 노치 최우선 전략 또는 일반 네스팅 분기 |
| **`_solve_notch_first`** | `(cnt, all_parts, concave_parts, other_parts, gap, margin)` | `list` (배치 플랜) | **v5.22 초고속 L자 노치 솔버**: 모재 좌하단 배치 -> 노치 서브캐비티 0.05s 충진 -> 잔여 부재 배치 |
| **`_solve_standard_nesting`** | `(cnt, c_parts, gap=5.0, margin=1.0)` | `list` (배치 플랜) | 표준 GA 및 다중 시드(Bigness, Area, Aspect) 기반 BLF 네스팅 |
| **`solve_grouped_nesting`** | `(cnts, parts, gap=5.0, margin=1.0, mode='fast')` | `list` (배치 플랜) | 공간 좌표 및 container_id 기반 1:1 컨테이너 그룹 매칭 배치 |
| **`solve_competitive_evolution`** | `(cnts, parts, gap=5.0, margin=1.0, mode='fast')` | `list` (배치 플랜) | 다중 컨테이너 간 수율 경쟁 유전 알고리즘 (20-Core 병렬 프로세스 풀) |
| **`post_verify_and_relax`** | `(plan, containers, parts_data=None, gap=5.0, margin=1.0)` | `list` (검증 플랜) | **0-오차 간섭/경계 검증기**: 거대 모재(Part 0) 1순위 최우선 안착 보장 |
| **`_gravity_compact`** | `(plan, cnt, parts_data, gap=5.0, margin=1.0)` | `list` (압축 플랜) | 상단 부재들을 수직 하향(Y축 음수 방향)으로 유격 한계까지 중력 밀착 |
| **`_shake_and_drop_annealing`** | `(plan, cnt, parts_data, gap, margin, max_passes=2)` | `list` (낙하 플랜) | 좌우 진동(±50~1mm)+회전으로 오목 호(Arc Cavity) 및 노치 틈새로 부재 수직 낙하 |
| **`_compact_minimize_height`** | `(plan, cnt, parts_data, gap=5.0, margin=1.0)` | `list` (최종 플랜) | 전체 배치 상단 절단선 높이(Strip Height)를 최소화하는 정밀 압축 |
| **`rescue_unplaced_in_container`** | `(cnt, unplaced_part, gap=5.0)` | `bool` (성공 여부) | 미배치된 부재를 기존 배치 틈새에 구제하는 루틴 (소형 집합 전용) |
| **`get_transformed_poly`** | `(part_data, gx, gy, rot)` | `Polygon` or `None` | 단품 부재를 해당 위치/각도로 변환한 Shapely Polygon 생성 (캐싱 연동) |

---

## 2. AutoLISP 프론트엔드 함수 카탈로그 (`MBB.LSP`)

| 함수명 | 인자 (Arguments) | 역할 및 설명 |
| :--- | :--- | :--- |
| **`c:MBBN`** | `()` | **네스팅 메인 명령어**: 도면 객체 선택, JSON 생성, Python 오프로드, 결과 로드 및 CAD 이동 |
| **`c:MBBG`** | `()` | **Group-Box 정렬 명령어**: 객체 선택, 정렬 계산 오프로드, 사각 박스 도면 배치 |
| **`c:MBB`** | `()` | 최소 경계 상자(BBOX) 산출 및 도면 텍스트 표기 |
| **`c:MBB2`** | `()` | 두께 입력형 철판 정밀 BBOX 및 단위 중량 자동 계산 |
| **`c:MBBC`** | `()` | Welzl 알고리즘 기반 최소 외접원(MEC) 산출 |
| **`c:PCLOSE`** | `()` | 미세하게 열린 폴리선 일괄 강제 닫힘(Closed) 처리 |
| **`MBB:GetPartContourEx`** | `(ename / ...)` | 폴리선에서 원호(Bulge)를 정밀 샘플링하여 다각형 정점 리스트로 반환 |
| **`MBB:FindClosestContainerId`** | `(pt containers / ...)` | 부재의 중심 좌표와 가장 가까운 컨테이너 식별자(ID) 1:1 매칭 |
| **`MBB:ForceClosePolyline`** | `(ename / ...)` | 단일 폴리선의 시작점과 끝점을 허용오차 내에서 결합 및 플래그 1 설정 |
| **`MBB:CalcSmartNetWeight`** | `(area thk / ...)` | 면적($mm^2$)과 두께($mm$)를 바탕으로 비중 7.85를 적용하여 중량($kg$) 산출 |
| **`LM:ExtractPointsHighPrecision`** | `(ent / ...)` | Lee Mac 라이브러리 기반 고정밀 정점 좌표 추출기 |
| **`LM:Welzl`** | `(pts / ...)` | $O(N)$ 기하학적 최소 외접원 계산 서브루틴 |

---

## 3. 핵심 데이터 구조 및 변수 (Global Variables)

- `*MBBN_MARGIN*`: AutoCAD 세션 내 유지되는 컨테이너 외곽 여백 기본값 (초기값: 2.0mm).
- `*MBBN_GAP*`: AutoCAD 세션 내 유지되는 부재 간 이격 거리 기본값 (초기값: 3.0mm).
- `MBB_NESTING_PLAN`: `mbb_nesting_out.lsp` 로드 시 생성되는 배치 결과 리스트.
  - 형식: `((part_id container_id gx gy w h rot parent_id) ...)`
- `MBB_UNPLACED_PLAN`: 컨테이너 용량 초과로 인해 외곽에 대기 배치된 부재 리스트.
- `MBB_TOTAL_YIELD`: 실효 잔재 컨테이너 수율 (%).
- `MBB_PART_YIELD`: 총 단품 면적 대비 소모 수율 (%).
