# MBB 시스템 코드 지능 지도 (Code Intelligence Map)

> **버전**: v5.22  
> **최종 갱신 일시**: 2026-10-28 15:30:00  
> **기준 언어**: 한국어 (`--language ko`)  
> **설계 표준**: 25년 경력 CAD/AutoLISP & 3D CAD 수석 엔지니어 규격

---

## 1. 프로젝트 개요 (Overview)
**MBB (Minimum Bounding Box & Nesting Engine)**는 AutoCAD 환경에서 조선/해양/플랜트용 강판 부재(Plate Parts)와 잔재(Scrap Sheets)를 정밀 분석하고, 고성능 2D 판재 네스팅(Nesting)을 전자동으로 수행하는 엔지니어링 솔루션입니다.

- **AutoCAD 프론트엔드 (`MBB.LSP`)**: 객체 선택, 정밀 기하/원호(Arc) 추출, DCL 대화상자 UI, WScript.Shell을 통한 Python 엔진 구동, 결과 LISP 로드 및 CAD 엔티티 자동 변환.
- **Python 고성능 백엔드 (`mbb_nesting.py`)**: 20-Core 병렬 유전 알고리즘(GA), Shapely 기반 기하 연산, L자 노치 최우선 서브캐비티 충진, 진동 중력 낙하(Shake-Annealing), 0-오차 간섭/경계 검증.

---

## 2. 코드 지도 디렉토리 구조 (.understand/)

| 문서 파일 | 주요 내용 및 목적 | 토큰 참조 우선순위 |
| :--- | :--- | :---: |
| [**index.md**](file:///c:/Users/like/OneDrive/문서/MBB/.understand/index.md) | 코드 지능 지도 메인 허브, 프로젝트 요약, 핀포인트 컨텍스트 가이드 | 1순위 (필수) |
| [**architecture.md**](file:///c:/Users/like/OneDrive/문서/MBB/.understand/architecture.md) | AutoCAD ↔ Python 오프로드 아키텍처, JSON 스키마, 통신 파이프라인 | 2순위 (연동 작업 시) |
| [**mbb_nesting.md**](file:///c:/Users/like/OneDrive/문서/MBB/.understand/mbb_nesting.md) | Python 백엔드 네스팅 코어 엔진 완벽 해부 (v5.22 정밀 압축 알고리즘) | 2순위 (네스팅 수정 시) |
| [**mbb_lsp.md**](file:///c:/Users/like/OneDrive/문서/MBB/.understand/mbb_lsp.md) | AutoLISP 프론트엔드 명령어(MBBN, MBBG, MBB 등), DCL, 트랜잭션 | 2순위 (CAD/UI 수정 시) |
| [**functions_map.md**](file:///c:/Users/like/OneDrive/문서/MBB/.understand/functions_map.md) | 전체 함수/클래스 카탈로그, 시그니처, 파라미터, 의존성 관계도 | 3순위 (함수 핀포인트 로딩 시) |
| [**side_effect_guide.md**](file:///c:/Users/like/OneDrive/문서/MBB/.understand/side_effect_guide.md) | 사전 사이드이펙트 분석 가이드 및 작업 안전 체크리스트 | 3순위 (코드 수정 전 필독) |

---

## 3. 핵심 파일 맵 (Core Source Files)

```
c:\Users\like\OneDrive\문서\MBB\
├── MBB.LSP                     # [AutoLISP/DCL] CAD 프론트엔드 통합 솔루션 (ANSI CP949)
├── mbb_nesting.py              # [Python 3.14] 20-Core 초고속 기하 네스팅 백엔드 코어 (UTF-8)
├── mbbg_align.py               # [Python] Group-Box 정렬 및 BBOX 정밀 회전 연산
├── test_all_12_groups.py       # [Test] 12개 실무 복합 그룹 종합 회귀 검증기 (29개 부재)
├── run_notch_test.py           # [Test] L자 노치(Concave Giant) 100% 안착 회귀 검증기 (11개 부재)
├── DEBUG_nesting_input.json    # [Fixture] 실제 대형 프로젝트 21개 복합 부재 디버그 데이터
└── .understand/                # [Knowledge] 한국어 코드 지능 지도 (.understand)
```

---

## 4. 토큰 최적화 핀포인트 로딩 규칙 (Token Optimization Protocol)
불필요하게 130KB에 달하는 전체 소스코드를 context에 로드하지 않고, 작업 성격에 맞추어 아래 표의 핵심 지점만 로드합니다.

1. **네스팅 알고리즘 및 압축 수정 시**:
   - `mbb_nesting.py` 내 `_solve_notch_first` (라인 2212~2415)
   - `_gravity_compact` 및 `_shake_and_drop_annealing` (라인 1460~1650)
   - `post_verify_and_relax` (라인 1163~1250)
2. **AutoCAD 객체 추출 및 JSON 연동 수정 시**:
   - `MBB.LSP` 내 `c:MBBN` (라인 2479~3120)
   - `MBB:GetPartContourEx` (원호/Bulge 정밀 추출 루틴)
3. **사이드이펙트 방지**:
   - 작업 착수 전 [**side_effect_guide.md**](file:///c:/Users/like/OneDrive/문서/MBB/.understand/side_effect_guide.md)를 먼저 확인하여 WCS 좌표계, JSON 포맷 깨짐, CP949 인코딩 무손실 저장 원칙을 준수합니다.
